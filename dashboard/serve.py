#!/usr/bin/env python3
"""
Agrobot Robot teleoperation dashboard server.

  GET  /api/gnss              — latest GNSS fix from gnss_coords.json
  GET  /api/camera            — latest camera frame as JPEG (polled by browser)
  GET  /api/zed               — latest front camera frame as JPEG (ZED, if available)
  GET  /api/detection         — latest YOLOv8 annotated frame
  GET  /api/detection/data    — latest detection JSON
  POST /api/cmd_vel           — publish wheel speed commands via rclpy

Usage:
    python3 dashboard/serve.py [--port 8766] [--gnss /tmp/gnss_coords.json]
"""
import argparse
import json
import logging
import os
import posixpath
import socket
import socketserver
import sys
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
from datetime import datetime, timezone
from http.server import SimpleHTTPRequestHandler
from pathlib import Path

# Chassis abstraction (agrobot | jackal). serve.py runs as `python3 dashboard/serve.py`,
# so its own directory is on sys.path; the tests insert dashboard/ as well.
try:
    import chassis
except ImportError:                       # imported as a package (dashboard.serve)
    from dashboard import chassis

# PLC command allow-lists (import-safe — plc_client lazy-imports grpc, so this never
# pulls in grpc at module load and tests run without it).
try:
    from plc_client import (SEQUENCE_COMMANDS, MACHINE_COMMANDS, ROBOT_COMMANDS,
                            PLC_TAG_MAP, symbol_roles)
except ImportError:
    from dashboard.plc_client import (SEQUENCE_COMMANDS, MACHINE_COMMANDS, ROBOT_COMMANDS,
                                      PLC_TAG_MAP, symbol_roles)

# Pure business logic (importing chassis above put the repo root on sys.path).
from agrobot_dashboard.domain import auto_drive, geo, kinematics

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%Y-%m-%dT%H:%M:%S",
)
log = logging.getLogger("dashboard")

# Browser-visible event log — agrobot_dashboard.services.events; served via
# GET /api/events?since=<unix_ts>.
from agrobot_dashboard.services.events import events_since, log_event
from agrobot_dashboard.adapters.cloud import push_seedling
from agrobot_dashboard.adapters import cameras
from agrobot_dashboard.adapters.cameras import STREAM_FPS
from agrobot_dashboard.services.detection import draw_box as _draw_box
from agrobot_dashboard.services.recording import recording_loop


DASHBOARD_DIR        = Path(__file__).parent

GNSS_FILE_DEFAULT    = "/tmp/gnss_coords.json"

# Agrobot robot speed parameters
# Manual example: [500,500] = conservative go-ahead, recommended range 300~800.
# Joystick at full throttle (0.5 m/s physical max) sends ~3000 units.
# LINEAR_SCALE maps dashboard m/s → Modbus speed units.
SPEED_MAX     = 32767  # INT16 max — let the Modbus master slider reach full range
SPEED_DEFAULT = 500

# 1.0 m/s → 3000 Modbus units  (Normal preset hits physical max ~0.5 m/s)
LINEAR_SCALE  = 3000   # (linear_x m/s) * LINEAR_SCALE  = base speed units
ANGULAR_SCALE = 1000   # (angular_z rad/s) * ANGULAR_SCALE = differential units

# Hard server-side ceiling on accepted velocity commands (before unit conversion).
# Allow up to INT16_MAX / LINEAR_SCALE so the Modbus master slider can reach 32767 units.
MAX_LIN_INPUT  = 15.0  # m/s — 15 * 3000 = 45000 units, clamped to SPEED_MAX anyway
MAX_ANG_INPUT  = 15.0  # rad/s
MAX_POST_BYTES = 65_536  # 64 KB upper limit on any POST body

PULSE_PER_M      = 3211.0              # encoder pulses per metre (calibrated)
FWD_2M_PULSES    = 2.0  * PULSE_PER_M  # 6422 pulses for the 2 m auto-drive
FWD_SLOW_PULSES  = 0.5  * PULSE_PER_M  # last 0.5 m: drop to slow speed to kill coasting
FWD_SLOW_SPEED   = 0.08                # m/s for final-approach crawl (~250 Modbus units)


def twist_to_wheel_speeds(linear_x: float, angular_z: float):
    """Convert Twist (m/s, rad/s) to [left, right] wheel speed integers.

    Differential drive convention (matching teleop_keyboard.py):
      forward  W: [+speed, +speed]
      backward S: [-speed, -speed]
      left     A: [-speed, +speed]   (positive angular_z = turn left in ROS)
      right    D: [+speed, -speed]
    """
    ch = Handler.chassis
    if ch is not None:
        # The active chassis's scaling (config/chassis/*.yaml) is the single
        # source of truth; the module constants are only the fallback for
        # chassis-less use (unit tests, standalone import).
        return ch.twist_to_wheel_speeds(linear_x, angular_z)
    return kinematics.twist_to_wheel_speeds(
        linear_x, angular_z, LINEAR_SCALE, ANGULAR_SCALE, SPEED_MAX)


_dms = geo.dms


# ---------------------------------------------------------------------------
# Runtime state — one thread-safe store shared by capture threads, ROS
# callbacks and HTTP request threads (agrobot_dashboard.services.telemetry).
# ---------------------------------------------------------------------------
from agrobot_dashboard.services.telemetry import TelemetryStore

TELEM = TelemetryStore(settings_defaults={
    'maxLinear':   2.0,    # absolute max forward speed m/s (= Fast preset)
    'maxAngular':  0.5,    # max turn rate rad/s
    'modbusSpeed': 1500,   # raw Modbus units for Normal preset (mirrors slLinear/4 * LINEAR_SCALE)
    'seedlingType': '',    # species label appended to every planted-seedling record
    'driveDistance': 2.0,  # user-configurable Fwd/Bwd auto-drive distance (m); source of truth for /api/fwd2m
})

# Bounds for the configurable Fwd/Bwd auto-drive distance (metres).
DRIVE_DISTANCE_MIN = 0.1
DRIVE_DISTANCE_MAX = 20.0


def drive_distance(direction, speed, meters=2.0, cancel=None):
    """Server-side fixed-distance auto-drive shared by /api/fwd2m and the
    battery-test auto-cycle. Runs a tight 50 Hz encoder loop (speed has no
    effect on accuracy) and blocks until the target distance is reached, an
    external velocity command overrides us, `cancel()` returns True, or a 30 s
    deadline elapses. Writes velocity through TELEM.vel like a teleop command,
    so the same deadman applies. Returns
    {done, aborted, traveled_m[, error]} — error="encoder" if odom is stale.

    `direction`: "forward" | "backward". `cancel`: optional zero-arg predicate
    polled each tick so a caller (the battery test) can abort mid-drive.
    """
    sign  = -1.0 if direction == "backward" else 1.0
    speed = max(0.05, min(MAX_LIN_INPUT, abs(speed)))

    # Encoder calibration comes from the active chassis config; the module
    # constant is only the fallback when no chassis is loaded (tests).
    ppm = Handler.chassis.pulse_per_m if Handler.chassis is not None else PULSE_PER_M
    target_pulses = meters * ppm
    slow_pulses   = 0.5 * ppm   # final 0.5 m: crawl to kill coasting

    start_l, start_r, last_odom, _ = TELEM.odom.snapshot()
    if last_odom == 0 or (time.monotonic() - last_odom) > 3.0:
        return {"done": False, "aborted": False, "traveled_m": 0.0, "error": "encoder"}

    cmd_speed = sign * speed   # what we last wrote to vel.lin (signed)
    TELEM.vel.set_command(cmd_speed, 0.0)

    deadline = time.monotonic() + 30.0
    traveled = 0.0
    aborted  = False

    try:
        while time.monotonic() < deadline:
            time.sleep(0.02)                   # 50 Hz encoder check

            if cancel is not None and cancel():
                aborted = True
                break

            cur_l, cur_r, _, _ = TELEM.odom.snapshot()
            # Distance travelled in the *commanded* direction (always ≥0 as
            # the robot progresses), so the planner is direction-agnostic.
            traveled = sign * (((cur_l - start_l) + (cur_r - start_r)) / 2.0)

            next_speed = auto_drive.plan_speed(
                traveled, target_pulses, slow_pulses, speed, FWD_SLOW_SPEED)
            if next_speed is None:   # target distance reached
                break

            with TELEM.vel.lock:
                # Abort if an external command overrode our last commanded speed
                if abs(TELEM.vel.lin - cmd_speed) > 0.01:
                    aborted = True
                    break
                TELEM.vel.lin  = sign * next_speed
                TELEM.vel.last = time.monotonic()   # keep deadman alive
            cmd_speed = sign * next_speed
    finally:
        TELEM.vel.set_command(0.0, 0.0)   # single stop, then the deadman idles

    done = (not aborted) and (traveled >= target_pulses)
    return {"done": done, "aborted": aborted, "traveled_m": round(traveled / ppm, 3)}


# ---------------------------------------------------------------------------
# HTTP handler
# ---------------------------------------------------------------------------
class Handler(SimpleHTTPRequestHandler):
    gnss_file: str   = GNSS_FILE_DEFAULT
    speed_cmd_pub    = None   # rclpy publisher, set by main() after rclpy.init()
    chassis          = None   # active chassis.Chassis, set by main(); None in tests
    plc              = None   # plc_client.PlcClient, set by main() on plc-enabled chassis
    telemetry        = TELEM  # the TelemetryStore — ALL mutable runtime state lives there

    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(DASHBOARD_DIR), **kwargs)

    # ── Declarative routing ──────────────────────────────────────────────────
    # Exact paths are matched first, then prefixes longest-first — so route
    # registration order can never shadow another route (the old if/elif
    # ladders silently depended on their ordering).
    #
    # A route spec is a method-name string, a (method-name, *args) tuple, or a
    # plain callable taking the handler instance (used by extensions).
    INDEX_FILE = "index.html"   # what "/" serves; overridden by serve_plc.py
    STATIC_PREFIXES = ("/logo/", "/js/")

    GET_EXACT = {
        "/api/wheel_odom":         "_serve_wheel_odom",
        "/api/chassis_battery":    "_serve_chassis_battery",
        "/api/gnss":               "_serve_gnss",
        "/api/camera/status":      "_serve_camera_status",
        "/api/zed/status":         "_serve_zed_status",
        "/api/detection/data":     "_serve_detection_data",
        "/api/detection/rear_data": "_serve_rear_detection_data",
        "/api/settings":           "_serve_settings_get",
        "/api/config":             "_serve_config",
        "/api/plc/status":         ("_serve_plc_read", "get_machine_status"),
        "/api/plc/sequence":       ("_serve_plc_read", "get_sequence_detail"),
        "/api/plc/auger_motor":    ("_serve_plc_read", "get_auger_motor_status"),
        "/api/plc/banner":         ("_serve_plc_read", "get_banner"),
        "/api/plc/tags":           "_serve_plc_tags",
    }
    GET_PREFIX = {
        "/api/detection/rear_stream": "_serve_rear_detection_stream",
        "/api/detection/stream":      "_serve_detection_stream",
        "/api/camera/stream":         "_serve_camera_stream",
        "/api/zed/stream":            "_serve_zed_stream",
        "/api/detection":             "_serve_detection_image",
        "/api/events":                "_serve_events",
        "/api/camera":                "_serve_camera",
        "/api/zed":                   "_serve_zed",
    }
    POST_EXACT = {
        "/api/cmd_vel":      "_serve_cmd_vel",
        "/api/track/save":   "_serve_track_save",
        "/api/plant":        "_serve_plant_log",
        "/api/record/start": "_serve_record_start",
        "/api/record/stop":  "_serve_record_stop",
        "/api/settings":     "_serve_settings_post",
        "/api/fwd2m":        "_serve_fwd2m",
        "/api/plc/auger":    ("_serve_plc_sequence", "control_auger"),
        "/api/plc/planter":  ("_serve_plc_sequence", "control_planter"),
        "/api/plc/both":     ("_serve_plc_sequence", "control_both"),
        "/api/plc/machine":  ("_serve_plc_command", "machine_command"),
        "/api/plc/robot":    ("_serve_plc_command", "control_robot"),
    }

    @classmethod
    def add_route(cls, method, path, spec, prefix=False):
        """Register an extra route (used by serve_plc.py instead of the old
        monkey-patching of do_GET/do_POST). `spec` may be a Handler method
        name or a callable taking the handler instance."""
        if method == "GET":
            (cls.GET_PREFIX if prefix else cls.GET_EXACT)[path] = spec
        elif method == "POST":
            cls.POST_EXACT[path] = spec
        else:
            raise ValueError(f"unsupported method {method!r}")

    def _run_route(self, spec):
        if callable(spec):
            spec(self)
        elif isinstance(spec, tuple):
            getattr(self, spec[0])(*spec[1:])
        else:
            getattr(self, spec)()

    @staticmethod
    def _route_path(raw):
        """Strip query/fragment, percent-decode, and normalize away any '..'
        segments. The whitelist below must be matched against the SAME path
        SimpleHTTPRequestHandler will resolve, otherwise '/js/../serve.py'
        (or its percent-encoded form) passes the '/js/' prefix check and then
        serves serve.py."""
        path = raw.split('?')[0].split('#')[0]
        return posixpath.normpath(urllib.parse.unquote(path))

    def do_GET(self):
        path = self._route_path(self.path)
        spec = self.GET_EXACT.get(path)
        if spec is None:
            for prefix in sorted(self.GET_PREFIX, key=len, reverse=True):
                if path.startswith(prefix):
                    spec = self.GET_PREFIX[prefix]
                    break
        if spec is not None:
            self._run_route(spec)
            return
        # Static fallback: only the dashboard page and whitelisted asset
        # directories; everything else (including serve.py itself) is 403
        # to avoid exposing server internals.
        if path in ('/', '/index.html', f'/{self.INDEX_FILE}'):
            self.path = f'/{self.INDEX_FILE}'
            super().do_GET()
        elif any(path.startswith(p) for p in self.STATIC_PREFIXES):
            self.path = path   # serve the normalized path, not the raw one
            super().do_GET()
        else:
            log.warning("Blocked static path %s from %s", path, self.client_address[0])
            self.send_error(403)

    def do_POST(self):
        path = self._route_path(self.path)
        spec = self.POST_EXACT.get(path)
        if spec is not None:
            self._run_route(spec)
        else:
            self.send_error(404)

    # Required top-level fields for a valid GNSS payload.
    _GNSS_REQUIRED = frozenset({"lat", "lon", "fix", "sats", "hdop", "alt"})

    def _serve_gnss(self):
        try:
            data = Path(self.gnss_file).read_text()
        except FileNotFoundError:
            self._json_response(404, b'{"error":"GNSS data not available"}')
            return
        except OSError as e:
            log.warning("[gnss] file read error: %s", e)
            self._json_response(503, b'{"error":"GNSS file unreadable"}')
            return

        try:
            parsed = json.loads(data)
        except json.JSONDecodeError as e:
            log.warning("[gnss] corrupt JSON in %s: %s", self.gnss_file, e)
            self._json_response(503, b'{"error":"GNSS data corrupted"}')
            return

        # Freshness: a fix or heartbeat written within the last 10 s means the
        # receiver is connected.
        try:
            ts_str = parsed.get("ts", "")
            age = (datetime.now(timezone.utc) - datetime.fromisoformat(ts_str)).total_seconds()
            fresh = age < 10.0
        except Exception:
            fresh = False

        if "lat" in parsed:
            # Position payload — a partial one is an error (keeps strict schema).
            missing = self._GNSS_REQUIRED - parsed.keys()
            if missing:
                log.warning("[gnss] payload missing fields: %s", missing)
                self._json_response(503, b'{"error":"GNSS payload incomplete"}')
                return
            parsed["connected"] = fresh
            parsed.setdefault("has_fix", parsed.get("fix", 0) >= 1)
            parsed["gps_state"] = ("fixed"     if fresh and parsed.get("fix", 0) >= 1
                                   else "no_signal" if fresh
                                   else "disconnected")
            self._json_response(200, json.dumps(parsed).encode())
            return

        # Heartbeat payload (no position) — receiver connected but without a fix.
        parsed["connected"] = fresh
        parsed["has_fix"]   = False
        parsed["gps_state"] = "no_signal" if fresh else "disconnected"
        self._json_response(200, json.dumps(parsed).encode())

    def _serve_camera(self):
        with TELEM.rear_cam.lock:
            frame = TELEM.rear_cam.jpeg
        if frame is None:
            self._json_response(503, b'{"error":"No camera frame available"}')
            return
        self.send_response(200)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Content-Length", str(len(frame)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(frame)

    def _serve_camera_status(self):
        with TELEM.rear_cam.lock:
            connected = TELEM.rear_cam.connected
            last_t    = TELEM.rear_cam.last_frame_time
            count     = TELEM.rear_cam.frame_count
            err       = TELEM.rear_cam.last_error
            has_frame = TELEM.rear_cam.jpeg is not None
        # If no frame has arrived in the last 3 s, treat as disconnected regardless
        # of the flag (handles ROS topic going silent without an explicit error).
        if connected and (time.monotonic() - last_t) > 3.0:
            connected = False
        body = json.dumps({
            "connected":       connected,
            "has_frame":       has_frame,
            "frames_received": count,
            "last_error":      err,
        }).encode()
        self._json_response(200, body)

    def _serve_zed(self):
        with TELEM.front_zed.lock:
            frame = TELEM.front_zed.jpeg
        if frame is None:
            self._json_response(503, b'{"error":"No ZED frame available"}')
            return
        self.send_response(200)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Content-Length", str(len(frame)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(frame)

    def _serve_zed_status(self):
        with TELEM.front_zed.lock:
            connected = TELEM.front_zed.connected
            count     = TELEM.front_zed.frame_count
            err       = TELEM.front_zed.last_error
            has_frame = TELEM.front_zed.jpeg is not None
        body = json.dumps({
            "connected":       connected,
            "has_frame":       has_frame,
            "frames_received": count,
            "last_error":      err,
        }).encode()
        self._json_response(200, body)

    def _serve_detection_image(self):
        TELEM.front_det.mark_wanted()
        with TELEM.front_det.lock:
            frame = TELEM.front_det.jpeg
        if frame is None:
            self._json_response(503, b'{"error":"No detection frame available"}')
            return
        self.send_response(200)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Content-Length", str(len(frame)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(frame)

    def _serve_detection_data(self):
        TELEM.front_det.mark_wanted()
        with TELEM.front_det.lock:
            payload = TELEM.front_det.payload
        if payload is None:
            # YOLO hasn't finished its first inference yet — return empty, not an error.
            self._json_response(200, json.dumps(
                {"ts": time.time(), "count": 0, "detections": []}
            ).encode())
        else:
            self._json_response(200, json.dumps(payload).encode())

    def _stream_jpeg(self, get_frame_fn):
        """Push JPEG frames as multipart/x-mixed-replace at STREAM_FPS.

        Holds the connection open; exits when the client disconnects.
        Only sends a frame when a new one is available (identity check on bytes object).
        """
        # TCP_NODELAY: send each frame immediately without Nagle batching.
        # SO_SNDBUF=65536: cap the OS send buffer at 64 KB (~8-12 frames at our
        # reduced quality). When the client is slow the buffer fills and write()
        # blocks briefly; on resume get_frame_fn() returns the LATEST frame,
        # so stale frames are automatically dropped instead of queuing up.
        try:
            self.request.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.request.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 65536)
        except Exception:
            pass
        interval = 1.0 / STREAM_FPS
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-cache, no-store")
        self.send_header("Pragma", "no-cache")
        self.end_headers()
        last_frame = None
        try:
            while True:
                t0 = time.monotonic()
                frame = get_frame_fn()
                if frame is not None and frame is not last_frame:
                    last_frame = frame
                    header = (
                        b"--frame\r\n"
                        b"Content-Type: image/jpeg\r\n"
                        b"Content-Length: " + str(len(frame)).encode() + b"\r\n\r\n"
                    )
                    self.wfile.write(header + frame + b"\r\n")
                    self.wfile.flush()
                elapsed = time.monotonic() - t0
                sleep_t = interval - elapsed
                if sleep_t > 0:
                    time.sleep(sleep_t)
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass

    def _serve_camera_stream(self):
        def get_frame():
            with TELEM.rear_cam.lock:
                return TELEM.rear_cam.jpeg
        self._stream_jpeg(get_frame)

    def _serve_zed_stream(self):
        def get_frame():
            with TELEM.front_zed.lock:
                return TELEM.front_zed.jpeg
        self._stream_jpeg(get_frame)

    def _serve_detection_stream(self):
        TELEM.front_det.mark_wanted()
        def get_frame():
            TELEM.front_det.mark_wanted()
            # Read live camera frame (stream-res numpy) and latest YOLO boxes independently.
            # This decouples video rate (camera fps) from inference rate (YOLO fps):
            # the stream is always real-time; boxes are drawn from the last YOLO result.
            with TELEM.front_zed.lock:
                frame = TELEM.front_zed.frame
            if frame is None:
                with TELEM.front_det.lock:
                    return TELEM.front_det.jpeg   # fallback before first camera frame
            with TELEM.front_det.lock:
                boxes = list(TELEM.front_det.boxes)
            try:
                import cv2
                if boxes:
                    out = frame.copy()
                    for x1, y1, x2, y2, label in boxes:
                        _draw_box(out, x1, y1, x2, y2, label)
                else:
                    out = frame
                ok, enc = cv2.imencode('.jpg', out, [cv2.IMWRITE_JPEG_QUALITY, 60])
                return enc.tobytes() if ok else None
            except Exception:
                with TELEM.front_det.lock:
                    return TELEM.front_det.jpeg
        self._stream_jpeg(get_frame)

    def _serve_rear_detection_stream(self):
        """Rear camera live feed with YOLO boxes composited at stream rate."""
        TELEM.rear_det.mark_wanted()
        def get_frame():
            TELEM.rear_det.mark_wanted()
            with TELEM.rear_cam.lock:
                frame = TELEM.rear_cam.frame
            if frame is None:
                return None
            with TELEM.rear_det.lock:
                boxes = list(TELEM.rear_det.boxes)
            try:
                import cv2
                if boxes:
                    out = frame.copy()
                    for x1, y1, x2, y2, label in boxes:
                        _draw_box(out, x1, y1, x2, y2, label)
                else:
                    out = frame
                ok, enc = cv2.imencode('.jpg', out, [cv2.IMWRITE_JPEG_QUALITY, 60])
                return enc.tobytes() if ok else None
            except Exception:
                return None
        self._stream_jpeg(get_frame)

    def _serve_rear_detection_data(self):
        TELEM.rear_det.mark_wanted()
        with TELEM.rear_det.lock:
            payload = TELEM.rear_det.payload
        if payload is None:
            self._json_response(200, json.dumps(
                {"ts": time.time(), "count": 0, "detections": []}
            ).encode())
        else:
            self._json_response(200, json.dumps(payload).encode())

    def _serve_cmd_vel(self):
        # Parse and validate the request body first — before checking ROS state.
        # This ensures 400/413 errors are returned for bad input regardless of
        # whether ROS is available (important for security and correct test behaviour).
        try:
            length = int(self.headers.get("Content-Length", 0))
            if length > MAX_POST_BYTES:
                self._json_response(413, b'{"error":"Request too large"}')
                return
            body   = self.rfile.read(length)
            data   = json.loads(body)
            lx     = float(data.get("linear_x",  0.0))
            az     = float(data.get("angular_z", 0.0))
        except Exception as exc:
            self._json_response(400, json.dumps({"error": str(exc)}).encode())
            return

        max_lin = Handler.chassis.max_linear  if Handler.chassis else MAX_LIN_INPUT
        max_ang = Handler.chassis.max_angular if Handler.chassis else MAX_ANG_INPUT
        if abs(lx) > max_lin or abs(az) > max_ang:
            log.warning("cmd_vel rejected: lin=%.2f ang=%.2f exceeds limits from %s",
                        lx, az, self.client_address[0])
            log_event("WARN", "ROS",
                      f"Velocity command rejected: lin={lx:.2f} m/s, ang={az:.2f} rad/s "
                      f"exceeds chassis limits (max lin={max_lin}, ang={max_ang})",
                      "Reduce joystick sensitivity or increase max_linear/max_angular "
                      "in config/chassis/agrobot.yaml (or jackal.yaml). "
                      "Current hard limits are set in the Advanced Settings panel.",
                      _key="cmd-vel-reject", _debounce_s=10)
            self._json_response(400, b'{"error":"velocity out of range"}')
            return

        if Handler.speed_cmd_pub is None:
            self._json_response(503, b'{"error":"ROS publisher not ready"}')
            return

        TELEM.vel.set_command(lx, az)
        self._json_response(200, b"{}")

    def _serve_wheel_odom(self):
        l, r, last, mileage = TELEM.odom.snapshot()
        connected = last > 0 and (time.monotonic() - last) < 5.0
        data = json.dumps({"left": l, "right": r, "connected": connected, "mileage": mileage}).encode()
        self._json_response(200, data)

    def _serve_chassis_battery(self):
        """GET /api/chassis_battery — smoothed chassis pack voltage (V) and a
        freshness flag. Populated only on chassis with the `battery` feature
        (agrobot); on others it simply reports voltage 0 / connected false."""
        v, last = TELEM.battery.snapshot()
        connected = last > 0 and (time.monotonic() - last) < 30.0
        data = json.dumps({"voltage_v": v, "connected": connected}).encode()
        self._json_response(200, data)

    ui_wide = False   # set by --wide in main(); the UI letterboxes video when true

    def _serve_config(self):
        """GET /api/config — chassis name, comms, feature flags, and limits so
        the web UI can adapt (show/hide chassis-specific panels)."""
        if Handler.chassis is not None:
            cfg = Handler.chassis.to_browser_config()
        else:
            # No chassis loaded (e.g. in tests) — report module defaults.
            cfg = {
                "chassis":  "unknown",
                "comms":    "modbus_speed",
                "features": {},
                "limits":   {"maxLinear": MAX_LIN_INPUT, "maxAngular": MAX_ANG_INPUT},
            }
        cfg["ui"] = {"wide": Handler.ui_wide}
        self._json_response(200, json.dumps(cfg).encode())

    # ── PLC Gateway relay ────────────────────────────────────────────────────
    # Browsers can't speak gRPC, so these endpoints forward to the PlcClient
    # (Handler.plc), which talks gRPC to the PLC gateway. Every PLC response already
    # carries `connected`/`success`/`message`; we pass it straight through as JSON so
    # the UI can show the gateway/PLC state. None of these raise — the client maps a
    # downed gateway to {connected:false}, returned here as a normal 200 body.
    def _plc_client(self):
        """Return the PlcClient if PLC control is available on the active chassis,
        else send a 503 and return None (e.g. jackal, or PLC disabled)."""
        if Handler.plc is None or (Handler.chassis is not None and not Handler.chassis.plc_enabled):
            self._json_response(503, b'{"error":"PLC control not available on this chassis"}')
            return None
        return Handler.plc

    def _read_command(self, allowed):
        """Parse {command} from the POST body and validate it against `allowed`
        (case-insensitive). Returns the upper-cased command, or None after sending
        a 400/413. Mirrors the body-parsing guard in _serve_cmd_vel/_serve_fwd2m."""
        try:
            length = int(self.headers.get("Content-Length", 0))
            if length > MAX_POST_BYTES:
                self._json_response(413, b'{"error":"Request too large"}')
                return None
            body    = self.rfile.read(length) if length else b"{}"
            data    = json.loads(body)
            command = str(data.get("command", "")).strip().upper()
        except Exception as exc:
            self._json_response(400, json.dumps({"error": str(exc)}).encode())
            return None
        if command not in allowed:
            self._json_response(400, json.dumps(
                {"error": f"unknown command '{command}'", "allowed": sorted(allowed)}).encode())
            return None
        return command

    def _log_plc_transition(self, result, client):
        """Log PLC connect/disconnect transitions exactly once on state change."""
        connected = result.get('connected', True)
        if not TELEM.plc_link.transition(connected):
            return
        if not connected:
            log_event("ERROR", "PLC",
                      f"PLC connection lost — {client.target} unreachable",
                      "Check LAN cable between Jetson and PLC. Verify: ping 192.168.1.2  "
                      "The PLC CPU Ethernet port must be at 192.168.1.2:502 (not the FEnet card).",
                      _key="plc-conn", _debounce_s=60)
        else:
            log_event("INFO", "PLC",
                      f"PLC connection restored — {client.target}",
                      _key="plc-reconn", _debounce_s=5)

    def _serve_plc_read(self, method_name):
        """GET — forward a read RPC (get_machine_status / get_sequence_detail /
        get_auger_motor_status) and return its dict as JSON."""
        client = self._plc_client()
        if client is None:
            return
        result = getattr(client, method_name)()
        self._log_plc_transition(result, client)
        self._json_response(200, json.dumps(result).encode())

    def _serve_plc_tags(self):
        """GET /api/plc/tags — static PLC tag/register reference for the UI's PLC panel:
        the curated read/write/reserved tag map (PLC_TAG_MAP) plus the full PLC symbol
        table read from docs/plc/, each symbol annotated with its integration role. No
        gateway call — works whether or not the gateway is up; only gated on the chassis
        having a PLC (jackal → 503, via _plc_client)."""
        if self._plc_client() is None:
            return
        roles   = symbol_roles()
        symbols = []
        csv_path = DASHBOARD_DIR.parent / "docs" / "plc" / "GTS_Tree_Planter_symbols.csv"
        try:
            import csv as _csv
            with open(csv_path, newline="") as fh:
                for row in _csv.DictReader(fh):
                    name = (row.get("Name") or "").strip()
                    if not name:
                        continue
                    symbols.append({
                        "name":    name,
                        "type":    (row.get("Type") or "").strip(),
                        "address": (row.get("Address") or "").strip(),
                        "role":    roles.get(name, ""),
                    })
        except Exception as exc:
            log.warning("PLC symbol table unavailable (%s): %s", csv_path, exc)
        self._json_response(200, json.dumps({"map": PLC_TAG_MAP, "symbols": symbols}).encode())

    def _serve_plc_sequence(self, method_name):
        """POST {command: START|STOP} — forward a sequence RPC (control_auger /
        control_planter / control_both)."""
        client = self._plc_client()
        if client is None:
            return
        command = self._read_command(SEQUENCE_COMMANDS)
        if command is None:
            return
        result = getattr(client, method_name)(command)
        self._log_plc_transition(result, client)
        if result.get('connected') and not result.get('success'):
            log_event("WARN", "PLC",
                      f"PLC write failed: {result.get('message', '')}",
                      "The Modbus write landed but the PLC did not confirm. "
                      "Check Auto mode is active and the subsystem is enabled.",
                      _key="plc-write-fail", _debounce_s=5)
        self._json_response(200, json.dumps(result).encode())

    def _serve_plc_command(self, method_name):
        """POST {command} — forward a machine/robot pushbutton RPC. The allow-list
        depends on the RPC (machine vs robot arm)."""
        client = self._plc_client()
        if client is None:
            return
        allowed = MACHINE_COMMANDS if method_name == 'machine_command' else ROBOT_COMMANDS
        command = self._read_command(allowed)
        if command is None:
            return
        result = getattr(client, method_name)(command)
        self._log_plc_transition(result, client)
        self._json_response(200, json.dumps(result).encode())

    def _serve_fwd2m(self):
        """Server-side 2 m auto-drive.  Runs a tight encoder loop so speed has
        no effect on accuracy — the robot always travels exactly 2 m."""
        # Auto-forward relies on wheel encoders; only chassis that report them.
        if Handler.chassis is not None and not Handler.chassis.has_feature('fwd2m'):
            self._json_response(503, b'{"error":"auto-forward not supported on this chassis"}')
            return
        try:
            length = int(self.headers.get("Content-Length", 0))
            body   = self.rfile.read(length) if length else b"{}"
            data   = json.loads(body)
            speed     = float(data.get("speed", 0.5))
            direction = "backward" if data.get("direction") == "backward" else "forward"
            # Distance comes from the saved setting (source of truth); the body may
            # override it, but either way it is clamped to the allowed range.
            with TELEM.settings.lock:
                meters = float(TELEM.settings.data.get("driveDistance", 2.0))
            if "distance" in data:
                meters = float(data["distance"])
            meters = max(DRIVE_DISTANCE_MIN, min(DRIVE_DISTANCE_MAX, meters))
        except Exception as exc:
            self._json_response(400, json.dumps({"error": str(exc)}).encode())
            return

        result = drive_distance(direction, speed, meters=meters)
        if result.get("error") == "encoder":
            self._json_response(503, b'{"error":"Encoder not connected"}')
            return
        self._json_response(200, json.dumps(result).encode())

    def _serve_track_save(self):
        try:
            length = int(self.headers.get("Content-Length", 0))
            if length > MAX_POST_BYTES:
                self._json_response(413, b'{"error":"Request too large"}')
                return
            body   = self.rfile.read(length)
            data   = json.loads(body)
        except Exception as exc:
            self._json_response(400, json.dumps({"error": str(exc)}).encode())
            return

        points = data.get("points", [])
        if not points:
            self._json_response(400, b'{"error":"No points recorded"}')
            return

        started_at = data.get("started_at", "")
        try:
            dt = datetime.fromisoformat(started_at.replace("Z", "+00:00"))
            ts_str = dt.strftime("%Y-%m-%d_%H-%M-%S")
        except Exception:
            ts_str = datetime.now(timezone.utc).strftime("%Y-%m-%d_%H-%M-%S")

        log_dir = DASHBOARD_DIR.parent / "logs" / "recordings" / ts_str
        log_dir.mkdir(parents=True, exist_ok=True)

        filename = log_dir / "gnss.jsonl"
        try:
            with open(filename, "w") as f:
                meta = {
                    "type":        "track_meta",
                    "started_at":  data.get("started_at"),
                    "ended_at":    data.get("ended_at"),
                    "point_count": len(points),
                }
                f.write(json.dumps(meta) + "\n")
                for pt in points:
                    f.write(json.dumps(pt) + "\n")
            log.info("[track] Saved %d points → %s", len(points), filename)
            self._json_response(200, json.dumps(
                {"saved": str(filename), "points": len(points)}).encode())
        except Exception as exc:
            self._json_response(500, json.dumps({"error": str(exc)}).encode())

    def _serve_plant_log(self):
        try:
            length = int(self.headers.get("Content-Length", 0))
            if length > MAX_POST_BYTES:
                self._json_response(413, b'{"error":"Request too large"}')
                return
            body = self.rfile.read(length)
            data = json.loads(body) if length else {}
        except Exception as exc:
            self._json_response(400, json.dumps({"error": str(exc)}).encode())
            return

        required = {"lat", "lon", "ts", "count"}
        missing  = required - data.keys()
        if missing:
            self._json_response(400, json.dumps({"error": f"Missing fields: {missing}"}).encode())
            return

        lat, lon = data["lat"], data["lon"]
        chassis_name = Handler.chassis.name if Handler.chassis else "unknown"

        with TELEM.settings.lock:
            seedling_type = TELEM.settings.data.get('seedlingType', '')

        slug = '_'.join(seedling_type.lower().split()) or 'unknown'
        seedling_id = f"{slug}_{lat:.4f}_{lon:.4f}"

        seed_dir = DASHBOARD_DIR.parent / "logs" / "planted_seedlings"
        seed_dir.mkdir(parents=True, exist_ok=True)
        # One cumulative line-delimited JSON log; each seedling carries both
        # decimal degrees and a human-readable DMS geo-coordinate.
        entry = {
            "index":        data["count"],
            "ts":           data["ts"],
            "chassis":      chassis_name,
            "lat":          lat,
            "lon":          lon,
            "lat_dms":      _dms(lat, True),
            "lon_dms":      _dms(lon, False),
            "fix":          data.get("fix"),
            "fix_label":    data.get("fix_label"),
            "sats":         data.get("sats"),
            "hdop":         data.get("hdop"),
            "alt":          data.get("alt"),
            "seedling_type": seedling_type,
            "seedling_id":  seedling_id,
        }
        seed_log = seed_dir / "seedlings.jsonl"
        try:
            with open(seed_log, "a", encoding="utf-8") as f:
                f.write(json.dumps(entry, ensure_ascii=False) + "\n")
            log.info("[seedling] #%s at %s %s (%s) type=%r → %s",
                     data["count"], entry["lat_dms"], entry["lon_dms"],
                     chassis_name, seedling_type, seed_log)
            threading.Thread(target=push_seedling, args=(entry,),
                             daemon=True, name="seedling-push").start()
            self._json_response(200, json.dumps({
                "ok": True, "count": data["count"],
                "lat_dms": entry["lat_dms"], "lon_dms": entry["lon_dms"],
            }).encode())
        except Exception as exc:
            self._json_response(500, json.dumps({"error": str(exc)}).encode())

    def _serve_record_start(self):
        try:
            length = int(self.headers.get("Content-Length", 0))
            if length > MAX_POST_BYTES:
                self._json_response(413, b'{"error":"Request too large"}')
                return
            body   = self.rfile.read(length)
            data   = json.loads(body) if length else {}
        except Exception as exc:
            self._json_response(400, json.dumps({"error": str(exc)}).encode())
            return

        with TELEM.recording.lock:
            if TELEM.recording.active:
                self._json_response(200, json.dumps({"status": "already_recording"}).encode())
                return

            started_at = data.get("started_at", "")
            try:
                dt = datetime.fromisoformat(started_at.replace("Z", "+00:00"))
                ts_str = dt.strftime("%Y-%m-%d_%H-%M-%S")
            except Exception:
                ts_str = datetime.now(timezone.utc).strftime("%Y-%m-%d_%H-%M-%S")

            rec_dir = DASHBOARD_DIR.parent / "logs" / "recordings" / ts_str
            rec_dir.mkdir(parents=True, exist_ok=True)
            TELEM.recording.active = True
            TELEM.recording.dir    = rec_dir
            TELEM.recording.ts     = ts_str

        threading.Thread(target=recording_loop, args=(TELEM,),
                 daemon=True, name="cam-record").start()
        log.info("[record] Started → %s", rec_dir)
        self._json_response(200, json.dumps({"status": "started", "dir": str(rec_dir)}).encode())

    def _serve_record_stop(self):
        with TELEM.recording.lock:
            if not TELEM.recording.active:
                self._json_response(200, json.dumps({"status": "not_recording"}).encode())
                return
            TELEM.recording.active = False
            rec_dir = TELEM.recording.dir
            TELEM.recording.dir = None

        log.info("[record] Stopped → %s", rec_dir)
        self._json_response(200, json.dumps({"status": "stopped", "dir": str(rec_dir)}).encode())

    def _serve_settings_get(self):
        with TELEM.settings.lock:
            body = json.dumps(TELEM.settings.data).encode()
        self._json_response(200, body)

    def _serve_settings_post(self):
        try:
            length = int(self.headers.get("Content-Length", 0))
            if length > MAX_POST_BYTES:
                self._json_response(413, b'{"error":"Request too large"}')
                return
            body = self.rfile.read(length)
            data = json.loads(body)
        except Exception as exc:
            self._json_response(400, json.dumps({"error": str(exc)}).encode())
            return

        updates = {}
        for key, lo, hi in [('maxLinear', 0.05, 15.0), ('maxAngular', 0.05, 15.0),
                             ('modbusSpeed', 0, 32767),
                             ('driveDistance', DRIVE_DISTANCE_MIN, DRIVE_DISTANCE_MAX)]:
            if key in data:
                try:
                    v = float(data[key])
                except (ValueError, TypeError):
                    self._json_response(400, json.dumps({"error": f"{key} must be a number"}).encode())
                    return
                if not (lo <= v <= hi):
                    self._json_response(400, json.dumps(
                        {"error": f"{key} out of range ({lo}–{hi})"}).encode())
                    return
                updates[key] = int(v) if key == 'modbusSpeed' else round(v, 3)

        if 'seedlingType' in data:
            st = str(data['seedlingType']).strip()[:100]
            updates['seedlingType'] = st

        with TELEM.settings.lock:
            TELEM.settings.data.update(updates)
            body = json.dumps(TELEM.settings.data).encode()

        log.info("[settings] Updated: %s", updates)
        self._json_response(200, body)

    def _json_response(self, code: int, body: bytes):
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _serve_events(self):
        """GET /api/events?since=<unix_timestamp> — return all log entries newer
        than `since` (default 0 = all). The browser polls this every 3 s and
        passes the timestamp of its newest entry to get only fresh events."""
        try:
            from urllib.parse import urlparse, parse_qs
            qs    = parse_qs(urlparse(self.path).query)
            since = float((qs.get('since', ['0'])[0]) or 0)
        except Exception:
            since = 0.0
        self._json_response(200, json.dumps(events_since(since)).encode())

    def log_message(self, fmt, *args):
        if args and str(args[1]) in ("200", "304"):
            return
        if args and str(args[1]) in ("200", "503") and "/api/camera" in str(args[0]):
            return
        if args and str(args[1]) in ("200", "503", "404") and "/api/detection" in str(args[0]):
            return
        if args and str(args[1]) in ("200", "503") and "/api/zed" in str(args[0]):
            return
        if args and str(args[1]) == "200" and "/api/events" in str(args[0]):
            return
        if args and str(args[1]) == "404":
            path = str(args[0])
            if any(x in path for x in ("/api/gnss", "favicon", ".svg", ".ico", ".png")):
                return
        super().log_message(fmt, *args)


class _Server(socketserver.ThreadingMixIn, socketserver.TCPServer):
    allow_reuse_address = True
    daemon_threads      = True

    def handle_error(self, request, client_address):
        # A client that closes the connection mid-response — navigating away,
        # aborting an MJPEG stream, or closing a tab while a static file or
        # camera frame is still being written — surfaces here as a broken pipe
        # / connection reset. That is normal client behaviour, not a server
        # fault, so swallow it quietly. Genuine errors fall through to the
        # default handler so they stay visible.
        exc = sys.exc_info()[1]
        if isinstance(exc, (BrokenPipeError, ConnectionResetError,
                            ConnectionAbortedError)):
            return
        super().handle_error(request, client_address)




def main():
    ap = argparse.ArgumentParser(description="Dual-robot dashboard server (agrobot | jackal)")
    ap.add_argument("--port", type=int, default=8766)
    ap.add_argument("--gnss", default=GNSS_FILE_DEFAULT)
    ap.add_argument("--chassis", default=None,
                    help="agrobot | jackal — overrides config/active_chassis.yaml")
    ap.add_argument("--rear-camera", dest="rear_camera", default=None,
                    help="zed | webcam | none — overrides the chassis rear_camera setting")
    ap.add_argument("--wide", action="store_true",
                    help="front ZED at HD2K — full ~110° FOV at 15 fps; the UI "
                         "letterboxes video instead of cropping")
    args = ap.parse_args()

    Handler.gnss_file = args.gnss
    if args.wide:
        cameras.ZED_FRONT_RESOLUTION = "HD2K"
        cameras.ZED_FRONT_FPS        = 15    # HD2K hardware cap
        Handler.ui_wide      = True
        log.info("[zed] wide mode — front ZED at HD2K / 15 fps, UI letterboxing on")

    # Resolve the active chassis: --chassis > $ROBOT_CHASSIS > active_chassis.yaml.
    CHASSIS = chassis.load_active(args.chassis)
    Handler.chassis = CHASSIS
    # Resolve the rear-camera source: --rear-camera > $REAR_CAMERA > chassis yaml.
    rear_src = chassis.resolve_rear_camera(args.rear_camera, CHASSIS)
    CHASSIS.rear_camera = rear_src   # so GET /api/config reflects the effective source
    log.info("[chassis] active: %s — %s (comms=%s, rear_camera=%s)",
             CHASSIS.name, CHASSIS.description, CHASSIS.comms, rear_src)

    # PLC client — sends auger/planter/robot-arm commands DIRECTLY over Modbus TCP to the
    # PLC at plc_host:plc_port (502). Built only for chassis with plc.enabled (agrobot); the
    # connection is lazy, so this never blocks startup if the PLC is unreachable.
    if CHASSIS.plc_enabled:
        try:
            from plc_client import PlcClient
        except ImportError:
            from dashboard.plc_client import PlcClient
        Handler.plc = PlcClient(CHASSIS.plc_host, CHASSIS.plc_port)
        log.info("[plc] Modbus TCP client → %s (connects on first request)", Handler.plc.target)
    else:
        log.info("[plc] disabled for chassis %s", CHASSIS.name)

    # Set true once the ROS camera subscription is created and delivering the rear
    # feed; the local V4L2 capture is then skipped to avoid encoding the same camera
    # twice (agrobot) or uselessly re-opening a non-existent local device (jackal).
    _ros_camera_active = False

    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        from sensor_msgs.msg import Image as RosImage

        rclpy.init()
        _node = Node('dashboard_server')

        # Velocity publisher (+ optional wheel-odom feedback) for the active chassis.
        # Sets Handler.speed_cmd_pub and returns the publish callable for the timer.
        publish_velocity = CHASSIS.setup_ros(_node, Handler)
        log.info("[speed_cmd] ROS publisher ready → %s (comms=%s)",
                 CHASSIS.speed_cmd_topic, CHASSIS.comms)
        if CHASSIS.wheel_odom_topic:
            log.info("[wheel_odom] Subscribed to %s", CHASSIS.wheel_odom_topic)
        if CHASSIS.has_feature("battery") and CHASSIS.battery_topic:
            log.info("[chassis_battery] Subscribed to %s", CHASSIS.battery_topic)

        # The 20 Hz velocity command runs on a DEDICATED thread, not a ROS timer in
        # the executor. The same executor also runs the camera-image callback (JPEG
        # encode); keeping control on its own thread means a busy executor can never
        # delay or jitter a teleop command — the root cause of the key→robot lag and
        # deadman stalls under camera/AI load. publish_velocity() only builds and
        # publishes a message, which is thread-safe in rclpy. Deadman semantics are
        # unchanged: silence past VEL_TIMEOUT publishes a single stop, then idles.
        def _vel_loop():
            period = 0.05   # 20 Hz
            while True:
                t0 = time.monotonic()
                try:
                    cmd = TELEM.vel.consume_for_publish()
                    if cmd is not None:
                        publish_velocity(cmd[0], cmd[1])
                except Exception as exc:
                    log.error("[speed_cmd] publish error: %s", exc)
                    log_event("ERROR", "ROS",
                              f"Velocity command publish failed: {exc}",
                              "Check ROS domain ID and DDS configuration. "
                              "Verify: ros2 topic list | grep cmd_vel  "
                              "Restart the dashboard if the issue persists.",
                              _key="vel-publish", _debounce_s=10)
                dt = time.monotonic() - t0
                if dt < period:
                    time.sleep(period - dt)

        threading.Thread(target=_vel_loop, daemon=True, name="vel-control").start()

        # Camera subscriber — topic depends on the active chassis.
        _cam_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        _encode_fn = _build_encode_fn()
        # Skip the ROS camera topic when a local webcam is the rear source (the two
        # feeds would fight over the same buffer) or when the rear view is disabled.
        if _encode_fn and CHASSIS.camera_topic and rear_src not in ("webcam", "none", "zed"):
            _cam_enc_interval = 1.0 / max(STREAM_FPS, 1.0)
            _cam_last_enc     = [0.0]   # mutable holder for the closure
            def _on_image(msg: RosImage):
                # Throttle to the display rate: the publisher may emit 30 fps, but the
                # browser only needs STREAM_FPS and every JPEG encode costs CPU.
                now = time.monotonic()
                if now - _cam_last_enc[0] < _cam_enc_interval:
                    return
                _cam_last_enc[0] = now
                try:
                    frame = _encode_fn(msg)
                    if frame:
                        with TELEM.rear_cam.lock:
                            TELEM.rear_cam.jpeg            = frame
                            TELEM.rear_cam.frame_count     += 1
                            TELEM.rear_cam.connected       = True
                            TELEM.rear_cam.last_error      = None
                            TELEM.rear_cam.last_frame_time = time.monotonic()
                            n = TELEM.rear_cam.frame_count
                        if n == 1 or n % 300 == 0:
                            log.debug("[camera] %d frames received (%dx%d)",
                                      n, msg.width, msg.height)
                except Exception as exc:
                    err = f"{type(exc).__name__}: {exc}"
                    with TELEM.rear_cam.lock:
                        TELEM.rear_cam.last_error = err
                    log.error("[camera] encode error: %s", err)
                    log_event("WARN", "Camera",
                              f"Camera frame encode error: {err}",
                              "Usually transient. If persistent, check camera USB connection "
                              "and restart the dashboard.",
                              _key="cam-encode", _debounce_s=30)

            _node.create_subscription(RosImage, CHASSIS.camera_topic,
                                      _on_image, _cam_qos)
            _ros_camera_active = True
            log.info("[camera] Subscribed to %s", CHASSIS.camera_topic)
        elif not _encode_fn:
            log.warning("[camera] neither cv2 nor PIL found — /api/camera will return 503")
        elif rear_src == "webcam":
            log.info("[camera] rear_camera=webcam — local capture only, ROS camera topic skipped")
        elif rear_src == "none":
            log.info("[camera] rear_camera=none — rear view disabled, ROS camera topic skipped")
        else:
            log.info("[camera] no ROS camera_topic for this chassis — using local capture only")

        _executor = MultiThreadedExecutor(num_threads=4)
        _executor.add_node(_node)

        def _ros_spin():
            try:
                _executor.spin()
            except Exception:
                pass  # ExternalShutdownException on clean exit — not an error

        threading.Thread(target=_ros_spin, daemon=True).start()

    except Exception as exc:
        log.warning("[rclpy] ROS init failed — teleop will not work. Reason: %s", exc)
        log_event("ERROR", "ROS",
                  f"ROS 2 initialization failed — robot cannot be driven: {exc}",
                  "Run: source /opt/ros/humble/setup.bash && source install/setup.bash  "
                  "then restart the dashboard. If the error persists, check the ROS "
                  "installation: ros2 doctor",
                  _key="ros-init", _debounce_s=3600)

    if rear_src == "none":
        log.info("[rear-cam] rear_camera=none — rear view disabled, no rear capture")
    elif _ros_camera_active:
        # The rear feed already arrives over ROS — don't also capture/encode it via
        # V4L2 (that doubled the work on agrobot and error-looped on jackal).
        log.info("[rear-cam] rear feed served via ROS camera topic — skipping redundant V4L2 capture")
    else:
        cameras.start_rear_camera_thread(TELEM, rear_src, CHASSIS.rear_camera_device)
    cameras.start_zed_thread(TELEM)

    with _Server(("", args.port), Handler) as httpd:
        local_ip = _local_ip()
        log.info("Dashboard ready")
        log.info("  Chassis  → %s (%s)", CHASSIS.name, CHASSIS.comms)
        log.info("  Local    → http://localhost:%d", args.port)
        log.info("  Network  → http://%s:%d", local_ip, args.port)
        log.info("  GNSS     → %s", args.gnss)
        log.info("  Vel max  → lin=%.1f m/s  ang=%.1f rad/s",
                 CHASSIS.max_linear, CHASSIS.max_angular)
        log.info("Press Ctrl-C to stop.")
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            log.info("Stopped.")


def _build_encode_fn():
    """Return a function(RosImage) -> bytes that JPEG-encodes a ROS image frame."""
    try:
        import cv2
        import numpy as np

        def encode_cv2(msg):
            raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)
            channels = len(raw) // (msg.height * msg.width) if msg.width and msg.height else 3
            channels = max(1, min(channels, 4))
            arr = raw.reshape(msg.height, msg.step)[:, :msg.width * channels]
            arr = arr.reshape(msg.height, msg.width, channels)
            enc_lower = msg.encoding.lower()
            if enc_lower in ('rgb8', 'rgb'):
                arr = arr[:, :, ::-1]
            ok, enc = cv2.imencode('.jpg', arr, [cv2.IMWRITE_JPEG_QUALITY, 75])
            return enc.tobytes() if ok else None

        return encode_cv2
    except ImportError:
        pass

    try:
        from PIL import Image as PilImage
        import io

        def encode_pil(msg):
            img = PilImage.frombytes('RGB', (msg.width, msg.height), bytes(msg.data))
            buf = io.BytesIO()
            img.save(buf, format='JPEG', quality=75)
            return buf.getvalue()

        return encode_pil
    except ImportError:
        pass

    return None


def _local_ip() -> str:
    import socket
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("8.8.8.8", 80))
            return s.getsockname()[0]
    except Exception:
        return "127.0.0.1"


if __name__ == "__main__":
    main()
