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
import collections
import contextlib
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

# ── Browser-visible event log ──────────────────────────────────────────────
# Thread-safe ring buffer served via GET /api/events?since=<unix_ts>.
# Entries: {ts, level, source, message, suggestion}
_event_log      = collections.deque(maxlen=500)
_event_log_lock = threading.Lock()
# Debounce: suppress identical (source, key) events within a time window so
# a permanently-down PLC or disconnected camera doesn't flood the log.
_event_debounce: dict = {}   # (source, key) → monotonic time of last emission


def log_event(level: str, source: str, message: str,
              suggestion: str = "", _key: str = "", _debounce_s: float = 0.0):
    """Append a structured event to the browser-visible ring buffer.

    level      : 'INFO' | 'WARN' | 'ERROR'
    source     : subsystem label ('PLC', 'Camera', 'GNSS', 'ROS', 'System', 'Network')
    message    : human-readable description
    suggestion : actionable fix hint shown in the log panel
    _key       : debounce key; if set with _debounce_s > 0, identical events are
                 suppressed within that many seconds
    """
    if _key and _debounce_s > 0:
        now = time.monotonic()
        dk  = (source, _key)
        # log_event is called from capture threads, ROS callbacks and HTTP
        # request threads concurrently — the check-then-set must be atomic.
        with _event_log_lock:
            if now - _event_debounce.get(dk, 0.0) < _debounce_s:
                return
            _event_debounce[dk] = now
    entry = {
        "ts":         time.time(),
        "level":      level.upper(),
        "source":     source,
        "message":    message,
        "suggestion": suggestion,
    }
    with _event_log_lock:
        _event_log.append(entry)
    lvl = level.upper()
    if lvl == "ERROR":
        log.error("[%s] %s", source, message)
    elif lvl == "WARN":
        log.warning("[%s] %s", source, message)
    else:
        log.info("[%s] %s", source, message)


@contextlib.contextmanager
def _quiet_stderr():
    """Redirect C-level stderr to /dev/null to suppress OpenCV V4L2 WARN spam."""
    null_fd = os.open(os.devnull, os.O_WRONLY)
    saved   = os.dup(2)
    os.dup2(null_fd, 2)
    try:
        yield
    finally:
        os.dup2(saved, 2)
        os.close(null_fd)
        os.close(saved)

DASHBOARD_DIR        = Path(__file__).parent

# Supabase ingest endpoint for planted-seedling records.  The write key MUST
# come from the environment — a previous revision committed it to source, so
# that key is considered leaked and needs rotation on the Supabase side.
_SUPABASE_INGEST_URL = os.environ.get(
    "AGROBOT_SUPABASE_URL",
    "https://ingest.invalid/functions/v1/ingest")
_SUPABASE_AGROBOT_KEY   = os.environ.get("AGROBOT_SUPABASE_KEY", "")
GNSS_FILE_DEFAULT    = "/tmp/gnss_coords.json"
DETECTIONS_FILE      = "/tmp/object_detections.json"
ZED_DEVICE           = "/dev/zed2i"    # symlink used by the OpenCV grayscale fallback only
ZED_FRONT_INDEX      = 0              # pyzed camera index — front ZED 2i
ZED_REAR_INDEX       = 1              # pyzed camera index — rear ZED 2i
# Front-ZED capture mode. --wide switches to HD2K: the full sensor (~110° FOV,
# 2208×1242) at its 15 fps hardware cap; the UI letterboxes instead of cropping.
ZED_FRONT_RESOLUTION = "HD720"
ZED_FRONT_FPS        = 30
WEBCAM_DEVICE_DEFAULT = "/dev/video0"  # generic USB UVC webcam (e.g. Logitech)
RS_DISPLAY_FPS       = 20.0   # live camera capture / display (20 fps: smoother + far lighter than 30 on the Jetson)
RECORD_FPS           = 15.0   # rear.mp4 + front.mp4 saved at 15 fps (lighter on the Jetson)
STREAM_FPS           = 20.0   # live MJPEG stream to the browser at 20 fps

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


def _push_seedling(entry: dict):
    """POST a planted-seedling record to the Supabase ingest endpoint.
    Runs in a daemon thread so it never blocks the HTTP response."""
    if not _SUPABASE_AGROBOT_KEY:
        log_event("WARN", "GNSS",
                  "Seedling cloud upload skipped — AGROBOT_SUPABASE_KEY is not set",
                  "Export AGROBOT_SUPABASE_KEY before launching the dashboard. "
                  "The record is saved locally in logs/planted_seedlings/seedlings.jsonl.",
                  _key="seedling-push", _debounce_s=3600)
        return
    payload = json.dumps({"source": "robot", "records": [entry]}).encode()
    req = urllib.request.Request(
        _SUPABASE_INGEST_URL,
        data=payload,
        headers={
            "Content-Type": "application/json",
            "x-agrobot-key":   _SUPABASE_AGROBOT_KEY,
        },
        method="POST",
    )
    try:
        with urllib.request.urlopen(req, timeout=10) as resp:
            body = resp.read().decode()
            log.info("[seedling] Supabase ingest OK: %s", body)
    except urllib.error.HTTPError as exc:
        log.warning("[seedling] Supabase ingest HTTP %s: %s", exc.code, exc.read().decode())
        log_event("WARN", "GNSS",
                  f"Seedling cloud upload failed (HTTP {exc.code})",
                  "Check internet connection and the SUPABASE_KEY value in serve.py. "
                  "The record is saved locally in logs/planted_seedlings/seedlings.jsonl.",
                  _key="seedling-push", _debounce_s=60)
    except Exception as exc:
        log.warning("[seedling] Supabase ingest failed: %s", exc)
        log_event("WARN", "GNSS",
                  f"Seedling cloud upload error: {exc}",
                  "Check internet connection. "
                  "The record is saved locally in logs/planted_seedlings/seedlings.jsonl.",
                  _key="seedling-push", _debounce_s=60)


# ---------------------------------------------------------------------------
# HTTP handler
# ---------------------------------------------------------------------------
class Handler(SimpleHTTPRequestHandler):
    gnss_file: str   = GNSS_FILE_DEFAULT
    speed_cmd_pub    = None   # rclpy publisher, set by main() after rclpy.init()
    chassis          = None   # active chassis.Chassis, set by main(); None in tests
    plc              = None   # plc_client.PlcClient, set by main() on plc-enabled chassis

    _cam_lock            = threading.Lock()
    _cam_jpeg: bytes     = None
    _cam_frame           = None   # latest stream-res BGR numpy frame for rear detection compositing
    _cam_frame_count     = 0
    _cam_last_error      = None
    _cam_connected       = False
    _cam_last_frame_time = 0.0   # kept for MJPEG stream throttling

    _det_lock         = threading.Lock()
    _det_jpeg: bytes  = None   # kept as fallback when _zed_frame is not yet available
    _det_boxes        = []     # [(x1,y1,x2,y2,label), ...] at stream-res; under _det_lock
    _det_payload      = None   # last detection JSON dict; under _det_lock; served by /api/detection/data
    # On-demand detection: YOLO inference runs ONLY while a client is actively
    # viewing detections. Each detection request stamps this monotonic time; the
    # capture loop checks _detection_wanted() and skips inference entirely when the
    # detection view is closed — so person-detection costs zero CPU/GPU when off.
    _det_last_request = 0.0
    DET_IDLE_TIMEOUT  = 3.0   # stop inferring this long after the last detection request

    # Rear camera detection — same pattern as front; under _rear_det_lock.
    _rear_det_lock    = threading.Lock()
    _rear_det_boxes   = []
    _rear_det_payload = None
    _rear_det_last_request = 0.0

    _zed_lock            = threading.Lock()
    _zed_jpeg: bytes     = None
    _zed_frame           = None  # latest stream-res BGR numpy frame for detection compositing
    _zed_frame_count     = 0
    _zed_connected       = False
    _zed_last_error      = None
    _zed_last_frame_time = 0.0   # monotonic; 0 = no frame ever received

    _rec_lock         = threading.Lock()
    _rec_active: bool = False
    _rec_dir          = None
    _rec_ts           = None

    # Velocity state — written by HTTP handler threads, read by the ROS timer thread.
    _vel_lock   = threading.Lock()
    _vel_lin    = 0.0
    _vel_ang    = 0.0
    _vel_active = False
    _vel_last   = 0.0
    VEL_TIMEOUT = 0.5     # stop if browser goes silent for this long (s)

    # Wheel odometry — written by ROS subscriber, read by HTTP handler threads.
    _odom_lock     = threading.Lock()
    _odom_l        = 0
    _odom_r        = 0
    _odom_last     = 0.0    # monotonic time of last received message; 0 = never
    _odom_mileage  = 0.0    # accumulated |center displacement| in pulses
    _odom_prev_l   = None   # previous encoder values for delta computation
    _odom_prev_r   = None

    # Chassis battery voltage — written by the ROS subscriber (chassis.setup_ros,
    # agrobot only), read by HTTP handler threads. Raw readings are median-smoothed
    # over a 15 s window in the subscriber; the handler just reports the result.
    _chassis_batt_lock        = threading.Lock()
    _chassis_batt_window      = []   # [(monotonic_time, voltage_v), ...]
    _chassis_batt_last        = 0.0  # monotonic time of last received message; 0 = never
    _chassis_batt_smoothed    = 0.0  # median of the last valid window
    _chassis_batt_smoothed_at = 0.0  # when the smoothed value was last computed

    # PLC connection state — track transitions so we only log connect/disconnect
    # events when the state actually changes, not on every polled request.
    _plc_last_connected = None   # None=unknown, True=up, False=down
    _plc_state_lock     = threading.Lock()

    _settings_lock = threading.Lock()
    _settings: dict = {
        'maxLinear':   2.0,    # absolute max forward speed m/s (= Fast preset)
        'maxAngular':  0.5,    # max turn rate rad/s
        'modbusSpeed': 1500,   # raw Modbus units for Normal preset (mirrors slLinear/4 * LINEAR_SCALE)
        'seedlingType': '',    # species label appended to every planted-seedling record
    }

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
        with Handler._cam_lock:
            frame = Handler._cam_jpeg
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
        with Handler._cam_lock:
            connected = Handler._cam_connected
            last_t    = Handler._cam_last_frame_time
            count     = Handler._cam_frame_count
            err       = Handler._cam_last_error
            has_frame = Handler._cam_jpeg is not None
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
        with Handler._zed_lock:
            frame = Handler._zed_jpeg
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
        with Handler._zed_lock:
            connected = Handler._zed_connected
            count     = Handler._zed_frame_count
            err       = Handler._zed_last_error
            has_frame = Handler._zed_jpeg is not None
        body = json.dumps({
            "connected":       connected,
            "has_frame":       has_frame,
            "frames_received": count,
            "last_error":      err,
        }).encode()
        self._json_response(200, body)

    @classmethod
    def _set_zed_status(cls, connected: bool, error=None):
        """Update the front-ZED connectivity flags under _zed_lock.

        The capture threads used to write these flags directly while
        _serve_zed_status read them under the lock — every status write must
        go through here so reads and writes are actually synchronized."""
        with cls._zed_lock:
            cls._zed_connected  = connected
            cls._zed_last_error = error

    @classmethod
    def _mark_detection_wanted(cls):
        """A client is viewing front detections — keep front YOLO inference running."""
        cls._det_last_request = time.monotonic()

    @classmethod
    def _detection_wanted(cls) -> bool:
        """True while a client has requested front detections recently."""
        return (time.monotonic() - cls._det_last_request) < cls.DET_IDLE_TIMEOUT

    @classmethod
    def _mark_rear_detection_wanted(cls):
        """A client is viewing rear detections — keep rear YOLO inference running."""
        cls._rear_det_last_request = time.monotonic()

    @classmethod
    def _rear_detection_wanted(cls) -> bool:
        """True while a client has requested rear detections recently."""
        return (time.monotonic() - cls._rear_det_last_request) < cls.DET_IDLE_TIMEOUT

    def _serve_detection_image(self):
        Handler._mark_detection_wanted()
        with Handler._det_lock:
            frame = Handler._det_jpeg
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
        Handler._mark_detection_wanted()
        with Handler._det_lock:
            payload = Handler._det_payload
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
            with Handler._cam_lock:
                return Handler._cam_jpeg
        self._stream_jpeg(get_frame)

    def _serve_zed_stream(self):
        def get_frame():
            with Handler._zed_lock:
                return Handler._zed_jpeg
        self._stream_jpeg(get_frame)

    def _serve_detection_stream(self):
        Handler._mark_detection_wanted()
        def get_frame():
            Handler._mark_detection_wanted()
            # Read live camera frame (stream-res numpy) and latest YOLO boxes independently.
            # This decouples video rate (camera fps) from inference rate (YOLO fps):
            # the stream is always real-time; boxes are drawn from the last YOLO result.
            with Handler._zed_lock:
                frame = Handler._zed_frame
            if frame is None:
                with Handler._det_lock:
                    return Handler._det_jpeg   # fallback before first camera frame
            with Handler._det_lock:
                boxes = list(Handler._det_boxes)
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
                with Handler._det_lock:
                    return Handler._det_jpeg
        self._stream_jpeg(get_frame)

    def _serve_rear_detection_stream(self):
        """Rear camera live feed with YOLO boxes composited at stream rate."""
        Handler._mark_rear_detection_wanted()
        def get_frame():
            Handler._mark_rear_detection_wanted()
            with Handler._cam_lock:
                frame = Handler._cam_frame
            if frame is None:
                return None
            with Handler._rear_det_lock:
                boxes = list(Handler._rear_det_boxes)
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
        Handler._mark_rear_detection_wanted()
        with Handler._rear_det_lock:
            payload = Handler._rear_det_payload
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

        with Handler._vel_lock:
            Handler._vel_lin    = lx
            Handler._vel_ang    = az
            Handler._vel_active = True
            Handler._vel_last   = time.monotonic()
        self._json_response(200, b"{}")

    def _serve_wheel_odom(self):
        with Handler._odom_lock:
            l, r, last = Handler._odom_l, Handler._odom_r, Handler._odom_last
            mileage = Handler._odom_mileage
        connected = last > 0 and (time.monotonic() - last) < 5.0
        data = json.dumps({"left": l, "right": r, "connected": connected, "mileage": mileage}).encode()
        self._json_response(200, data)

    def _serve_chassis_battery(self):
        """GET /api/chassis_battery — smoothed chassis pack voltage (V) and a
        freshness flag. Populated only on chassis with the `battery` feature
        (agrobot); on others it simply reports voltage 0 / connected false."""
        with Handler._chassis_batt_lock:
            v    = Handler._chassis_batt_smoothed
            last = Handler._chassis_batt_last
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
        # Concurrent PLC requests race on this transition flag — without the
        # lock a flap can be logged twice or a transition missed entirely.
        with Handler._plc_state_lock:
            if Handler._plc_last_connected == connected:
                return
            Handler._plc_last_connected = connected
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
            speed  = float(data.get("speed", 0.5))
            speed  = max(0.05, min(MAX_LIN_INPUT, abs(speed)))
        except Exception as exc:
            self._json_response(400, json.dumps({"error": str(exc)}).encode())
            return

        # Encoder calibration comes from the active chassis config; the module
        # constant is only the fallback when no chassis is loaded (tests).
        ppm = Handler.chassis.pulse_per_m if Handler.chassis is not None else PULSE_PER_M
        target_pulses = 2.0 * ppm   # the full 2 m drive
        slow_pulses   = 0.5 * ppm   # final 0.5 m: crawl to kill coasting

        with Handler._odom_lock:
            start_l   = Handler._odom_l
            start_r   = Handler._odom_r
            last_odom = Handler._odom_last

        if last_odom == 0 or (time.monotonic() - last_odom) > 3.0:
            self._json_response(503, b'{"error":"Encoder not connected"}')
            return

        cmd_speed = speed   # what we last wrote to vel_lin

        with Handler._vel_lock:
            Handler._vel_lin    = cmd_speed
            Handler._vel_ang    = 0.0
            Handler._vel_active = True
            Handler._vel_last   = time.monotonic()

        deadline = time.monotonic() + 30.0
        traveled = 0.0
        aborted  = False

        try:
            while time.monotonic() < deadline:
                time.sleep(0.02)                   # 50 Hz encoder check

                with Handler._odom_lock:
                    cur_l = Handler._odom_l
                    cur_r = Handler._odom_r
                traveled = ((cur_l - start_l) + (cur_r - start_r)) / 2.0

                next_speed = auto_drive.plan_speed(
                    traveled, target_pulses, slow_pulses, speed, FWD_SLOW_SPEED)
                if next_speed is None:   # target distance reached
                    break

                with Handler._vel_lock:
                    # Abort if an external command overrode our last commanded speed
                    if abs(Handler._vel_lin - cmd_speed) > 0.01:
                        aborted = True
                        break
                    Handler._vel_lin  = next_speed
                    Handler._vel_last = time.monotonic()   # keep deadman alive
                cmd_speed = next_speed
        finally:
            with Handler._vel_lock:
                Handler._vel_lin    = 0.0
                Handler._vel_ang    = 0.0
                Handler._vel_active = True
                Handler._vel_last   = time.monotonic()

        done   = (not aborted) and (traveled >= target_pulses)
        result = {"done": done, "aborted": aborted,
                  "traveled_m": round(traveled / ppm, 3)}
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

        with Handler._settings_lock:
            seedling_type = Handler._settings.get('seedlingType', '')

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
            threading.Thread(target=_push_seedling, args=(entry,),
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

        with Handler._rec_lock:
            if Handler._rec_active:
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
            Handler._rec_active = True
            Handler._rec_dir    = rec_dir
            Handler._rec_ts     = ts_str

        threading.Thread(target=_recording_loop, daemon=True, name="cam-record").start()
        log.info("[record] Started → %s", rec_dir)
        self._json_response(200, json.dumps({"status": "started", "dir": str(rec_dir)}).encode())

    def _serve_record_stop(self):
        with Handler._rec_lock:
            if not Handler._rec_active:
                self._json_response(200, json.dumps({"status": "not_recording"}).encode())
                return
            Handler._rec_active = False
            rec_dir = Handler._rec_dir
            Handler._rec_dir = None

        log.info("[record] Stopped → %s", rec_dir)
        self._json_response(200, json.dumps({"status": "stopped", "dir": str(rec_dir)}).encode())

    def _serve_settings_get(self):
        with Handler._settings_lock:
            body = json.dumps(Handler._settings).encode()
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
                             ('modbusSpeed', 0, 32767)]:
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

        with Handler._settings_lock:
            Handler._settings.update(updates)
            body = json.dumps(Handler._settings).encode()

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
        with _event_log_lock:
            events = [e for e in _event_log if e['ts'] > since]
        self._json_response(200, json.dumps(events).encode())

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


def _recording_loop():
    """Write rear and front cameras to MP4 at RECORD_FPS."""
    try:
        import cv2
        import numpy as np
    except ImportError:
        log.error("[record] cv2 not available — cannot record video")
        log_event("ERROR", "System",
                  "Video recording failed — OpenCV (cv2) is not installed",
                  "Install OpenCV: pip3 install opencv-python",
                  _key="record-cv2", _debounce_s=3600)
        with Handler._rec_lock:
            Handler._rec_active = False
        return

    interval     = 1.0 / max(RECORD_FPS, 0.1)
    fourcc       = cv2.VideoWriter_fourcc(*'mp4v')
    rear_writer  = None
    front_writer = None
    rear_frames  = 0
    front_frames = 0

    with Handler._rec_lock:
        rec_dir = Handler._rec_dir
        rec_ts  = Handler._rec_ts

    if rec_dir is None:
        log.error("[record] rec_dir is None at loop start — aborting")
        with Handler._rec_lock:
            Handler._rec_active = False
        return

    rear_path  = rec_dir / "rear.mp4"
    front_path = rec_dir / "front.mp4"

    while True:
        t0 = time.monotonic()
        with Handler._rec_lock:
            if not Handler._rec_active:
                break
        with Handler._cam_lock:
            rear_jpeg = Handler._cam_jpeg
        with Handler._zed_lock:
            front_jpeg = Handler._zed_jpeg

        for jpeg, which in ((rear_jpeg, 'rear'), (front_jpeg, 'front')):
            if jpeg is None:
                continue
            arr = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
            if arr is None:
                continue
            h, w = arr.shape[:2]
            if which == 'rear':
                if rear_writer is None:
                    rear_writer = cv2.VideoWriter(str(rear_path), fourcc, RECORD_FPS, (w, h))
                rear_writer.write(arr)
                rear_frames += 1
            else:
                if front_writer is None:
                    front_writer = cv2.VideoWriter(str(front_path), fourcc, RECORD_FPS, (w, h))
                front_writer.write(arr)
                front_frames += 1

        elapsed = time.monotonic() - t0
        sleep_t = interval - elapsed
        if sleep_t > 0:
            time.sleep(sleep_t)

    if rear_writer:
        rear_writer.release()
    if front_writer:
        front_writer.release()
    log.info("[record] Saved — rear=%d frames, front=%d frames", rear_frames, front_frames)


def _find_rs_device() -> str:
    """Return the first /dev/videoN that is a RealSense color (YUYV) stream."""
    import os, subprocess
    for i in range(10):
        dev = f"/dev/video{i}"
        if not os.path.exists(dev):
            continue
        try:
            info = subprocess.run(
                ['v4l2-ctl', f'--device={dev}', '--info', '--list-formats'],
                capture_output=True, text=True, timeout=2,
            ).stdout
            if 'RealSense' in info and 'YUYV' in info:
                return dev
        except Exception:
            pass
    return RS_DEVICE_DEFAULT


def _is_capture_webcam(dev):
    """True if `dev` is a generic USB UVC capture camera (not a RealSense or ZED).
    The ZED and RealSense are driven elsewhere (ZED SDK / RealSense node), so the
    rear webcam must never grab them — doing so yields a grayscale, split stereo
    frame and blocks the ZED SDK's color path."""
    import subprocess
    try:
        info = subprocess.run(
            ['v4l2-ctl', f'--device={dev}', '--info', '--list-formats'],
            capture_output=True, text=True, timeout=2,
        ).stdout
    except Exception:
        return False
    if 'RealSense' in info or 'ZED' in info:
        return False
    return ('Video Capture' in info or 'MJPG' in info or 'YUYV' in info)


def _find_webcam_device(preferred=None):
    """Resolve the rear USB webcam to a STABLE device path so it can never be
    confused with the ZED/RealSense and survives unplug/replug or node renumbering.

    Priority: explicit `preferred` > a /dev/v4l/by-id/* symlink (keyed to the
    camera's USB serial — does not change when devices are re-enumerated) > a bare
    /dev/videoN scan. RealSense and ZED nodes are always skipped."""
    if preferred is not None:
        return preferred
    import os
    # Prefer the serial-keyed by-id symlinks: stable across replug/renumber.
    byid_dir = "/dev/v4l/by-id"
    try:
        names = sorted(n for n in os.listdir(byid_dir) if n.endswith("-video-index0"))
    except OSError:
        names = []
    for name in names:
        if "RealSense" in name or "ZED" in name:   # skip by name without opening
            continue
        path = os.path.join(byid_dir, name)
        if _is_capture_webcam(path):
            return path                              # stable serial-keyed path
    # Fallback: scan bare nodes (older kernels / no by-id) and skip ZED/RealSense.
    for i in range(10):
        dev = f"/dev/video{i}"
        if os.path.exists(dev) and _is_capture_webcam(dev):
            return dev
    return WEBCAM_DEVICE_DEFAULT


def _depth_at_bbox(depth_data, x1, y1, x2, y2, r=5):
    """Median depth (metres) in a small patch at the bounding-box centre.

    depth_data: float32 numpy array from ZED MEASURE.DEPTH (metres, NaN/inf for invalid).
    Returns None if no valid pixels are found.
    """
    import numpy as np
    cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
    h, w   = depth_data.shape[:2]
    patch  = depth_data[max(0, cy - r):min(h, cy + r + 1),
                        max(0, cx - r):min(w, cx + r + 1)]
    valid  = patch[np.isfinite(patch) & (patch > 0.1) & (patch < 40.0)]
    return float(np.median(valid)) if len(valid) > 0 else None


def _start_rear_camera_thread(source="realsense", device=None):
    """Capture the rear camera directly via V4L2 — no ROS driver needed.

    source: 'zed' (ZED 2i via pyzed SDK, index ZED_REAR_INDEX) | 'webcam' (generic USB
    UVC, opened MJPG for full frame-rate at 720p). `device` optionally pins the V4L2
    device path/index for the webcam source; auto-detected when None.

    Writes to Handler._cam_jpeg / _cam_frame_count so /api/camera works even when no
    ROS camera node is running.  If the ROS subscriber is also active (realsense
    source), both paths write the same buffer; last write wins — harmless, same camera.
    """
    if source == "zed":
        _start_zed_rear_thread()
        return

    try:
        import cv2
        import numpy as np
    except ImportError:
        log.warning("[rear-cam] cv2 not available — rear camera direct capture disabled")
        return

    if source == "webcam":
        use_mjpg = True
        label    = "webcam (USB UVC)"
        # Re-resolved on every (re)connect so an unplug/replug re-binds to the
        # correct stable by-id path rather than whatever node it lands on.
        _resolve_dev = lambda: _find_webcam_device(device)
    else:
        use_mjpg = False
        label    = "webcam (V4L2)"
        _resolve_dev = lambda: (device if device is not None else WEBCAM_DEVICE_DEFAULT)

    def _capture_loop():
        _interval    = 1.0 / max(RS_DISPLAY_FPS, 0.1)
        last_display = 0.0
        retry_sleep  = 1.0   # exponential back-off on repeated failures
        cap = None
        dev = _resolve_dev()
        while True:
            if cap is None or not cap.isOpened():
                dev = _resolve_dev()              # re-bind on each reconnect (replug-safe)
                with _quiet_stderr():
                    cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
                if cap.isOpened():
                    if use_mjpg:   # webcams need MJPG for 30 fps at 720p
                        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  1280)
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
                    cap.set(cv2.CAP_PROP_FPS, 30)
                    # Verify the device can actually deliver a frame before announcing
                    with _quiet_stderr():
                        ok_test, _ = cap.read()
                    if not ok_test:
                        cap.release()
                        cap = None
                        with Handler._cam_lock:
                            Handler._cam_connected  = False
                            Handler._cam_last_error = f"{dev} opened but no frames"
                        time.sleep(retry_sleep)
                        retry_sleep = min(retry_sleep * 2, 5.0)
                        continue
                    retry_sleep = 1.0
                    log.info("[rear-cam] Opened %s (%s)", dev, label)
                else:
                    with Handler._cam_lock:
                        Handler._cam_connected  = False
                        Handler._cam_last_error = f"{dev} not available"
                    time.sleep(retry_sleep)
                    retry_sleep = min(retry_sleep * 2, 5.0)
                    continue
            with _quiet_stderr():
                ret, frame = cap.read()
            if not ret:
                with Handler._cam_lock:
                    Handler._cam_connected  = False
                    Handler._cam_last_error = "Frame read failed — reconnecting"
                cap.release()
                cap = None
                time.sleep(retry_sleep)
                retry_sleep = min(retry_sleep * 2, 5.0)
                continue

            now = time.monotonic()
            if now - last_display >= _interval:
                last_display = now
                ok, enc = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 75])
                if ok:
                    with Handler._cam_lock:
                        Handler._cam_jpeg            = enc.tobytes()
                        Handler._cam_frame_count     += 1
                        Handler._cam_connected       = True
                        Handler._cam_last_error      = None
                        Handler._cam_last_frame_time = time.monotonic()
                        n = Handler._cam_frame_count
                    if n == 1 or n % 300 == 0:
                        log.debug("[rear-cam] %d frames captured (%dx%d)",
                                  n, frame.shape[1], frame.shape[0])

    threading.Thread(target=_capture_loop, daemon=True, name="rear-cam").start()
    log.info("[rear-cam] Direct capture thread started (%s, %s)", _resolve_dev(), label)


def _start_zed_rear_thread():
    """Capture the ZED 2i rear camera (pyzed SDK index ZED_REAR_INDEX) for color.

    Mirrors _start_zed_thread: runs YOLO when detection is requested on the rear view.
    Mirrors _start_zed_thread including depth (DEPTH_MODE.PERFORMANCE) so detections
    carry both confidence and distance_m. Uses the shared YOLO singleton +
    _YOLO_INFER_LOCK so front and rear inferences are serialized and don't fight over the GPU.
    """
    try:
        import cv2
    except ImportError:
        log.warning("[rear-cam] cv2 not available — rear ZED capture disabled")
        return

    _min_infer_interval = 1.0 / max(YOLO_THROTTLE_HZ, 0.1)

    def _capture_loop(zed, image_mat, depth_mat):
        import pyzed.sl as sl
        yolo, yolo_dev, yolo_half = _get_shared_yolo()

        _inf_lock  = threading.Lock()
        _inf_frame = [None]   # (color_bgr, depth_float32 | None)
        _inf_event = threading.Event()

        def _rear_yolo_worker():
            while True:
                _inf_event.wait()
                _inf_event.clear()
                with _inf_lock:
                    data = _inf_frame[0]
                    _inf_frame[0] = None
                if data is None:
                    continue
                color, depth = data
                try:
                    with _YOLO_INFER_LOCK:
                        results = yolo(color, classes=[YOLO_PERSON_CLASS],
                                       conf=YOLO_CONFIDENCE, imgsz=YOLO_IMGSZ,
                                       device=yolo_dev, half=yolo_half,
                                       max_det=YOLO_MAX_DET, verbose=False)
                    boxes_half = []
                    detections = []
                    for result in results:
                        for box in result.boxes:
                            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                            conf = float(box.conf[0])
                            dist = _depth_at_bbox(depth, x1, y1, x2, y2) if depth is not None else None
                            label = f'person {conf:.0%}'
                            if dist is not None:
                                label += f'  {dist:.1f} m'
                            boxes_half.append((x1 // 2, y1 // 2, x2 // 2, y2 // 2, label))
                            detections.append({
                                'label':      'person',
                                'confidence': round(conf, 3),
                                'distance_m': round(dist, 2) if dist is not None else None,
                                'bbox':       [x1, y1, x2, y2],
                            })
                    det_payload = {
                        'ts':         time.time(),
                        'count':      len(detections),
                        'detections': detections,
                    }
                    with Handler._rear_det_lock:
                        Handler._rear_det_boxes   = boxes_half
                        Handler._rear_det_payload = det_payload
                except Exception as exc:
                    log.error("[rear-cam] YOLO error: %s", exc)

        if yolo is not None:
            threading.Thread(target=_rear_yolo_worker, daemon=True,
                             name="yolo-rear").start()

        runtime_params    = sl.RuntimeParameters()
        last_display      = 0.0
        last_queue        = 0.0
        _display_interval = 1.0 / STREAM_FPS
        retry_sleep       = 1.0
        lost              = 0

        while True:
            err = zed.grab(runtime_params)
            if err == sl.ERROR_CODE.SUCCESS:
                zed.retrieve_image(image_mat, sl.VIEW.LEFT)
                frame = image_mat.get_data()[:, :, :3]   # BGRA → BGR
                retry_sleep = 1.0
                lost        = 0
                now = time.monotonic()
                if now - last_display >= _display_interval:
                    last_display = now
                    small = cv2.resize(frame, (frame.shape[1] // 2, frame.shape[0] // 2))
                    ok, enc = cv2.imencode('.jpg', small, [cv2.IMWRITE_JPEG_QUALITY, 60])
                    if ok:
                        with Handler._cam_lock:
                            Handler._cam_jpeg            = enc.tobytes()
                            Handler._cam_frame           = small   # raw frame for rear detection
                            Handler._cam_frame_count    += 1
                            Handler._cam_connected       = True
                            Handler._cam_last_error      = None
                            Handler._cam_last_frame_time = time.monotonic()

                if yolo is not None and Handler._rear_detection_wanted():
                    if now - last_queue >= _min_infer_interval:
                        last_queue = now
                        zed.retrieve_measure(depth_mat, sl.MEASURE.DEPTH)
                        depth_arr = depth_mat.get_data().copy()
                        with _inf_lock:
                            _inf_frame[0] = (frame.copy(), depth_arr)
                        _inf_event.set()

            elif err == sl.ERROR_CODE.END_OF_SVOFILE_REACHED:
                zed.set_svo_position(0)
            else:
                with Handler._cam_lock:
                    Handler._cam_connected  = False
                    Handler._cam_last_error = f"rear ZED grab: {err}"
                lost += 1
                if lost >= ZED_GRAB_LOST_LIMIT:
                    log.warning("[rear-cam] rear ZED grab failing (%s) — releasing to re-open", err)
                    log_event("WARN", "Camera",
                              f"Rear ZED camera grab failed ({err}) — reconnecting",
                              "Unplug and replug the rear ZED 2i USB-C cable. "
                              "Check: lsusb | grep STEREOLABS",
                              _key="rear-zed-grab", _debounce_s=30)
                    return
                time.sleep(retry_sleep)
                retry_sleep = min(retry_sleep * 2, 5.0)

    def _start():
        try:
            import ctypes
            ctypes.CDLL('/lib/aarch64-linux-gnu/libusb-1.0.so.0').libusb_init(None)
            import pyzed.sl as sl
        except ImportError:
            log.warning("[rear-cam] pyzed not installed — rear ZED capture disabled")
            return
        except Exception as exc:
            log.warning("[rear-cam] libusb/pyzed init failed (%s) — rear ZED disabled", exc)
            return

        attempt = 0
        delay   = ZED_SDK_OPEN_RETRY_DELAY
        while True:
            attempt += 1
            zed = None
            try:
                zed         = sl.Camera()
                init_params = sl.InitParameters()
                init_params.camera_resolution = sl.RESOLUTION.HD720
                init_params.camera_fps        = 30
                init_params.depth_mode        = sl.DEPTH_MODE.PERFORMANCE
                init_params.coordinate_units  = sl.UNIT.METER
                # Select the second camera (pyzed 4.x API; 3.x uses camera_linux_id)
                try:
                    init_params.input.set_from_camera_index(ZED_REAR_INDEX)
                except AttributeError:
                    try:
                        init_params.camera_linux_id = ZED_REAR_INDEX
                    except AttributeError:
                        pass
                err = zed.open(init_params)
                if err == sl.ERROR_CODE.SUCCESS:
                    image_mat = sl.Mat()
                    depth_mat = sl.Mat()
                    log.info("[rear-cam] ZED 2i rear (SDK index %d) opened", ZED_REAR_INDEX)
                    delay = ZED_SDK_OPEN_RETRY_DELAY
                    _capture_loop(zed, image_mat, depth_mat)
                    err = "camera lost"
                else:
                    with Handler._cam_lock:
                        Handler._cam_last_error = f"rear ZED SDK open: {err}"
            except Exception as exc:
                err = exc
                with Handler._cam_lock:
                    Handler._cam_last_error = f"rear ZED SDK: {exc}"

            with Handler._cam_lock:
                Handler._cam_connected = False
            try:
                if zed is not None:
                    zed.close()
            except Exception:
                pass

            if attempt <= ZED_SDK_OPEN_RETRIES or attempt % 10 == 0:
                log.warning("[rear-cam] rear ZED open failed (%s) — retrying in %.1fs",
                            err, delay)
                log_event("WARN", "Camera",
                          f"Rear ZED camera unavailable (attempt {attempt}): {err}",
                          "Check USB-C cable on the rear ZED 2i. "
                          "Run: lsusb | grep STEREOLABS — two entries expected.",
                          _key="rear-zed-open", _debounce_s=60)
            time.sleep(delay)
            delay = min(delay * 1.5, 5.0)

    threading.Thread(target=_start, daemon=True, name="rear-cam").start()
    log.info("[rear-cam] ZED 2i rear capture thread started (SDK, index %d)", ZED_REAR_INDEX)


# Person-detection (front/ZED feed), tuned for the Jetson Orin GPU.
# The CPU cost of detection is dominated by fixed per-call overhead (~18 ms/call),
# Detection runs entirely on the GPU (CUDA FP16). CPU is only used for the
# thin Python glue: queuing frames (numpy copy) and writing the JPEG + JSON.
# Model: yolov8n (nano) — 3-4× faster than small on the same GPU with
# negligible accuracy loss for nearby persons.
# imgsz=320: 4× fewer NMS candidates than 640, which eliminates the
# "NMS time limit exceeded" warning seen with the larger model+resolution.
# max_det=20: further caps NMS work (unlikely to see >20 people at once).
# At 10 Hz throttle and ~30-80 ms GPU inference, detection lag ≈ 100-200 ms.
YOLO_MODEL        = str(DASHBOARD_DIR.parent / 'models' / 'yolov8n.pt')   # nano: 3-4× faster than small, GPU FP16
YOLO_CONFIDENCE   = 0.5
YOLO_PERSON_CLASS = 0
YOLO_THROTTLE_HZ  = 10.0           # 10 Hz — achievable now that inference is fast
YOLO_IMGSZ        = 320            # 320: fast NMS, good enough for nearby persons
YOLO_MAX_DET      = 20             # cap NMS candidates
YOLO_DEVICE       = 0              # CUDA device index (GPU-only; warns loudly if unavailable)
YOLO_HALF         = True           # FP16 — halves GPU memory and speeds matmul


def _load_yolo(label=""):
    """Load YOLO on the GPU (CUDA FP16), limit CPU threads, and warm up.

    Returns (model, device, half) on success, (None, 'cpu', False) on failure.
    torch.set_num_threads(1): all heavy math runs on the GPU; spawning one
    intra-op thread per CPU core only causes cache thrashing with no benefit.
    """
    try:
        import numpy as np
        import torch
        from ultralytics import YOLO
        torch.set_num_threads(1)
        if not torch.cuda.is_available():
            log.error("[yolo] CUDA unavailable — detection will NOT run. "
                      "Check that the Jetson's CUDA drivers are installed.")
            log_event("ERROR", "Camera",
                      "YOLO person detection disabled — CUDA GPU not available",
                      "Check Jetson GPU driver: nvidia-smi. "
                      "If unavailable, reinstall the Jetson CUDA toolkit. "
                      "Detection will not run until CUDA is available.",
                      _key="yolo-cuda", _debounce_s=3600)
            return None, "cpu", False
        dev, half = YOLO_DEVICE, YOLO_HALF
        model = YOLO(YOLO_MODEL)
        model.to(f'cuda:{dev}')   # pin model to GPU before warm-up
        model(np.zeros((YOLO_IMGSZ, YOLO_IMGSZ, 3), dtype=np.uint8),
              classes=[YOLO_PERSON_CLASS], imgsz=YOLO_IMGSZ, device=dev,
              half=half, max_det=YOLO_MAX_DET, verbose=False)
        gpu_name = torch.cuda.get_device_name(dev)
        log.info("[yolo] %s'%s' on GPU:%d (%s) FP16=%s imgsz=%d %.0f Hz — GPU-only confirmed",
                 f"{label} " if label else "", YOLO_MODEL, dev, gpu_name,
                 half, YOLO_IMGSZ, YOLO_THROTTLE_HZ)
        return model, dev, half
    except Exception as exc:
        log.warning("[yolo] model unavailable — %s", exc)
        log_event("WARN", "Camera",
                  f"YOLO model failed to load: {exc}",
                  f"Check that '{YOLO_MODEL}' exists in the project root. "
                  f"Download with: python3 -c \"from ultralytics import YOLO; YOLO('{YOLO_MODEL}')\"",
                  _key="yolo-model", _debounce_s=3600)
        return None, "cpu", False

# ZED SDK is the only color path; the OpenCV fallback is grayscale on this Jetson.
# The SDK open can fail transiently ("CAMERA NOT DETECTED") if the camera is still
# releasing from a prior (unclean) shutdown or a V4L2 process briefly holds it, so
# retry before giving up to grayscale instead of falling back on the first failure.
ZED_SDK_OPEN_RETRIES     = 6
ZED_SDK_OPEN_RETRY_DELAY = 2.5   # seconds between attempts (~15 s total)
ZED_GRAB_LOST_LIMIT      = 15    # consecutive grab failures → treat camera as lost, re-open


# ---------------------------------------------------------------------------
# Shared YOLO singleton — loaded once; both front and rear ZED threads share it.
# Inference is serialized under _YOLO_INFER_LOCK so neither camera stalls the
# GPU with concurrent calls (yolov8n is too small to benefit from parallelism).
# ---------------------------------------------------------------------------
_YOLO_MODEL      = None
_YOLO_DEV        = 'cpu'
_YOLO_HALF       = False
_YOLO_LOAD_LOCK  = threading.Lock()
_YOLO_INFER_LOCK = threading.Lock()


def _get_shared_yolo():
    """Return (model, dev, half), loading the singleton on first call."""
    global _YOLO_MODEL, _YOLO_DEV, _YOLO_HALF
    with _YOLO_LOAD_LOCK:
        if _YOLO_MODEL is None:
            _YOLO_MODEL, _YOLO_DEV, _YOLO_HALF = _load_yolo("(shared)")
    return _YOLO_MODEL, _YOLO_DEV, _YOLO_HALF


def _draw_box(img, x1, y1, x2, y2, label):
    """Draw a red bounding box + label on img in-place (used by YOLO worker and detection stream)."""
    import cv2
    cv2.rectangle(img, (x1, y1), (x2, y2), (0, 0, 220), 2)
    (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2)
    cv2.rectangle(img, (x1, y1 - th - 8), (x1 + tw + 6, y1), (0, 0, 220), -1)
    cv2.putText(img, label, (x1 + 3, y1 - 4),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)


def _start_zed_thread():
    """Capture the ZED 2i front feed in COLOR via the pyzed SDK, which identifies the
    camera by USB serial (deterministic) and yields a single rectified left-eye image
    (no split, no mirror). The SDK open is retried until the camera is available, so
    the feed is always color — the grayscale OpenCV fallback runs ONLY when the pyzed
    SDK is not installed at all."""
    try:
        import cv2
        import numpy as np
    except ImportError:
        log.warning("[zed] cv2 not available — /api/zed will return 503")
        return

    _min_infer_interval = 1.0 / max(YOLO_THROTTLE_HZ, 0.1)

    # ── pyzed SDK capture (full color + depth for YOLO distance) ─────────────
    def _capture_loop_sdk(zed, image_mat):
        import pyzed.sl as sl
        import numpy as np
        yolo, yolo_dev, yolo_half = _get_shared_yolo()
        depth_mat = sl.Mat()   # reused each grab; retrieve_measure fills it in-place

        # ── YOLO worker thread ────────────────────────────────────────────────
        # Inference can take >1 s (NMS on large scenes), which would stall grab()
        # and trigger ZED grab-lost errors if done inline. The worker runs at its
        # own pace: the grab loop writes the latest frame and signals; the worker
        # always picks up the most recent frame (last-write-wins, no queue build-up).
        _inf_lock  = threading.Lock()
        _inf_frame = [None]        # (color_bgr_copy, depth_float32_copy) or None
        _inf_event = threading.Event()

        def _yolo_worker():
            while True:
                _inf_event.wait()
                _inf_event.clear()
                with _inf_lock:
                    data = _inf_frame[0]
                    _inf_frame[0] = None
                if data is None:
                    continue
                color, depth = data
                try:
                    with _YOLO_INFER_LOCK:
                        results = yolo(color, classes=[YOLO_PERSON_CLASS],
                                       conf=YOLO_CONFIDENCE, imgsz=YOLO_IMGSZ,
                                       device=yolo_dev, half=yolo_half,
                                       max_det=YOLO_MAX_DET, verbose=False)
                    # Store boxes at stream-res (÷2) — the detection stream composites
                    # them onto the live camera frame so video is always real-time.
                    boxes_half = []
                    detections = []
                    for result in results:
                        for box in result.boxes:
                            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                            conf = float(box.conf[0])
                            dist = _depth_at_bbox(depth, x1, y1, x2, y2)
                            label = f'person {conf:.0%}'
                            if dist is not None:
                                label += f'  {dist:.1f} m'
                            boxes_half.append((x1 // 2, y1 // 2, x2 // 2, y2 // 2, label))
                            detections.append({
                                'label':      'person',
                                'confidence': round(conf, 3),
                                'distance_m': round(dist, 2) if dist is not None else None,
                                'bbox':       [x1, y1, x2, y2],
                            })
                    det_payload = {
                        'ts':         time.time(),
                        'count':      len(detections),
                        'detections': detections,
                    }
                    with Handler._det_lock:
                        Handler._det_boxes   = boxes_half
                        Handler._det_payload = det_payload
                    try:
                        Path(DETECTIONS_FILE).write_text(json.dumps(det_payload))
                    except Exception:
                        pass
                except Exception as exc:
                    log.error("[zed] YOLO error: %s", exc)

        if yolo is not None:
            threading.Thread(target=_yolo_worker, daemon=True, name="yolo-infer").start()

        # ── ZED grab loop — must call grab() continuously at camera rate ──────
        runtime_params    = sl.RuntimeParameters()
        last_queue        = 0.0    # last time we queued a frame for the YOLO worker
        last_display      = 0.0
        _display_interval = 1.0 / STREAM_FPS
        retry_sleep       = 1.0
        lost              = 0

        while True:
            err = zed.grab(runtime_params)
            if err == sl.ERROR_CODE.SUCCESS:
                zed.retrieve_image(image_mat, sl.VIEW.LEFT)
                # get_data() returns BGRA (4-channel); drop alpha
                left = image_mat.get_data()[:, :, :3]

                Handler._set_zed_status(True)
                retry_sleep = 1.0
                lost        = 0

                now = time.monotonic()
                if now - last_display >= _display_interval:
                    last_display = now
                    # Halve resolution for the stream — cuts per-frame size ~4× and
                    # prevents the OS/browser from buffering stale frames.
                    small = cv2.resize(left, (left.shape[1] // 2, left.shape[0] // 2))
                    ok, enc = cv2.imencode('.jpg', small, [cv2.IMWRITE_JPEG_QUALITY, 60])
                    if ok:
                        with Handler._zed_lock:
                            Handler._zed_jpeg            = enc.tobytes()
                            Handler._zed_frame           = small   # raw frame for detection compositing
                            Handler._zed_frame_count    += 1
                            Handler._zed_last_frame_time = time.monotonic()

                # Queue a frame for the YOLO worker (non-blocking — never stalls grab).
                # retrieve_measure is fast (buffer copy, computed at grab time).
                if yolo is not None and Handler._detection_wanted():
                    if now - last_queue >= _min_infer_interval:
                        last_queue = now
                        zed.retrieve_measure(depth_mat, sl.MEASURE.DEPTH)
                        with _inf_lock:
                            _inf_frame[0] = (left.copy(), depth_mat.get_data().copy())
                        _inf_event.set()

            elif err == sl.ERROR_CODE.END_OF_SVOFILE_REACHED:
                zed.set_svo_position(0)
            else:
                Handler._set_zed_status(False, f"grab error: {err}")
                lost += 1
                # Camera unplugged / hung: stop grabbing and return so the caller
                # closes this handle and re-opens (by serial) when it reconnects.
                if lost >= ZED_GRAB_LOST_LIMIT:
                    log.warning("[zed] grab failing (%s) — releasing to re-open on reconnect", err)
                    log_event("WARN", "Camera",
                              f"Front ZED camera grab failed ({err}) — reconnecting",
                              "Unplug and replug the front ZED 2i USB-C cable. "
                              "Check: lsusb | grep STEREOLABS",
                              _key="front-zed-grab", _debounce_s=30)
                    return
                time.sleep(retry_sleep)
                retry_sleep = min(retry_sleep * 2, 5.0)

    # ── OpenCV fallback (grayscale — YUYV color bug on Jetson pip OpenCV) ─────
    def _capture_loop_opencv():
        yolo, yolo_dev, yolo_half = _get_shared_yolo()

        # Same worker-thread pattern as the SDK path so capture isn't stalled.
        _inf_lock  = threading.Lock()
        _inf_frame = [None]
        _inf_event = threading.Event()

        def _yolo_worker_cv():
            while True:
                _inf_event.wait()
                _inf_event.clear()
                with _inf_lock:
                    color = _inf_frame[0]
                    _inf_frame[0] = None
                if color is None:
                    continue
                try:
                    with _YOLO_INFER_LOCK:
                        results = yolo(color, classes=[YOLO_PERSON_CLASS],
                                       conf=YOLO_CONFIDENCE, imgsz=YOLO_IMGSZ,
                                       device=yolo_dev, half=yolo_half,
                                       max_det=YOLO_MAX_DET, verbose=False)
                    boxes_half = []
                    detections = []
                    for result in results:
                        for box in result.boxes:
                            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                            conf = float(box.conf[0])
                            boxes_half.append((x1 // 2, y1 // 2, x2 // 2, y2 // 2,
                                               f'person {conf:.0%}'))
                            detections.append({
                                'label':      'person',
                                'confidence': round(conf, 3),
                                'distance_m': None,
                                'bbox':       [x1, y1, x2, y2],
                            })
                    det_payload = {
                        'ts': time.time(), 'count': len(detections), 'detections': detections,
                    }
                    with Handler._det_lock:
                        Handler._det_boxes   = boxes_half
                        Handler._det_payload = det_payload
                    try:
                        Path(DETECTIONS_FILE).write_text(json.dumps(det_payload))
                    except Exception:
                        pass
                except Exception as exc:
                    log.error("[zed] YOLO error: %s", exc)

        if yolo is not None:
            threading.Thread(target=_yolo_worker_cv, daemon=True, name="yolo-infer").start()

        def _device_index():
            try:
                target = os.readlink(ZED_DEVICE)
                return int(target.replace("video", ""))
            except Exception:
                return ZED_DEVICE

        def _fix_yuyv(frame):
            # pip OpenCV NEON YUYV→BGR zeros UV on Jetson: G channel = Y (luminance only).
            gray = frame[:, :, 1]
            return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

        cap          = None
        last_queue   = 0.0
        last_display = 0.0
        _display_interval = 1.0 / STREAM_FPS
        retry_sleep  = 1.0

        while True:
            if cap is None or not cap.isOpened():
                with _quiet_stderr():
                    cap = cv2.VideoCapture(_device_index())
                if cap.isOpened():
                    Handler._set_zed_status(True)
                    retry_sleep = 1.0
                    log.info("[zed] Opened %s via OpenCV (grayscale)", ZED_DEVICE)
                else:
                    Handler._set_zed_status(False, f"{ZED_DEVICE} not available")
                    time.sleep(retry_sleep)
                    retry_sleep = min(retry_sleep * 2, 5.0)
                    continue

            with _quiet_stderr():
                ret, frame = cap.read()
            if not ret:
                Handler._set_zed_status(False, "Frame read failed — reconnecting")
                cap.release()
                cap = None
                time.sleep(retry_sleep)
                retry_sleep = min(retry_sleep * 2, 5.0)
                continue

            h, w = frame.shape[:2]
            left = _fix_yuyv(frame[:, :w // 2])

            now = time.monotonic()
            if now - last_display >= _display_interval:
                last_display = now
                small = cv2.resize(left, (left.shape[1] // 2, left.shape[0] // 2))
                ok, enc = cv2.imencode('.jpg', small, [cv2.IMWRITE_JPEG_QUALITY, 60])
                if ok:
                    with Handler._zed_lock:
                        Handler._zed_jpeg            = enc.tobytes()
                        Handler._zed_frame           = small
                        Handler._zed_frame_count    += 1
                        Handler._zed_last_frame_time = time.monotonic()

            if yolo is not None and Handler._detection_wanted():
                if now - last_queue >= _min_infer_interval:
                    last_queue = now
                    with _inf_lock:
                        _inf_frame[0] = left.copy()
                    _inf_event.set()

    # ── ZED front feed: SDK only (color), retried forever. The SDK identifies the
    #    camera by its USB serial, so the front view is deterministic — it is always
    #    the ZED, never another camera, and never the grayscale V4L2 fallback. If the
    #    ZED is absent/busy the front reports "unavailable" and auto-recovers on
    #    reconnect. The grayscale OpenCV fallback runs ONLY when pyzed is not installed.
    def _start():
        try:
            # libusb 1.0.25 (Ubuntu 22.04) doesn't auto-init the default context;
            # ZED SDK calls libusb_get_device_list(NULL) which segfaults without it.
            import ctypes
            ctypes.CDLL('/lib/aarch64-linux-gnu/libusb-1.0.so.0').libusb_init(None)
            import pyzed.sl as sl
        except ImportError:
            log.warning("[zed] pyzed SDK not installed — cannot guarantee color; "
                        "using OpenCV grayscale fallback")
            _capture_loop_opencv()
            return
        except Exception as exc:
            log.warning("[zed] libusb/pyzed init failed (%s) — OpenCV grayscale fallback", exc)
            _capture_loop_opencv()
            return

        attempt = 0
        delay   = ZED_SDK_OPEN_RETRY_DELAY
        while True:                       # retry forever — color or nothing, never grayscale
            attempt += 1
            zed = None
            try:
                zed         = sl.Camera()
                init_params = sl.InitParameters()
                init_params.camera_resolution = getattr(sl.RESOLUTION, ZED_FRONT_RESOLUTION)
                init_params.camera_fps        = ZED_FRONT_FPS
                # PERFORMANCE depth enables distance readout in YOLO detection.
                init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
                try:
                    init_params.coordinate_units = sl.UNIT.METER
                except AttributeError:
                    pass
                # Front camera is always the default (index 0); don't call
                # set_from_camera_index here — it doesn't exist in pyzed 3.x
                # and would crash the retry loop on that SDK version.

                err = zed.open(init_params)
                if err == sl.ERROR_CODE.SUCCESS:
                    image_mat = sl.Mat()
                    log.info("[zed] ZED 2i front SDK opened — color+depth active")
                    delay = ZED_SDK_OPEN_RETRY_DELAY
                    _capture_loop_sdk(zed, image_mat)   # returns if the camera is lost
                    # capture loop exited (camera unplugged) — release and re-open below
                    err = "camera lost"
                else:
                    Handler._set_zed_status(False, f"SDK open: {err}")
            except Exception as exc:
                err = exc
                Handler._set_zed_status(False, f"SDK open: {exc}")

            with Handler._zed_lock:
                Handler._zed_connected = False   # keep the last error message for /api/zed/status
            try:
                if zed is not None:
                    zed.close()   # release before retrying so the next open can detect it
            except Exception:
                pass

            # Stay quiet after the first few failures to avoid log spam while idle.
            if attempt <= ZED_SDK_OPEN_RETRIES or attempt % 10 == 0:
                log.warning("[zed] SDK open failed (%s) — retrying in %.1fs (color-only, "
                            "no grayscale)", err, delay)
                log_event("WARN", "Camera",
                          f"Front ZED camera unavailable (attempt {attempt}): {err}",
                          "Check USB-C cable on the front ZED 2i. "
                          "Run: lsusb | grep STEREOLABS — two entries expected (one per camera).",
                          _key="front-zed-open", _debounce_s=60)
            time.sleep(delay)
            delay = min(delay * 1.5, 5.0)

    threading.Thread(target=_start, daemon=True, name="zed-capture").start()
    log.info("[zed] Capture thread started (%s)", ZED_DEVICE)


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
        global ZED_FRONT_RESOLUTION, ZED_FRONT_FPS
        ZED_FRONT_RESOLUTION = "HD2K"
        ZED_FRONT_FPS        = 15    # HD2K hardware cap
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
        def _vel_publish_once():
            with Handler._vel_lock:
                if not Handler._vel_active:
                    return None
                if time.monotonic() - Handler._vel_last > Handler.VEL_TIMEOUT:
                    Handler._vel_active = False
                    return (0.0, 0.0)
                lx = Handler._vel_lin
                az = Handler._vel_ang
                if lx == 0.0 and az == 0.0:
                    Handler._vel_active = False
                return (lx, az)

        def _vel_loop():
            period = 0.05   # 20 Hz
            while True:
                t0 = time.monotonic()
                try:
                    cmd = _vel_publish_once()
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
                        with Handler._cam_lock:
                            Handler._cam_jpeg            = frame
                            Handler._cam_frame_count     += 1
                            Handler._cam_connected       = True
                            Handler._cam_last_error      = None
                            Handler._cam_last_frame_time = time.monotonic()
                            n = Handler._cam_frame_count
                        if n == 1 or n % 300 == 0:
                            log.debug("[camera] %d frames received (%dx%d)",
                                      n, msg.width, msg.height)
                except Exception as exc:
                    err = f"{type(exc).__name__}: {exc}"
                    with Handler._cam_lock:
                        Handler._cam_last_error = err
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
        _start_rear_camera_thread(rear_src, CHASSIS.rear_camera_device)
    _start_zed_thread()

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
