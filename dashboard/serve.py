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
import contextlib
import json
import logging
import os
import socketserver
import threading
import time
from datetime import datetime, timezone
from http.server import SimpleHTTPRequestHandler
from pathlib import Path

# Chassis abstraction (agrobot | jackal). serve.py runs as `python3 dashboard/serve.py`,
# so its own directory is on sys.path; the tests insert dashboard/ as well.
try:
    import chassis
except ImportError:                       # imported as a package (dashboard.serve)
    from dashboard import chassis

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%Y-%m-%dT%H:%M:%S",
)
log = logging.getLogger("dashboard")


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
GNSS_FILE_DEFAULT    = "/tmp/gnss_coords.json"
DETECTIONS_FILE      = "/tmp/object_detections.json"
ZED_DEVICE           = "/dev/zed2i"
RS_DEVICE_DEFAULT    = "/dev/video4"   # RealSense D435 color stream (YUYV)
WEBCAM_DEVICE_DEFAULT = "/dev/video0"  # generic USB UVC webcam (e.g. Logitech)
RS_DISPLAY_FPS       = 30.0   # live camera capture / display
RECORD_FPS           = 15.0   # rear.mp4 + front.mp4 saved at 15 fps (lighter on the Jetson)
STREAM_FPS           = 30.0   # live MJPEG stream to the browser at 30 fps

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
    lin = int(linear_x  * LINEAR_SCALE)
    ang = int(angular_z * ANGULAR_SCALE)
    left  = max(-SPEED_MAX, min(SPEED_MAX, lin - ang))
    right = max(-SPEED_MAX, min(SPEED_MAX, lin + ang))
    return left, right


def _dms(value, is_lat):
    """Format a decimal-degree coordinate as degrees-minutes-seconds with a
    hemisphere suffix, e.g. 51.5074 -> 51°30'26.64\"N.  Returns "" for None."""
    if value is None:
        return ""
    hemi = ("N" if value >= 0 else "S") if is_lat else ("E" if value >= 0 else "W")
    v = abs(float(value))
    deg = int(v)
    minutes_full = (v - deg) * 60.0
    minutes = int(minutes_full)
    seconds = (minutes_full - minutes) * 60.0
    return f"{deg}°{minutes:02d}'{seconds:05.2f}\"{hemi}"


# ---------------------------------------------------------------------------
# HTTP handler
# ---------------------------------------------------------------------------
class Handler(SimpleHTTPRequestHandler):
    gnss_file: str   = GNSS_FILE_DEFAULT
    speed_cmd_pub    = None   # rclpy publisher, set by main() after rclpy.init()
    chassis          = None   # active chassis.Chassis, set by main(); None in tests

    _cam_lock            = threading.Lock()
    _cam_jpeg: bytes     = None
    _cam_frame_count     = 0
    _cam_last_error      = None
    _cam_connected       = False
    _cam_last_frame_time = 0.0   # kept for MJPEG stream throttling

    _det_lock         = threading.Lock()
    _det_jpeg: bytes  = None

    _zed_lock            = threading.Lock()
    _zed_jpeg: bytes     = None
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

    _settings_lock = threading.Lock()
    _settings: dict = {
        'maxLinear':  2.0,    # absolute max forward speed m/s (= Fast preset)
        'maxAngular': 0.5,    # max turn rate rad/s
        'modbusSpeed': 1500,  # raw Modbus units for Normal preset (mirrors slLinear/4 * LINEAR_SCALE)
    }

    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(DASHBOARD_DIR), **kwargs)

    def do_GET(self):
        if self.path == "/api/wheel_odom":
            self._serve_wheel_odom()
        elif self.path == "/api/chassis_battery":
            self._serve_chassis_battery()
        elif self.path == "/api/gnss":
            self._serve_gnss()
        elif self.path == "/api/camera/status":
            self._serve_camera_status()
        elif self.path.startswith("/api/camera/stream"):
            self._serve_camera_stream()
        elif self.path.startswith("/api/camera"):
            self._serve_camera()
        elif self.path == "/api/zed/status":
            self._serve_zed_status()
        elif self.path.startswith("/api/zed/stream"):
            self._serve_zed_stream()
        elif self.path.startswith("/api/zed"):
            self._serve_zed()
        elif self.path == "/api/detection/data":
            self._serve_detection_data()
        elif self.path.startswith("/api/detection/stream"):
            self._serve_detection_stream()
        elif self.path.startswith("/api/detection"):
            self._serve_detection_image()
        elif self.path == '/api/settings':
            self._serve_settings_get()
        elif self.path == '/api/config':
            self._serve_config()
        else:
            # Only serve the dashboard HTML and logo assets; everything else
            # (including serve.py) returns 403 to avoid exposing server internals.
            p = self.path.split('?')[0].split('#')[0]
            if p in ('/', '/index.html') or p.startswith('/logo/'):
                super().do_GET()
            else:
                log.warning("Blocked static path %s from %s", p, self.client_address[0])
                self.send_error(403)

    def do_POST(self):
        if self.path == "/api/cmd_vel":
            self._serve_cmd_vel()
        elif self.path == "/api/track/save":
            self._serve_track_save()
        elif self.path == "/api/plant":
            self._serve_plant_log()
        elif self.path == "/api/record/start":
            self._serve_record_start()
        elif self.path == "/api/record/stop":
            self._serve_record_stop()
        elif self.path == '/api/settings':
            self._serve_settings_post()
        elif self.path == '/api/fwd2m':
            self._serve_fwd2m()
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
        body = json.dumps({
            "connected":       connected,
            "frames_received": count,
            "last_error":      err,
        }).encode()
        self._json_response(200, body)

    def _serve_detection_image(self):
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
        try:
            data = Path(DETECTIONS_FILE).read_text()
            self._json_response(200, data.encode())
        except FileNotFoundError:
            self._json_response(404, b'{"error":"No detections yet"}')
        except Exception as e:
            self._json_response(500, json.dumps({"error": str(e)}).encode())

    def _stream_jpeg(self, get_frame_fn):
        """Push JPEG frames as multipart/x-mixed-replace at STREAM_FPS.

        Holds the connection open; exits when the client disconnects.
        Only sends a frame when a new one is available (identity check on bytes object).
        """
        interval = 1.0 / STREAM_FPS
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store")
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
        def get_frame():
            with Handler._det_lock:
                return Handler._det_jpeg
        self._stream_jpeg(get_frame)

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

    def _serve_config(self):
        """GET /api/config — chassis name, comms, feature flags, and limits so
        the web UI can adapt (show/hide chassis-specific panels)."""
        if Handler.chassis is not None:
            body = json.dumps(Handler.chassis.to_browser_config()).encode()
        else:
            # No chassis loaded (e.g. in tests) — report module defaults.
            body = json.dumps({
                "chassis":  "unknown",
                "comms":    "modbus_speed",
                "features": {},
                "limits":   {"maxLinear": MAX_LIN_INPUT, "maxAngular": MAX_ANG_INPUT},
            }).encode()
        self._json_response(200, body)

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

                if traveled >= FWD_2M_PULSES:
                    break

                # Two-phase: slow crawl for the final 0.5 m to eliminate coasting overshoot
                remaining  = FWD_2M_PULSES - traveled
                next_speed = FWD_SLOW_SPEED if remaining <= FWD_SLOW_PULSES else speed

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

        done   = (not aborted) and (traveled >= FWD_2M_PULSES)
        result = {"done": done, "aborted": aborted,
                  "traveled_m": round(traveled / PULSE_PER_M, 3)}
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
        seed_dir = DASHBOARD_DIR.parent / "logs" / "planted_seedlings"
        seed_dir.mkdir(parents=True, exist_ok=True)
        # One cumulative line-delimited JSON log; each seedling carries both
        # decimal degrees and a human-readable DMS geo-coordinate.
        entry = {
            "index":     data["count"],
            "ts":        data["ts"],
            "chassis":   chassis_name,
            "lat":       lat,
            "lon":       lon,
            "lat_dms":   _dms(lat, True),
            "lon_dms":   _dms(lon, False),
            "fix":       data.get("fix"),
            "fix_label": data.get("fix_label"),
            "sats":      data.get("sats"),
            "hdop":      data.get("hdop"),
            "alt":       data.get("alt"),
        }
        seed_log = seed_dir / "seedlings.jsonl"
        try:
            with open(seed_log, "a", encoding="utf-8") as f:
                f.write(json.dumps(entry, ensure_ascii=False) + "\n")
            log.info("[seedling] #%s at %s %s (%s) → %s",
                     data["count"], entry["lat_dms"], entry["lon_dms"], chassis_name, seed_log)
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

    def log_message(self, fmt, *args):
        if args and str(args[1]) in ("200", "304"):
            return
        if args and str(args[1]) in ("200", "503") and "/api/camera" in str(args[0]):
            return
        if args and str(args[1]) in ("200", "503", "404") and "/api/detection" in str(args[0]):
            return
        if args and str(args[1]) in ("200", "503") and "/api/zed" in str(args[0]):
            return
        if args and str(args[1]) == "404":
            path = str(args[0])
            if any(x in path for x in ("/api/gnss", "favicon", ".svg", ".ico", ".png")):
                return
        super().log_message(fmt, *args)


class _Server(socketserver.ThreadingMixIn, socketserver.TCPServer):
    allow_reuse_address = True
    daemon_threads      = True


def _recording_loop():
    """Write rear and front cameras to MP4 at RECORD_FPS."""
    try:
        import cv2
        import numpy as np
    except ImportError:
        log.error("[record] cv2 not available — cannot record video")
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


def _find_webcam_device(preferred=None):
    """Return a generic USB UVC webcam device (e.g. Logitech): the preferred one if
    given, else the first /dev/videoN that is a capture device and is NOT a RealSense."""
    if preferred is not None:
        return preferred
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
            if 'RealSense' in info:
                continue
            if 'Video Capture' in info or 'MJPG' in info or 'YUYV' in info:
                return dev
        except Exception:
            pass
    return WEBCAM_DEVICE_DEFAULT


def _start_rear_camera_thread(source="realsense", device=None):
    """Capture the rear camera directly via V4L2 — no ROS driver needed.

    source: 'realsense' (D435 color, YUYV) | 'webcam' (generic USB UVC, e.g. Logitech;
    opened MJPG for full frame-rate at 720p). `device` optionally pins the V4L2
    device path/index; auto-detected when None.

    Writes to Handler._cam_jpeg / _cam_frame_count so /api/camera works even when no
    ROS camera node is running.  If the ROS subscriber is also active (realsense
    source), both paths write the same buffer; last write wins — harmless, same camera.
    """
    try:
        import cv2
        import numpy as np
    except ImportError:
        log.warning("[rear-cam] cv2 not available — rear camera direct capture disabled")
        return

    if source == "webcam":
        dev      = _find_webcam_device(device)
        use_mjpg = True
        label    = "webcam (USB UVC)"
    else:
        dev      = device if device is not None else _find_rs_device()
        use_mjpg = False
        label    = "RealSense color"

    def _capture_loop():
        _interval    = 1.0 / max(RS_DISPLAY_FPS, 0.1)
        last_display = 0.0
        retry_sleep  = 1.0   # exponential back-off on repeated failures
        cap = None
        while True:
            if cap is None or not cap.isOpened():
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
    log.info("[rear-cam] Direct capture thread started (%s, %s)", dev, label)


YOLO_MODEL        = 'yolov8s.pt'
YOLO_CONFIDENCE   = 0.5
YOLO_PERSON_CLASS = 0
YOLO_THROTTLE_HZ  = 10.0


def _start_zed_thread():
    """Capture ZED 2i frames via pyzed SDK (color) or OpenCV fallback (grayscale)."""
    try:
        import cv2
        import numpy as np
    except ImportError:
        log.warning("[zed] cv2 not available — /api/zed will return 503")
        return

    _min_infer_interval = 1.0 / max(YOLO_THROTTLE_HZ, 0.1)

    def _draw_box(img, x1, y1, x2, y2, label):
        cv2.rectangle(img, (x1, y1), (x2, y2), (0, 0, 220), 2)
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2)
        cv2.rectangle(img, (x1, y1 - th - 8), (x1 + tw + 6, y1), (0, 0, 220), -1)
        cv2.putText(img, label, (x1 + 3, y1 - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)

    # ── pyzed SDK capture (full color) ────────────────────────────────────────
    def _capture_loop_sdk(zed, image_mat):
        import pyzed.sl as sl
        yolo = None
        try:
            from ultralytics import YOLO
            yolo = YOLO(YOLO_MODEL)
            yolo(np.zeros((480, 640, 3), dtype=np.uint8),
                 classes=[YOLO_PERSON_CLASS], verbose=False)
            log.info("[zed] YOLO model '%s' ready (SDK path)", YOLO_MODEL)
        except Exception as exc:
            log.warning("[zed] YOLO unavailable — %s", exc)

        runtime_params   = sl.RuntimeParameters()
        last_infer       = 0.0
        last_display     = 0.0
        _display_interval = 1.0 / 30.0
        retry_sleep      = 1.0

        while True:
            err = zed.grab(runtime_params)
            if err == sl.ERROR_CODE.SUCCESS:
                zed.retrieve_image(image_mat, sl.VIEW.LEFT)
                # get_data() returns BGRA (4-channel); drop alpha
                left = image_mat.get_data()[:, :, :3]

                Handler._zed_connected  = True
                Handler._zed_last_error = None
                retry_sleep = 1.0

                now_disp = time.monotonic()
                if now_disp - last_display >= _display_interval:
                    last_display = now_disp
                    ok, enc = cv2.imencode('.jpg', left, [cv2.IMWRITE_JPEG_QUALITY, 75])
                    if ok:
                        with Handler._zed_lock:
                            Handler._zed_jpeg            = enc.tobytes()
                            Handler._zed_frame_count     += 1
                            Handler._zed_last_frame_time = time.monotonic()

                if yolo is None:
                    continue
                now = time.monotonic()
                if now - last_infer < _min_infer_interval:
                    continue
                last_infer = now

                try:
                    results  = yolo(left, classes=[YOLO_PERSON_CLASS],
                                    conf=YOLO_CONFIDENCE, verbose=False)
                    annotated = left.copy()
                    detections = []
                    for result in results:
                        for box in result.boxes:
                            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                            conf = float(box.conf[0])
                            _draw_box(annotated, x1, y1, x2, y2, f'person {conf:.0%}')
                            detections.append({
                                'label':      'person',
                                'confidence': round(conf, 3),
                                'distance_m': None,
                                'bbox':       [x1, y1, x2, y2],
                            })
                    ok2, enc2 = cv2.imencode('.jpg', annotated, [cv2.IMWRITE_JPEG_QUALITY, 80])
                    if ok2:
                        with Handler._det_lock:
                            Handler._det_jpeg = enc2.tobytes()
                    payload = json.dumps({
                        'ts':         time.time(),
                        'count':      len(detections),
                        'detections': detections,
                    })
                    try:
                        Path(DETECTIONS_FILE).write_text(payload)
                    except Exception:
                        pass
                except Exception as exc:
                    log.error("[zed] YOLO error: %s", exc)

            elif err == sl.ERROR_CODE.END_OF_SVOFILE_REACHED:
                zed.set_svo_position(0)
            else:
                Handler._zed_connected  = False
                Handler._zed_last_error = f"grab error: {err}"
                time.sleep(retry_sleep)
                retry_sleep = min(retry_sleep * 2, 5.0)

    # ── OpenCV fallback (grayscale — YUYV color bug on Jetson pip OpenCV) ─────
    def _capture_loop_opencv():
        yolo = None
        try:
            from ultralytics import YOLO
            yolo = YOLO(YOLO_MODEL)
            yolo(np.zeros((480, 640, 3), dtype=np.uint8),
                 classes=[YOLO_PERSON_CLASS], verbose=False)
            log.info("[zed] YOLO model '%s' ready (OpenCV fallback)", YOLO_MODEL)
        except Exception as exc:
            log.warning("[zed] YOLO unavailable — %s", exc)

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

        cap = None
        last_infer    = 0.0
        last_display  = 0.0
        _display_interval = 1.0 / 30.0
        retry_sleep   = 1.0

        while True:
            if cap is None or not cap.isOpened():
                with _quiet_stderr():
                    cap = cv2.VideoCapture(_device_index())
                if cap.isOpened():
                    Handler._zed_connected  = True
                    Handler._zed_last_error = None
                    retry_sleep = 1.0
                    log.info("[zed] Opened %s via OpenCV (grayscale)", ZED_DEVICE)
                else:
                    Handler._zed_connected  = False
                    Handler._zed_last_error = f"{ZED_DEVICE} not available"
                    time.sleep(retry_sleep)
                    retry_sleep = min(retry_sleep * 2, 5.0)
                    continue

            with _quiet_stderr():
                ret, frame = cap.read()
            if not ret:
                Handler._zed_connected  = False
                Handler._zed_last_error = "Frame read failed — reconnecting"
                cap.release()
                cap = None
                time.sleep(retry_sleep)
                retry_sleep = min(retry_sleep * 2, 5.0)
                continue

            h, w = frame.shape[:2]
            left = _fix_yuyv(frame[:, :w // 2])

            now_disp = time.monotonic()
            if now_disp - last_display >= _display_interval:
                last_display = now_disp
                ok, enc = cv2.imencode('.jpg', left, [cv2.IMWRITE_JPEG_QUALITY, 75])
                if ok:
                    with Handler._zed_lock:
                        Handler._zed_jpeg            = enc.tobytes()
                        Handler._zed_frame_count     += 1
                        Handler._zed_last_frame_time = time.monotonic()

            if yolo is None:
                continue
            now = time.monotonic()
            if now - last_infer < _min_infer_interval:
                continue
            last_infer = now

            try:
                results   = yolo(left, classes=[YOLO_PERSON_CLASS],
                                 conf=YOLO_CONFIDENCE, verbose=False)
                annotated = left.copy()
                detections = []
                for result in results:
                    for box in result.boxes:
                        x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                        conf = float(box.conf[0])
                        _draw_box(annotated, x1, y1, x2, y2, f'person {conf:.0%}')
                        detections.append({
                            'label':      'person',
                            'confidence': round(conf, 3),
                            'distance_m': None,
                            'bbox':       [x1, y1, x2, y2],
                        })
                ok2, enc2 = cv2.imencode('.jpg', annotated, [cv2.IMWRITE_JPEG_QUALITY, 80])
                if ok2:
                    with Handler._det_lock:
                        Handler._det_jpeg = enc2.tobytes()
                payload = json.dumps({
                    'ts':         time.time(),
                    'count':      len(detections),
                    'detections': detections,
                })
                try:
                    Path(DETECTIONS_FILE).write_text(payload)
                except Exception:
                    pass
            except Exception as exc:
                log.error("[zed] YOLO error: %s", exc)

    # ── Try pyzed SDK first; fall back to OpenCV ───────────────────────────────
    def _start():
        try:
            # libusb 1.0.25 (Ubuntu 22.04) doesn't auto-init the default context;
            # ZED SDK calls libusb_get_device_list(NULL) which segfaults without it.
            import ctypes
            ctypes.CDLL('/lib/aarch64-linux-gnu/libusb-1.0.so.0').libusb_init(None)

            import pyzed.sl as sl
            zed        = sl.Camera()
            init_params = sl.InitParameters()
            init_params.camera_resolution = sl.RESOLUTION.HD720
            init_params.camera_fps        = 30
            init_params.depth_mode        = sl.DEPTH_MODE.NONE

            err = zed.open(init_params)
            if err != sl.ERROR_CODE.SUCCESS:
                raise RuntimeError(f"ZED SDK open failed: {err}")

            image_mat = sl.Mat()
            log.info("[zed] ZED SDK opened — color feed active")
            _capture_loop_sdk(zed, image_mat)

        except ImportError:
            log.info("[zed] pyzed not found — using OpenCV grayscale fallback")
            _capture_loop_opencv()
        except Exception as exc:
            log.warning("[zed] SDK error (%s) — falling back to OpenCV", exc)
            _capture_loop_opencv()

    threading.Thread(target=_start, daemon=True, name="zed-capture").start()
    log.info("[zed] Capture thread started (%s)", ZED_DEVICE)


def main():
    ap = argparse.ArgumentParser(description="Dual-robot dashboard server (agrobot | jackal)")
    ap.add_argument("--port", type=int, default=8766)
    ap.add_argument("--gnss", default=GNSS_FILE_DEFAULT)
    ap.add_argument("--chassis", default=None,
                    help="agrobot | jackal — overrides config/active_chassis.yaml")
    ap.add_argument("--rear-camera", dest="rear_camera", default=None,
                    help="realsense | webcam — overrides the chassis rear_camera setting")
    args = ap.parse_args()

    Handler.gnss_file = args.gnss

    # Resolve the active chassis: --chassis > $ROBOT_CHASSIS > active_chassis.yaml.
    CHASSIS = chassis.load_active(args.chassis)
    Handler.chassis = CHASSIS
    # Resolve the rear-camera source: --rear-camera > $REAR_CAMERA > chassis yaml.
    rear_src = chassis.resolve_rear_camera(args.rear_camera, CHASSIS)
    CHASSIS.rear_camera = rear_src   # so GET /api/config reflects the effective source
    log.info("[chassis] active: %s — %s (comms=%s, rear_camera=%s)",
             CHASSIS.name, CHASSIS.description, CHASSIS.comms, rear_src)

    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.executors import SingleThreadedExecutor
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

        def _vel_publish_cb():
            with Handler._vel_lock:
                if not Handler._vel_active:
                    return
                if time.monotonic() - Handler._vel_last > Handler.VEL_TIMEOUT:
                    Handler._vel_active = False
                    lx, az = 0.0, 0.0
                else:
                    lx = Handler._vel_lin
                    az = Handler._vel_ang
                    if lx == 0.0 and az == 0.0:
                        Handler._vel_active = False

            publish_velocity(lx, az)

        _node.create_timer(0.05, _vel_publish_cb)  # 20 Hz

        # Camera subscriber — topic depends on the active chassis.
        _cam_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        _encode_fn = _build_encode_fn()
        # Skip the ROS camera topic when a local webcam is the rear source, so the
        # two feeds don't fight over the same buffer.
        if _encode_fn and CHASSIS.camera_topic and rear_src != "webcam":
            def _on_image(msg: RosImage):
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

            _node.create_subscription(RosImage, CHASSIS.camera_topic,
                                      _on_image, _cam_qos)
            log.info("[camera] Subscribed to %s", CHASSIS.camera_topic)
        elif not _encode_fn:
            log.warning("[camera] neither cv2 nor PIL found — /api/camera will return 503")
        elif rear_src == "webcam":
            log.info("[camera] rear_camera=webcam — local capture only, ROS camera topic skipped")
        else:
            log.info("[camera] no ROS camera_topic for this chassis — using local capture only")

        _executor = SingleThreadedExecutor()
        _executor.add_node(_node)

        def _ros_spin():
            try:
                _executor.spin()
            except Exception:
                pass  # ExternalShutdownException on clean exit — not an error

        threading.Thread(target=_ros_spin, daemon=True).start()

    except Exception as exc:
        log.warning("[rclpy] ROS init failed — teleop will not work. Reason: %s", exc)

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
