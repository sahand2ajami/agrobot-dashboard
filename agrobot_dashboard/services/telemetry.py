"""TelemetryStore — the single, thread-safe home for the dashboard's runtime
state.

This replaces the old pattern of ~40 mutable class attributes on the HTTP
Handler ("the blackboard") that capture threads, ROS callbacks and request
threads all wrote to directly. Each subsystem gets one small state object
with its own lock; everything that used to be `Handler._cam_*`, `_zed_*`,
`_det_*`, `_vel_*`, `_odom_*`, `_chassis_batt_*`, `_rec_*`, `_settings*` or
`_plc_last_connected` now lives here.

Locking convention: callers take `state.lock` themselves when they need a
multi-field transaction (e.g. the fwd2m control loop reading and writing the
velocity command atomically); the provided helper methods are for the common
single-shot updates.
"""
import threading
import time

from agrobot_dashboard.domain.battery import MedianVoltageFilter
from agrobot_dashboard.domain.odometry import OdometryAccumulator


class FeedState:
    """One camera feed: latest JPEG (+ optional raw frame for detection
    compositing), frame counter, and connectivity flags."""

    def __init__(self):
        self.lock = threading.Lock()
        self.jpeg = None
        self.frame = None            # stream-res BGR numpy frame, or None
        self.frame_count = 0
        self.connected = False
        self.last_error = None
        self.last_frame_time = 0.0   # monotonic; 0 = no frame ever received

    def set_frame(self, jpeg, frame=None):
        """Store a freshly encoded frame; implies the feed is healthy."""
        with self.lock:
            self.jpeg = jpeg
            self.frame = frame
            self.frame_count += 1
            self.connected = True
            self.last_error = None
            self.last_frame_time = time.monotonic()
            return self.frame_count

    def set_status(self, connected, error=None):
        with self.lock:
            self.connected = connected
            self.last_error = error

    def get_jpeg(self):
        with self.lock:
            return self.jpeg

    def snapshot(self):
        with self.lock:
            return {
                "connected":       self.connected,
                "has_frame":       self.jpeg is not None,
                "frames_received": self.frame_count,
                "last_error":      self.last_error,
            }, self.last_frame_time


class DetectionState:
    """YOLO results for one feed + the on-demand gate: inference runs only
    while a client has requested detections within `idle_timeout` seconds."""

    def __init__(self, idle_timeout=3.0):
        self.lock = threading.Lock()
        self.boxes = []              # [(x1,y1,x2,y2,label), ...] at stream-res
        self.payload = None          # last detection JSON dict
        self.jpeg = None             # legacy fallback frame for /api/detection
        self.idle_timeout = idle_timeout
        self._last_request = 0.0     # monotonic; guarded by lock

    def mark_wanted(self):
        with self.lock:
            self._last_request = time.monotonic()

    def wanted(self):
        with self.lock:
            return (time.monotonic() - self._last_request) < self.idle_timeout

    def set_result(self, boxes, payload):
        with self.lock:
            self.boxes = boxes
            self.payload = payload

    def get_boxes(self):
        with self.lock:
            return list(self.boxes)

    def get_payload(self):
        with self.lock:
            return self.payload


class VelocityState:
    """The commanded velocity, shared between HTTP request threads, the fwd2m
    control loop and the 20 Hz publish thread. Fields are public and callers
    take `.lock` for multi-field transactions — the deadman/override logic
    reads and writes several fields atomically."""

    def __init__(self, timeout=0.5):
        self.lock = threading.Lock()
        self.lin = 0.0
        self.ang = 0.0
        self.active = False
        self.last = 0.0        # monotonic time of the last command
        self.timeout = timeout  # deadman: stop after this much silence (s)

    def set_command(self, lin, ang):
        with self.lock:
            self.lin = lin
            self.ang = ang
            self.active = True
            self.last = time.monotonic()

    def consume_for_publish(self):
        """One 20 Hz tick: returns (lin, ang) to publish, or None to stay
        silent. Silence past the deadman timeout publishes a single stop,
        then idles (active=False) until the next command."""
        with self.lock:
            if not self.active:
                return None
            if time.monotonic() - self.last > self.timeout:
                self.active = False
                return (0.0, 0.0)
            if self.lin == 0.0 and self.ang == 0.0:
                self.active = False
            return (self.lin, self.ang)


class OdometryState:
    """Wheel-encoder state: the domain accumulator plus its lock."""

    def __init__(self):
        self.lock = threading.Lock()
        self.acc = OdometryAccumulator(max_delta=3000, reconnect_gap_s=5.0)

    def update(self, left, right):
        with self.lock:
            self.acc.update(left, right, time.monotonic())

    def snapshot(self):
        with self.lock:
            return self.acc.left, self.acc.right, self.acc.last_update, \
                   self.acc.mileage_pulses


class BatteryState:
    """Chassis pack voltage: the domain median filter plus its lock."""

    def __init__(self):
        self.lock = threading.Lock()
        self.filter = MedianVoltageFilter()

    def add_reading(self, volts):
        with self.lock:
            self.filter.add(volts, time.monotonic())

    def snapshot(self):
        with self.lock:
            return self.filter.smoothed, self.filter.last_reading


class RecordingState:
    def __init__(self):
        self.lock = threading.Lock()
        self.active = False
        self.dir = None
        self.ts = None


class SettingsStore:
    def __init__(self, defaults):
        self.lock = threading.Lock()
        self.data = dict(defaults)

    def get(self, key, default=None):
        with self.lock:
            return self.data.get(key, default)

    def as_dict(self):
        with self.lock:
            return dict(self.data)

    def update(self, updates):
        with self.lock:
            self.data.update(updates)
            return dict(self.data)


class PlcLinkState:
    """Tracks PLC connect/disconnect transitions so they log exactly once."""

    def __init__(self):
        self.lock = threading.Lock()
        self.last_connected = None   # None = unknown, True = up, False = down

    def transition(self, connected):
        """Record the new state; returns True if this is a state CHANGE."""
        with self.lock:
            if self.last_connected == connected:
                return False
            self.last_connected = connected
            return True


class TelemetryStore:
    def __init__(self, settings_defaults=None):
        self.rear_cam  = FeedState()
        self.front_zed = FeedState()
        self.front_det = DetectionState()
        self.rear_det  = DetectionState()
        self.vel       = VelocityState()
        self.odom      = OdometryState()
        self.battery   = BatteryState()
        self.recording = RecordingState()
        self.plc_link  = PlcLinkState()
        self.settings  = SettingsStore(settings_defaults or {})
