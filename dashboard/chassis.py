#!/usr/bin/env python3
"""
Chassis abstraction for the dual-robot teleoperation dashboard.

The dashboard server (serve.py) and the web UI are robot-agnostic. This module
is the single place that knows *which* robot is connected and how to talk to it:

  - agrobot   — differential-drive robot, Modbus RTU over serial. The dashboard
             publishes Int16MultiArray[left, right] on a speed topic that the
             separate robot_base_node turns into Modbus writes.
  - jackal — Clearpath Jackal, ROS 2 over LAN. The dashboard publishes
             geometry_msgs/Twist straight to the Jackal's cmd_vel topic.

Per-chassis parameters live in  config/chassis/<name>.yaml.
The active chassis is resolved (highest priority first) from:

  1. an explicit name passed to load_active()  (serve.py --chassis ...)
  2. the ROBOT_CHASSIS environment variable
  3. the `chassis:` field in config/active_chassis.yaml
  4. the fallback default ("agrobot")

The Chassis object itself is pure-Python and import-safe without ROS, so it can
be unit-tested anywhere. ROS message types are imported lazily inside setup_ros().
"""
import os
from pathlib import Path

REPO_ROOT       = Path(__file__).resolve().parent.parent
CONFIG_DIR      = REPO_ROOT / "config"
ACTIVE_FILE     = CONFIG_DIR / "active_chassis.yaml"
CHASSIS_DIR     = CONFIG_DIR / "chassis"
DEFAULT_CHASSIS = "agrobot"


# ---------------------------------------------------------------------------
# YAML loading (PyYAML when available; tiny fallback parser otherwise)
# ---------------------------------------------------------------------------
def _load_yaml(path):
    """Parse a YAML file. Prefer PyYAML; fall back to a minimal parser that
    understands the simple key/value + one-level-nested structure used by our
    chassis config files, so the dashboard and tests work even on a bare Python
    install without PyYAML."""
    text = Path(path).read_text(encoding="utf-8")
    try:
        import yaml  # type: ignore
        return yaml.safe_load(text) or {}
    except ImportError:
        return _minimal_yaml(text)


def _coerce(token):
    t = token.strip()
    if len(t) >= 2 and t[0] in "\"'" and t[-1] == t[0]:
        return t[1:-1]
    low = t.lower()
    if low in ("true", "yes"):
        return True
    if low in ("false", "no"):
        return False
    if low in ("null", "~", ""):
        return None
    try:
        return int(t)
    except ValueError:
        pass
    try:
        return float(t)
    except ValueError:
        pass
    return t


def _minimal_yaml(text):
    """Parse the limited YAML subset our config files use: comments, scalar
    key/value pairs, and one level of nested mapping (e.g. `features:`)."""
    root = {}
    stack = [(-1, root)]   # (indent, container)
    for raw in text.splitlines():
        line = raw.split("#", 1)[0].rstrip()   # our configs never use '#' in values
        if not line.strip():
            continue
        indent = len(line) - len(line.lstrip(" "))
        key, sep, val = line.strip().partition(":")
        if not sep:
            continue
        key = key.strip()
        val = val.strip()
        while stack and indent <= stack[-1][0]:
            stack.pop()
        parent = stack[-1][1]
        if val == "":
            child = {}
            parent[key] = child
            stack.append((indent, child))
        else:
            parent[key] = _coerce(val)
    return root


# ---------------------------------------------------------------------------
# Chassis driver
# ---------------------------------------------------------------------------
class Chassis:
    """Holds the active chassis configuration and builds its ROS velocity
    publisher / feedback subscriptions on demand."""

    def __init__(self, name, cfg):
        self.name           = name
        self.cfg            = cfg or {}
        self.description    = self.cfg.get("description", name)
        self.comms          = self.cfg.get("comms", "modbus_speed")

        # Velocity ceilings (m/s, rad/s) enforced by the dashboard.
        self.max_linear     = float(self.cfg.get("max_linear", 15.0))
        self.max_angular    = float(self.cfg.get("max_angular", 15.0))

        # Twist -> wheel-speed scaling (modbus_speed only).
        self.linear_scale   = float(self.cfg.get("linear_scale", 3000))
        self.angular_scale  = float(self.cfg.get("angular_scale", 1000))
        self.speed_max      = int(self.cfg.get("speed_max", 32767))
        self.pulse_per_m    = float(self.cfg.get("pulse_per_m", 3211.0))

        # ROS topics
        self.speed_cmd_topic  = self.cfg.get("speed_cmd_topic")
        self.wheel_odom_topic = self.cfg.get("wheel_odom_topic")   # may be None
        self.battery_topic    = self.cfg.get("battery_topic")      # may be None
        self.camera_topic     = self.cfg.get("camera_topic")

        # Chassis battery gauge range (V). The pack voltage is mapped to 0-100%
        # over [battery_min_v, battery_max_v]; defaults model a ~48 V (14S) pack.
        self.battery_min_v    = float(self.cfg.get("battery_min_v", 42.0))
        self.battery_max_v    = float(self.cfg.get("battery_max_v", 58.0))

        # Rear camera source: 'realsense' (D435 color) | 'webcam' (generic USB UVC,
        # e.g. a Logitech webcam). rear_camera_device optionally pins the V4L2 device
        # path or index; auto-detected when omitted.
        self.rear_camera        = (self.cfg.get("rear_camera") or "realsense")
        self.rear_camera_device = self.cfg.get("rear_camera_device")

        # Feature flags consumed by the UI and server.
        self.features         = dict(self.cfg.get("features") or {})

        # Network (jackal LAN)
        self.host_iface     = self.cfg.get("host_iface")
        self.host_ip        = self.cfg.get("host_ip")
        self.robot_ip       = self.cfg.get("robot_ip")
        self.ros_domain_id  = self.cfg.get("ros_domain_id")

        # Modbus (agrobot)
        self.serial_port     = self.cfg.get("serial_port")
        self.baud            = self.cfg.get("baud")
        self.slave_id        = self.cfg.get("slave_id")
        self.chassis_variant = self.cfg.get("chassis_variant")

        # PLC (agrobot tree-planter only). The dashboard sends auger / planter / robot-arm
        # commands DIRECTLY over Modbus TCP to the PLC at plc_host:plc_port (502); disabled
        # chassis (jackal) simply leave plc_enabled false and the PLC endpoints/UI stay hidden.
        plc = dict(self.cfg.get("plc") or {})
        self.plc_enabled = bool(plc.get("enabled", False))
        self.plc_host    = plc.get("host", "127.0.0.1")
        self.plc_port    = int(plc.get("port", 502))

    # -- helpers --------------------------------------------------------------
    def has_feature(self, key):
        return bool(self.features.get(key, False))

    def twist_to_wheel_speeds(self, linear_x, angular_z):
        """Convert Twist (m/s, rad/s) to [left, right] wheel speed integers.

        Differential-drive convention (matches teleop_keyboard.py):
          forward  W: [+,+]   backward S: [-,-]
          left     A: [-,+]   right    D: [+,-]   (positive angular_z = left)
        """
        lin = int(linear_x  * self.linear_scale)
        ang = int(angular_z * self.angular_scale)
        left  = max(-self.speed_max, min(self.speed_max, lin - ang))
        right = max(-self.speed_max, min(self.speed_max, lin + ang))
        return left, right

    def to_browser_config(self):
        """Payload for GET /api/config — lets the web UI adapt to the chassis."""
        return {
            "chassis":     self.name,
            "description": self.description,
            "comms":       self.comms,
            "rear_camera": self.rear_camera,
            "features":    {**self.features, "plc": self.plc_enabled},
            "battery":     {"minV": self.battery_min_v,
                            "maxV": self.battery_max_v},
            # Expose PLC as a derived feature flag so the UI's data-chassis-feature
            # hide mechanism gates the PLC panels automatically (true on agrobot only).
            # `actuators` stays separate — jackal keeps its cosmetic planter buttons.
            "plc":         {"enabled": self.plc_enabled},
            "limits":      {"maxLinear": self.max_linear,
                            "maxAngular": self.max_angular},
            "scaling":     {"linearScale": self.linear_scale,
                            "angularScale": self.angular_scale,
                            "speedMax": self.speed_max,
                            "pulsePerM": self.pulse_per_m},
        }

    # -- ROS wiring (lazy imports; only called from serve.main()) -------------
    def setup_ros(self, node, handler):
        """Create the velocity publisher (and optional wheel-odom subscription)
        on the given rclpy `node`, store the publisher on `handler.speed_cmd_pub`,
        and return a publish_velocity(linear_x, angular_z) callable for the
        velocity timer in serve.py.
        """
        import time

        if self.comms == "ros_twist":
            from geometry_msgs.msg import Twist
            pub = node.create_publisher(Twist, self.speed_cmd_topic, 10)

            def publish_velocity(linear_x, angular_z):
                msg = Twist()
                msg.linear.x  = float(linear_x)
                msg.angular.z = float(angular_z)
                pub.publish(msg)

        else:  # modbus_speed
            from std_msgs.msg import Int16MultiArray
            pub = node.create_publisher(Int16MultiArray, self.speed_cmd_topic, 10)

            def publish_velocity(linear_x, angular_z):
                left, right = self.twist_to_wheel_speeds(linear_x, angular_z)
                msg = Int16MultiArray()
                msg.data = [left, right]
                pub.publish(msg)

        handler.speed_cmd_pub = pub

        # Optional wheel-odometry feedback (agrobot only).
        if self.wheel_odom_topic:
            from std_msgs.msg import Int32MultiArray

            # Max plausible encoder delta per cycle (~10 Hz). Robot top speed ~1 m/s
            # ≈ 3211 pulses/m → ~321 pulses/cycle; 3000 gives a comfortable 9× margin.
            MAX_ODOM_DELTA = 3000

            def _on_wheel_odom(msg):
                if len(msg.data) >= 2:
                    l, r = int(msg.data[0]), int(msg.data[1])
                    now = time.monotonic()
                    with handler._odom_lock:
                        prev_last = handler._odom_last
                        # Reconnect: if the chassis was offline >5 s, treat as a fresh
                        # session and reset accumulated mileage to start from 0.
                        if prev_last > 0 and (now - prev_last) > 5.0:
                            handler._odom_mileage = 0.0
                            handler._odom_prev_l  = None
                            handler._odom_prev_r  = None
                        if handler._odom_prev_l is not None:
                            dl = l - handler._odom_prev_l
                            dr = r - handler._odom_prev_r
                            # Skip outlier deltas (bad Modbus read / register jump).
                            if abs(dl) < MAX_ODOM_DELTA and abs(dr) < MAX_ODOM_DELTA:
                                handler._odom_mileage += abs((dl + dr) / 2.0)
                        handler._odom_prev_l = l
                        handler._odom_prev_r = r
                        handler._odom_l    = l
                        handler._odom_r    = r
                        handler._odom_last = now

            node.create_subscription(Int32MultiArray, self.wheel_odom_topic,
                                     _on_wheel_odom, 10)

        # Optional chassis battery voltage (agrobot only — gated by the `battery`
        # feature). Raw Float32 readings are accumulated in a 15 s window, outliers
        # (≤0 V, or outside 30-70 V) are discarded, and the median is recomputed
        # every 10 s so the gauge is steady. The HTTP handler reads the smoothed value.
        if self.has_feature("battery") and self.battery_topic:
            from std_msgs.msg import Float32

            def _on_chassis_battery(msg):
                now = time.monotonic()
                with handler._chassis_batt_lock:
                    handler._chassis_batt_last = now
                    if msg.data > 0:
                        handler._chassis_batt_window.append((now, msg.data))
                    cutoff = now - 15.0
                    handler._chassis_batt_window = [
                        (t, v) for t, v in handler._chassis_batt_window if t > cutoff
                    ]
                    if now - handler._chassis_batt_smoothed_at >= 10.0:
                        valid = sorted(v for _, v in handler._chassis_batt_window
                                       if 30.0 < v < 70.0)
                        if valid:
                            handler._chassis_batt_smoothed = valid[len(valid) // 2]
                            handler._chassis_batt_smoothed_at = now

            node.create_subscription(Float32, self.battery_topic,
                                     _on_chassis_battery, 10)

        return publish_velocity


# ---------------------------------------------------------------------------
# Resolution / loading
# ---------------------------------------------------------------------------
def resolve_name(cli_name=None):
    """Return the active chassis name using the documented priority order."""
    if cli_name:
        return str(cli_name).strip()
    env = os.environ.get("ROBOT_CHASSIS")
    if env:
        return env.strip()
    if ACTIVE_FILE.exists():
        try:
            data = _load_yaml(ACTIVE_FILE) or {}
            name = data.get("chassis")
            if name:
                return str(name).strip()
        except Exception:
            pass
    return DEFAULT_CHASSIS


def resolve_rear_camera(cli_value=None, chassis=None):
    """Resolve the rear-camera source, priority:
    --rear-camera flag  >  $REAR_CAMERA  >  chassis yaml  >  'realsense'.

    Valid sources: 'realsense' | 'webcam' | 'none' (rear view disabled).
    'off'/'disabled' are accepted as aliases for 'none'."""
    val = (cli_value
           or os.environ.get("REAR_CAMERA")
           or (chassis.rear_camera if chassis else None)
           or "realsense")
    val = str(val).strip().lower()
    if val in ("none", "off", "disabled", "disable", "no"):
        return "none"
    return val if val in ("realsense", "webcam") else "realsense"


def load(name):
    """Load config/chassis/<name>.yaml and return a Chassis."""
    path = CHASSIS_DIR / f"{name}.yaml"
    if not path.exists():
        available = sorted(p.stem for p in CHASSIS_DIR.glob("*.yaml")) \
            if CHASSIS_DIR.exists() else []
        raise FileNotFoundError(
            f"No chassis config for '{name}' (expected {path}). "
            f"Available: {', '.join(available) or 'none'}")
    return Chassis(name, _load_yaml(path))


def load_active(cli_name=None):
    """Resolve the active chassis name and load its Chassis config."""
    return load(resolve_name(cli_name))
