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
import sys
from pathlib import Path

REPO_ROOT       = Path(__file__).resolve().parent.parent
CONFIG_DIR      = REPO_ROOT / "config"
ACTIVE_FILE     = CONFIG_DIR / "active_chassis.yaml"
CHASSIS_DIR     = CONFIG_DIR / "chassis"
DEFAULT_CHASSIS = "agrobot"

# Transitional (until this module moves into the agrobot_dashboard package):
# make the repo root importable so `agrobot_dashboard` resolves when running
# straight from a checkout without an installed package.
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import yaml  # PyYAML is a hard dependency (pyproject.toml); a hand-rolled
             # fallback parser used to live here and silently mis-parsed
             # anything beyond the exact shape of our config files.

from agrobot_dashboard.domain import kinematics


def _load_yaml(path):
    return yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}


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

        # Rear camera source: 'zed' (ZED 2i via pyzed SDK) | 'webcam' (generic USB UVC,
        # e.g. a Logitech webcam). rear_camera_device optionally pins the V4L2 device
        # path or index; auto-detected when omitted.
        self.rear_camera        = (self.cfg.get("rear_camera") or "zed")
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
        """Convert Twist (m/s, rad/s) to (left, right) wheel speed integers
        using this chassis's configured scaling."""
        return kinematics.twist_to_wheel_speeds(
            linear_x, angular_z,
            self.linear_scale, self.angular_scale, self.speed_max)

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

        # Optional wheel-odometry feedback (agrobot only). The accumulation rules
        # (outlier rejection, reconnect reset) live in domain.odometry, owned
        # by the TelemetryStore — this callback only feeds it.
        if self.wheel_odom_topic:
            from std_msgs.msg import Int32MultiArray

            def _on_wheel_odom(msg):
                if len(msg.data) >= 2:
                    handler.telemetry.odom.update(int(msg.data[0]), int(msg.data[1]))

            node.create_subscription(Int32MultiArray, self.wheel_odom_topic,
                                     _on_wheel_odom, 10)

        # Optional chassis battery voltage (agrobot only — gated by the `battery`
        # feature). Median smoothing lives in domain.battery via the store.
        if self.has_feature("battery") and self.battery_topic:
            from std_msgs.msg import Float32

            def _on_chassis_battery(msg):
                handler.telemetry.battery.add_reading(float(msg.data))

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
    --rear-camera flag  >  $REAR_CAMERA  >  chassis yaml  >  'zed'.

    Valid sources: 'zed' | 'realsense' | 'webcam' | 'none' (rear view disabled).
    'off'/'disabled' are accepted as aliases for 'none'."""
    val = (cli_value
           or os.environ.get("REAR_CAMERA")
           or (chassis.rear_camera if chassis else None)
           or "zed")
    val = str(val).strip().lower()
    if val in ("none", "off", "disabled", "disable", "no"):
        return "none"
    return val if val in ("zed", "realsense", "webcam") else "zed"


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
