"""
Tests for the dual-chassis abstraction (dashboard/chassis.py) and the
chassis-aware behaviour of dashboard/serve.py.

Runs without ROS or hardware:
  - chassis config loading (agrobot | jackal), feature flags, limits, topics
  - the minimal YAML fallback parser
  - chassis selection precedence (cli > env > active_chassis.yaml > default)
  - Twist -> wheel-speed conversion parity with serve.twist_to_wheel_speeds
  - GET /api/config reflects the active chassis
  - POST /api/cmd_vel enforces per-chassis velocity limits (jackal 3.0 vs agrobot 15.0)
"""
import json
import os
import socket
import sys
import threading
import time
import urllib.error
import urllib.request
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parent.parent / "dashboard"))
import chassis    # noqa: E402
import serve      # noqa: E402


# ── chassis.py: config loading ──────────────────────────────────────────────

class TestChassisLoading:
    def test_agrobot_loads(self):
        c = chassis.load("agrobot")
        assert c.name == "agrobot"
        assert c.comms == "modbus_speed"
        assert c.max_linear == 15.0
        assert c.speed_cmd_topic == "/avatar_robot/speed_cmd"
        assert c.wheel_odom_topic == "/avatar_robot/wheel_odom"
        assert c.camera_topic == "/camera/camera/color/image_raw"
        assert c.battery_topic == "/avatar_robot/battery"
        assert c.battery_min_v == 42.0
        assert c.battery_max_v == 58.0
        assert c.has_feature("battery") is True
        assert c.has_feature("fwd2m") is True
        assert c.has_feature("modbus_slider") is True

    def test_jackal_loads(self):
        c = chassis.load("jackal")
        assert c.name == "jackal"
        assert c.comms == "ros_twist"
        assert c.max_linear == 3.0
        assert c.max_angular == 3.0
        assert c.speed_cmd_topic == "/jackal1/cmd_vel"
        assert c.wheel_odom_topic is None          # no wheel odom on jackal
        assert c.battery_topic is None             # no chassis battery on jackal
        assert c.has_feature("battery") is False
        assert c.has_feature("fwd2m") is False
        assert c.has_feature("wheel_odom") is False
        assert c.has_feature("modbus_slider") is False

    def test_unknown_chassis_raises(self):
        with pytest.raises(FileNotFoundError):
            chassis.load("does-not-exist")

    def test_to_browser_config_shape(self):
        cfg = chassis.load("jackal").to_browser_config()
        assert cfg["chassis"] == "jackal"
        assert cfg["comms"] == "ros_twist"
        assert cfg["limits"]["maxLinear"] == 3.0
        assert cfg["features"]["fwd2m"] is False


# ── chassis.py: twist conversion parity ─────────────────────────────────────

class TestTwistParity:
    def test_agrobot_matches_serve(self):
        c = chassis.load("agrobot")
        for lx, az in [(0.0, 0.0), (0.5, 0.0), (-0.5, 0.0),
                       (0.0, 1.0), (0.3, 0.1), (999.0, 0.0)]:
            assert c.twist_to_wheel_speeds(lx, az) == \
                   serve.twist_to_wheel_speeds(lx, az)

    def test_outputs_are_ints(self):
        l, r = chassis.load("agrobot").twist_to_wheel_speeds(0.3, 0.1)
        assert isinstance(l, int) and isinstance(r, int)


# ── chassis.py: selection precedence + minimal parser ───────────────────────

class TestResolution:
    def test_default_from_active_file(self):
        # repo ships config/active_chassis.yaml -> agrobot (no cli/env override)
        os.environ.pop("ROBOT_CHASSIS", None)
        assert chassis.resolve_name() == "agrobot"

    def test_cli_overrides_env(self, monkeypatch):
        monkeypatch.setenv("ROBOT_CHASSIS", "jackal")
        assert chassis.resolve_name("agrobot") == "agrobot"

    def test_env_overrides_file(self, monkeypatch):
        monkeypatch.setenv("ROBOT_CHASSIS", "jackal")
        assert chassis.resolve_name() == "jackal"

    def test_minimal_yaml_parser(self):
        parsed = chassis._minimal_yaml(
            "name: x\ncomms: ros_twist\nmax_linear: 3.0\n"
            "features:\n  battery: false\n  fwd2m: true\n")
        assert parsed["name"] == "x"
        assert parsed["max_linear"] == 3.0
        assert parsed["features"] == {"battery": False, "fwd2m": True}


# ── rear-camera source (realsense | webcam) ─────────────────────────────────

class TestRearCamera:
    def test_default_is_realsense(self):
        assert chassis.load("agrobot").rear_camera == "realsense"
        assert chassis.load("jackal").rear_camera == "realsense"

    def test_cli_override(self):
        c = chassis.load("agrobot")
        assert chassis.resolve_rear_camera("webcam", c) == "webcam"

    def test_env_override(self, monkeypatch):
        monkeypatch.setenv("REAR_CAMERA", "webcam")
        assert chassis.resolve_rear_camera(None, chassis.load("agrobot")) == "webcam"

    def test_default_from_chassis(self, monkeypatch):
        monkeypatch.delenv("REAR_CAMERA", raising=False)
        assert chassis.resolve_rear_camera(None, chassis.load("agrobot")) == "realsense"

    def test_invalid_value_falls_back(self, monkeypatch):
        monkeypatch.delenv("REAR_CAMERA", raising=False)
        assert chassis.resolve_rear_camera("banana", chassis.load("agrobot")) == "realsense"


# ── serve.py: chassis-aware HTTP behaviour ──────────────────────────────────

def _free_port() -> int:
    with socket.socket() as s:
        s.bind(("", 0))
        return s.getsockname()[1]


@pytest.fixture(scope="module")
def server():
    """In-process serve.py server. Tests set serve.Handler.chassis per case;
    the fixture restores it to None on teardown so other test modules are
    unaffected by the shared class attribute."""
    port = _free_port()
    serve.Handler.speed_cmd_pub = None
    httpd = serve._Server(("127.0.0.1", port), serve.Handler)
    t = threading.Thread(target=httpd.serve_forever, daemon=True)
    t.start()
    time.sleep(0.05)
    yield port
    httpd.shutdown()
    serve.Handler.chassis = None
    serve.Handler.speed_cmd_pub = None


def _get(port, path):
    try:
        with urllib.request.urlopen(f"http://127.0.0.1:{port}{path}", timeout=3) as r:
            return r.status, json.loads(r.read())
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read())


def _post(port, path, body):
    req = urllib.request.Request(
        f"http://127.0.0.1:{port}{path}",
        data=json.dumps(body).encode(),
        headers={"Content-Type": "application/json"}, method="POST")
    try:
        with urllib.request.urlopen(req, timeout=3) as r:
            return r.status, json.loads(r.read())
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read())


class TestConfigEndpoint:
    def test_reports_active_chassis(self, server):
        serve.Handler.chassis = chassis.load("jackal")
        status, body = _get(server, "/api/config")
        assert status == 200
        assert body["chassis"] == "jackal"
        assert body["features"]["fwd2m"] is False
        assert body["limits"]["maxLinear"] == 3.0
        assert body["rear_camera"] == "realsense"

    def test_reports_agrobot(self, server):
        serve.Handler.chassis = chassis.load("agrobot")
        status, body = _get(server, "/api/config")
        assert body["chassis"] == "agrobot"
        assert body["features"]["modbus_slider"] is True
        assert body["features"]["battery"] is True
        assert body["battery"]["minV"] == 42.0
        assert body["battery"]["maxV"] == 58.0


class TestPerChassisVelocityLimits:
    def test_jackal_rejects_over_limit(self, server):
        serve.Handler.chassis = chassis.load("jackal")   # max 3.0
        status, body = _post(server, "/api/cmd_vel",
                             {"linear_x": 5.0, "angular_z": 0.0})
        assert status == 400

    def test_jackal_accepts_within_limit(self, server):
        serve.Handler.chassis = chassis.load("jackal")
        # within 3.0 but no ROS publisher -> 503 (not 400)
        status, body = _post(server, "/api/cmd_vel",
                             {"linear_x": 2.0, "angular_z": 0.0})
        assert status == 503

    def test_agrobot_accepts_same_value_jackal_rejects(self, server):
        serve.Handler.chassis = chassis.load("agrobot")     # max 15.0
        # 5.0 exceeds jackal's 3.0 but is fine for agrobot -> 503 (no ROS), not 400
        status, body = _post(server, "/api/cmd_vel",
                             {"linear_x": 5.0, "angular_z": 0.0})
        assert status == 503


class TestFwd2mFeatureGate:
    def test_jackal_fwd2m_unsupported(self, server):
        serve.Handler.chassis = chassis.load("jackal")
        status, body = _post(server, "/api/fwd2m", {"speed": 0.5})
        assert status == 503
        assert "error" in body
