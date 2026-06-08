"""
Integration tests for dashboard/serve.py HTTP endpoints.

Starts a real server in-process on a random port; tests confirm:
  - security behaviour (bounds, body size, static whitelist, CORS absent)
  - sensor-absent degradation (404/503 with JSON errors, not crashes)
  - GNSS schema validation
  - cmd_vel velocity bounds
"""
import json
import os
import socket
import sys
import tempfile
import threading
import time
import urllib.request
import urllib.error
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parent.parent / "dashboard"))
import serve  # noqa: E402  (after sys.path tweak)


# ── Server fixture ────────────────────────────────────────────────────────────

def _free_port() -> int:
    with socket.socket() as s:
        s.bind(("", 0))
        return s.getsockname()[1]


@pytest.fixture(scope="module")
def server(tmp_path_factory):
    """Start a serve.py HTTP server for the duration of the test module."""
    port = _free_port()
    gnss_dir = tmp_path_factory.mktemp("gnss")
    gnss_file = str(gnss_dir / "gnss_coords.json")

    serve.Handler.gnss_file  = gnss_file
    serve.Handler.speed_cmd_pub = None   # no ROS in tests

    httpd = serve._Server(("127.0.0.1", port), serve.Handler)
    t = threading.Thread(target=httpd.serve_forever, daemon=True)
    t.start()
    time.sleep(0.05)   # let the server bind

    yield port, gnss_file

    httpd.shutdown()


def _get(port, path, *, timeout=3):
    url = f"http://127.0.0.1:{port}{path}"
    try:
        with urllib.request.urlopen(url, timeout=timeout) as resp:
            return resp.status, resp.read(), dict(resp.headers)
    except urllib.error.HTTPError as e:
        return e.code, e.read(), {}


def _post(port, path, body_dict, *, timeout=3):
    url  = f"http://127.0.0.1:{port}{path}"
    data = json.dumps(body_dict).encode()
    req  = urllib.request.Request(url, data=data,
                                  headers={"Content-Type": "application/json"},
                                  method="POST")
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            return resp.status, json.loads(resp.read())
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read())


# ── Static file whitelist ─────────────────────────────────────────────────────

class TestStaticFileWhitelist:
    def test_root_returns_200(self, server):
        port, _ = server
        status, _, _ = _get(port, "/")
        assert status == 200

    def test_index_html_returns_200(self, server):
        port, _ = server
        status, _, _ = _get(port, "/index.html")
        assert status == 200

    def test_serve_py_blocked(self, server):
        port, _ = server
        status, _, _ = _get(port, "/serve.py")
        assert status == 403

    def test_arbitrary_path_blocked(self, server):
        port, _ = server
        status, _, _ = _get(port, "/../../etc/passwd")
        assert status == 403

    def test_api_path_not_blocked_by_whitelist(self, server):
        port, _ = server
        # /api/gnss with no file should be 404 not 403
        status, _, _ = _get(port, "/api/gnss")
        assert status == 404


# ── CORS absent ───────────────────────────────────────────────────────────────

class TestNoCors:
    def test_no_acao_on_index(self, server):
        port, _ = server
        _, _, headers = _get(port, "/")
        assert "Access-Control-Allow-Origin" not in headers

    def test_no_acao_on_gnss(self, server):
        port, _ = server
        _, _, headers = _get(port, "/api/gnss")
        assert "Access-Control-Allow-Origin" not in headers


# ── GNSS schema validation ────────────────────────────────────────────────────

class TestGnssServing:
    def test_missing_file_returns_404(self, server):
        port, gnss_file = server
        # Ensure file doesn't exist
        Path(gnss_file).unlink(missing_ok=True)
        status, _, _ = _get(port, "/api/gnss")
        assert status == 404

    def test_corrupt_json_returns_503(self, server):
        port, gnss_file = server
        Path(gnss_file).write_text("{not valid json")
        status, _, _ = _get(port, "/api/gnss")
        assert status == 503

    def test_incomplete_payload_returns_503(self, server):
        port, gnss_file = server
        Path(gnss_file).write_text(json.dumps({"lat": 51.5}))
        status, _, _ = _get(port, "/api/gnss")
        assert status == 503

    def test_valid_payload_returns_200(self, server):
        port, gnss_file = server
        good = {"lat": 51.5, "lon": -0.1, "fix": 4,
                "sats": 12, "hdop": 0.9, "alt": 50.0,
                "fix_label": "RTK Fixed"}
        Path(gnss_file).write_text(json.dumps(good))
        status, body, _ = _get(port, "/api/gnss")
        assert status == 200
        data = json.loads(body)
        assert data["fix"] == 4

    def test_fresh_fix_reports_fixed_state(self, server):
        from datetime import datetime, timezone
        port, gnss_file = server
        Path(gnss_file).write_text(json.dumps({
            "lat": 51.5, "lon": -0.1, "fix": 4, "sats": 12, "hdop": 0.9,
            "alt": 50.0, "fix_label": "RTK Fixed", "sats_in_view": 20,
            "ts": datetime.now(timezone.utc).isoformat(),
        }))
        status, body, _ = _get(port, "/api/gnss")
        assert status == 200
        d = json.loads(body)
        assert d["gps_state"] == "fixed"
        assert d["connected"] is True
        assert d["sats_in_view"] == 20

    def test_heartbeat_reports_no_signal(self, server):
        # Heartbeat payload: no lat/lon, fresh ts -> connected but no signal.
        from datetime import datetime, timezone
        port, gnss_file = server
        Path(gnss_file).write_text(json.dumps({
            "has_fix": False, "fix": 0, "sats": 4, "sats_in_view": 9,
            "ts": datetime.now(timezone.utc).isoformat(),
        }))
        status, body, _ = _get(port, "/api/gnss")
        assert status == 200
        d = json.loads(body)
        assert d["gps_state"] == "no_signal"
        assert d["connected"] is True
        assert d["has_fix"] is False


# ── cmd_vel velocity bounds ───────────────────────────────────────────────────

class TestCmdVelBounds:
    def test_valid_velocity_returns_503_no_ros(self, server):
        # Valid but no ROS publisher → 503
        port, _ = server
        status, body = _post(port, "/api/cmd_vel",
                             {"linear_x": 0.3, "angular_z": 0.1})
        assert status == 503

    def test_linear_out_of_range_returns_400(self, server):
        port, _ = server
        status, body = _post(port, "/api/cmd_vel",
                             {"linear_x": 999.0, "angular_z": 0.0})
        assert status == 400
        assert "error" in body

    def test_angular_out_of_range_returns_400(self, server):
        port, _ = server
        status, body = _post(port, "/api/cmd_vel",
                             {"linear_x": 0.0, "angular_z": 999.0})
        assert status == 400

    def test_negative_linear_out_of_range_returns_400(self, server):
        port, _ = server
        status, body = _post(port, "/api/cmd_vel",
                             {"linear_x": -999.0, "angular_z": 0.0})
        assert status == 400

    def test_boundary_linear_accepted(self, server):
        # Exactly at limit should not be rejected by bounds check
        port, _ = server
        status, _ = _post(port, "/api/cmd_vel",
                          {"linear_x": serve.MAX_LIN_INPUT, "angular_z": 0.0})
        assert status in (200, 503)  # not 400

    def test_malformed_json_returns_400(self, server):
        port, _ = server
        url  = f"http://127.0.0.1:{port}/api/cmd_vel"
        data = b"not json"
        req  = urllib.request.Request(url, data=data,
                                      headers={"Content-Type": "application/json"},
                                      method="POST")
        try:
            urllib.request.urlopen(req, timeout=3)
            assert False, "Expected error"
        except urllib.error.HTTPError as e:
            assert e.code == 400

    def test_oversized_body_returns_413(self, server):
        port, _ = server
        url  = f"http://127.0.0.1:{port}/api/cmd_vel"
        data = json.dumps({"linear_x": 0.1, "angular_z": 0.0,
                           "pad": "x" * 70_000}).encode()
        req  = urllib.request.Request(url, data=data,
                                      headers={"Content-Type": "application/json"},
                                      method="POST")
        try:
            urllib.request.urlopen(req, timeout=3)
            assert False, "Expected error"
        except urllib.error.HTTPError as e:
            assert e.code == 413


# ── Sensor absent degradation ─────────────────────────────────────────────────

class TestSensorAbsent:
    def test_camera_returns_503_when_no_frame(self, server):
        port, _ = server
        # No camera thread running → Handler._cam_jpeg is None
        serve.Handler._cam_jpeg = None
        status, _, _ = _get(port, "/api/camera")
        assert status == 503

    def test_zed_returns_503_when_no_frame(self, server):
        port, _ = server
        serve.Handler._zed_jpeg = None
        status, _, _ = _get(port, "/api/zed")
        assert status == 503

    def test_detection_returns_503_when_no_frame(self, server):
        port, _ = server
        serve.Handler._det_jpeg = None
        status, _, _ = _get(port, "/api/detection")
        assert status == 503

    def test_camera_status_json_when_no_frame(self, server):
        port, _ = server
        serve.Handler._cam_jpeg  = None
        serve.Handler._cam_last_error = "test-error"
        status, body, _ = _get(port, "/api/camera/status")
        assert status == 200
        data = json.loads(body)
        assert data["has_frame"] is False
        assert data["last_error"] == "test-error"

    def test_wheel_odom_returns_zeros_initially(self, server):
        port, _ = server
        serve.Handler._odom_l = 0
        serve.Handler._odom_r = 0
        status, body, _ = _get(port, "/api/wheel_odom")
        assert status == 200
        data = json.loads(body)
        assert data["left"] == 0
        assert data["right"] == 0


# ── Chassis battery endpoint ──────────────────────────────────────────────────

class TestChassisBattery:
    def test_no_data_reports_disconnected(self, server):
        port, _ = server
        with serve.Handler._chassis_batt_lock:
            serve.Handler._chassis_batt_last     = 0.0
            serve.Handler._chassis_batt_smoothed = 0.0
        status, body, _ = _get(port, "/api/chassis_battery")
        assert status == 200
        data = json.loads(body)
        assert data["connected"] is False
        assert data["voltage_v"] == 0.0

    def test_fresh_reading_reports_voltage(self, server):
        port, _ = server
        with serve.Handler._chassis_batt_lock:
            serve.Handler._chassis_batt_last     = time.monotonic()
            serve.Handler._chassis_batt_smoothed = 51.4
        status, body, _ = _get(port, "/api/chassis_battery")
        assert status == 200
        data = json.loads(body)
        assert data["connected"] is True
        assert data["voltage_v"] == 51.4

    def test_stale_reading_reports_disconnected(self, server):
        port, _ = server
        with serve.Handler._chassis_batt_lock:
            serve.Handler._chassis_batt_last     = time.monotonic() - 60.0
            serve.Handler._chassis_batt_smoothed = 51.4
        status, body, _ = _get(port, "/api/chassis_battery")
        assert status == 200
        data = json.loads(body)
        assert data["connected"] is False


# ── DMS coordinate formatting ─────────────────────────────────────────────────

class TestDms:
    def test_north(self):
        assert serve._dms(51.5, True) == "51°30'00.00\"N"

    def test_south_west_hemispheres(self):
        assert serve._dms(-33.8688, True).endswith("S")
        assert serve._dms(-70.6693, False).endswith("W")

    def test_east(self):
        assert serve._dms(0.125, False).endswith("E")

    def test_none_returns_empty(self):
        assert serve._dms(None, True) == ""


# ── Seedling planting log ──────────────────────────────────────────────────────

class TestPlantLog:
    def test_logs_seedling_with_dms(self, server):
        port, _ = server
        status, body = _post(port, "/api/plant", {
            "ts": "2026-06-05T12:00:00Z", "count": 1,
            "lat": 51.5, "lon": -0.12, "fix": 4, "sats": 12,
        })
        assert status == 200
        assert body["ok"] is True
        assert body["lat_dms"].endswith("N")
        assert body["lon_dms"].endswith("W")

    def test_missing_fields_returns_400(self, server):
        port, _ = server
        status, body = _post(port, "/api/plant", {"lat": 1.0})
        assert status == 400
        assert "error" in body
