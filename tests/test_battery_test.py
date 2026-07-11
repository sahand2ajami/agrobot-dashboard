"""
Tests for the battery drain-test dashboard (dashboard/serve_battery.py) and the
shared server-side auto-drive primitive it reuses (serve.drive_distance).

Runs without ROS or hardware. A lightweight background "encoder feeder" thread
advances TELEM.odom in whichever direction the current velocity command points,
so a fixed-distance drive completes exactly as it would on the real chassis.
"""
import json
import socket
import sys
import threading
import time
import urllib.error
import urllib.request
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parent.parent / "dashboard"))
import chassis           # noqa: E402
import serve             # noqa: E402
import serve_battery     # noqa: E402

serve_battery._register()   # register /api/battery_test/* + set INDEX_FILE


# ── HTTP fixture + helpers (mirrors test_chassis_config.py) ──────────────────

def _free_port() -> int:
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


@pytest.fixture(scope="module")
def server():
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
        with urllib.request.urlopen(req, timeout=5) as r:
            return r.status, json.loads(r.read())
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read())


# ── Encoder feeder: simulate wheels following the commanded velocity ─────────

class _OdomFeeder:
    """Advance TELEM.odom in the commanded direction so drives complete.

    Swaps in an isolated OdometryState for the duration and restores the
    original on exit, so these tests never leak encoder counts into the
    shared TelemetryStore other test modules rely on."""

    def __init__(self, step=4000):
        self.step = step
        self._stop = threading.Event()
        self._t = None
        self._saved = None

    def __enter__(self):
        from agrobot_dashboard.services.telemetry import OdometryState
        self._saved = serve.TELEM.odom
        serve.TELEM.odom = OdometryState()
        self._t = threading.Thread(target=self._run, daemon=True)
        self._t.start()
        time.sleep(0.05)   # let the first reading land (odom "connected")
        return self

    def __exit__(self, *exc):
        self._stop.set()
        self._t.join(timeout=2)
        serve.TELEM.odom = self._saved

    def _run(self):
        pos = 1_000_000.0
        serve.TELEM.odom.update(int(pos), int(pos))
        while not self._stop.is_set():
            with serve.TELEM.vel.lock:
                lin = serve.TELEM.vel.lin
            direction = 1 if lin > 1e-6 else -1 if lin < -1e-6 else 0
            pos += direction * self.step
            serve.TELEM.odom.update(int(pos), int(pos))
            time.sleep(0.01)


# ── serve.drive_distance ─────────────────────────────────────────────────────

class TestDriveDistance:
    def test_encoder_not_connected(self):
        """No fresh odom -> error 'encoder', returned immediately (no 30 s wait)."""
        from agrobot_dashboard.services.telemetry import OdometryState
        serve.Handler.chassis = chassis.load("agrobot")
        saved = serve.TELEM.odom
        serve.TELEM.odom = OdometryState()   # fresh store: last_update == 0
        try:
            t0 = time.monotonic()
            res = serve.drive_distance("forward", 0.5)
        finally:
            serve.TELEM.odom = saved
        assert res.get("error") == "encoder"
        assert res["done"] is False
        assert time.monotonic() - t0 < 1.0   # guard fires fast, no deadline wait

    def test_forward_completes(self):
        serve.Handler.chassis = chassis.load("agrobot")
        with _OdomFeeder():
            res = serve.drive_distance("forward", 0.5)
        assert res.get("error") is None
        assert res["done"] is True
        assert res["aborted"] is False
        assert res["traveled_m"] >= 2.0

    def test_backward_completes(self):
        serve.Handler.chassis = chassis.load("agrobot")
        with _OdomFeeder():
            res = serve.drive_distance("backward", 0.5)
        assert res["done"] is True
        assert res["traveled_m"] >= 2.0

    def test_cancel_aborts(self):
        serve.Handler.chassis = chassis.load("agrobot")
        cancelled = {"v": False}
        with _OdomFeeder(step=50):   # slow so we can cancel mid-drive
            t = threading.Timer(0.15, lambda: cancelled.update(v=True))
            t.start()
            res = serve.drive_distance("forward", 0.5, cancel=lambda: cancelled["v"])
            t.cancel()
        assert res["aborted"] is True
        assert res["done"] is False


# ── /api/fwd2m direction plumbing ────────────────────────────────────────────

class TestFwd2mDirection:
    def test_backward_accepted_and_completes(self, server):
        serve.Handler.chassis = chassis.load("agrobot")
        with _OdomFeeder():
            status, body = _post(server, "/api/fwd2m",
                                 {"speed": 0.5, "direction": "backward"})
        assert status == 200
        assert body["done"] is True


# ── /api/battery_test/* endpoints ────────────────────────────────────────────

class TestBatteryTestEndpoints:
    def test_status_idle_default(self, server):
        serve.Handler.chassis = chassis.load("agrobot")
        serve_battery._TEST.stop()   # ensure a clean baseline
        status, body = _get(server, "/api/battery_test/status")
        assert status == 200
        assert body["running"] is False
        assert body["phase"] == "idle"
        assert body["forward"] == 0 and body["backward"] == 0 and body["cycles"] == 0

    def test_start_unsupported_on_jackal(self, server):
        serve.Handler.chassis = chassis.load("jackal")
        status, body = _post(server, "/api/battery_test/start", {"speed": 0.5})
        assert status == 503
        assert "error" in body

    def test_full_cycle_counts(self, server):
        serve.Handler.chassis = chassis.load("agrobot")
        serve_battery._TEST.stop()
        with _OdomFeeder():
            status, body = _post(server, "/api/battery_test/start", {"speed": 0.5})
            assert status == 200 and body["started"] is True

            # Wait for at least one full forward+backward cycle to complete.
            deadline = time.time() + 15
            cycles = 0
            while time.time() < deadline:
                _, s = _get(server, "/api/battery_test/status")
                cycles = s["cycles"]
                if cycles >= 1:
                    break
                time.sleep(0.1)

            _post(server, "/api/battery_test/stop", {})

        assert cycles >= 1, "auto cycle never completed a forward+backward pair"
        _, s = _get(server, "/api/battery_test/status")
        assert s["running"] is False
        assert s["forward"] >= 1
        assert s["backward"] >= 1

    def test_double_start_is_idempotent(self, server):
        serve.Handler.chassis = chassis.load("agrobot")
        serve_battery._TEST.stop()
        with _OdomFeeder(step=50):   # slow drive so the run is still active
            s1, b1 = _post(server, "/api/battery_test/start", {"speed": 0.3})
            s2, b2 = _post(server, "/api/battery_test/start", {"speed": 0.3})
            _post(server, "/api/battery_test/stop", {})
        assert b1["started"] is True
        assert b2["started"] is False   # already running

    def test_stop_when_idle_ok(self, server):
        serve.Handler.chassis = chassis.load("agrobot")
        status, body = _post(server, "/api/battery_test/stop", {})
        assert status == 200
        assert body["stopped"] is True

    def test_status_has_plant_fields(self, server):
        serve.Handler.chassis = chassis.load("agrobot")
        serve_battery._TEST.stop()
        _, body = _get(server, "/api/battery_test/status")
        for k in ("augers", "planters", "plant_timeouts", "plant_which"):
            assert k in body


# ── Plant orchestration (auger → planter) against a fake PLC ─────────────────

class _FakePlc:
    """Minimal PlcClient stand-in modelling the Clear-of-Ground handshake.

    A momentary `amr_write(reg, 1, pulse=True)` (reg 5110 auger / 5111 planter)
    starts a short cycle: Clear-of-Ground (bit1 of %MW5100/5101) drops to 0 for a
    few polls, then returns to 1 (done). bit2 (Complete) stays latched high like
    the real machine. Records fire/stop calls and the last AMR-state word."""

    _REG = {5110: "auger", 5111: "planter"}

    def __init__(self, ticks=3):
        self._ticks = ticks
        self._cog   = {"auger": 0, "planter": 0}   # polls remaining "in ground"
        self.amr_state = None
        self.log = []
        self._lock = threading.Lock()

    def amr_write(self, reg, value, pulse=False):
        which = self._REG.get(int(reg))
        with self._lock:
            if which is not None and int(value) != 0:
                self.log.append((which, "FIRE"))
                self._cog[which] = self._ticks
        return {"success": True, "connected": True, "pulsed": bool(pulse)}

    def amr_poll(self):
        with self._lock:
            def word(which):
                n = self._cog[which]
                clear = 1 if n == 0 else 0          # bit1: 1 = home, 0 = working
                if n > 0:
                    self._cog[which] = n - 1
                return (1 << 2) | (clear << 1)       # bit2 latched Complete + bit1
            return {"connected": True,
                    "mw5100": word("auger"), "mw5101": word("planter"),
                    "mw5110": 0, "mw5111": 0, "mw5112": self.amr_state or 1}

    def amr_set_moving(self, moving):
        self.amr_state = 2 if moving else 1
        return {"success": True, "connected": True}

    def control_auger(self, command):
        with self._lock:
            self.log.append(("auger", (command or "").upper()))
        return {"success": True, "connected": True}

    def control_planter(self, command):
        with self._lock:
            self.log.append(("planter", (command or "").upper()))
        return {"success": True, "connected": True}


class TestPlantOrchestration:
    def test_auger_then_planter_completes_and_counts(self, server):
        serve.Handler.chassis = chassis.load("agrobot")
        serve_battery._TEST.stop()
        fake = _FakePlc()
        serve.Handler.plc = fake
        s = {}
        try:
            with _OdomFeeder():
                status, body = _post(server, "/api/battery_test/start",
                                     {"speed": 0.5, "plant_timeout": 5})
                assert status == 200 and body["started"] is True
                deadline = time.time() + 20
                while time.time() < deadline:
                    _, s = _get(server, "/api/battery_test/status")
                    if s["augers"] >= 1 and s["planters"] >= 1:
                        break
                    time.sleep(0.1)
                _post(server, "/api/battery_test/stop", {})
        finally:
            serve.Handler.plc = None

        assert s.get("augers", 0) >= 1, "auger cycle never completed via Clear-of-Ground"
        assert s.get("planters", 0) >= 1, "planter cycle never completed via Clear-of-Ground"
        # Auger is fired before planter in each plant step.
        fires = [w for (w, c) in fake.log if c == "FIRE"]
        assert fires[:2] == ["auger", "planter"]
        # AMR was marked stationary for the plant (1) and moving for the drives (2).
        assert fake.amr_state in (1, 2)

    def test_stop_releases_command_bits(self, server):
        serve.Handler.chassis = chassis.load("agrobot")
        serve_battery._TEST.stop()
        fake = _FakePlc(ticks=999)   # never completes on its own
        serve.Handler.plc = fake
        try:
            with _OdomFeeder():
                _post(server, "/api/battery_test/start",
                      {"speed": 0.5, "plant_timeout": 30})
                time.sleep(0.3)
                _post(server, "/api/battery_test/stop", {})
            # stop() must have cleared both command words.
            assert ("auger", "STOP") in fake.log
            assert ("planter", "STOP") in fake.log
        finally:
            serve.Handler.plc = None

    def test_no_plc_degrades_to_moves_only(self, server):
        """With no PLC, plant steps no-op and the forward/backward cycle still runs."""
        serve.Handler.chassis = chassis.load("agrobot")
        serve.Handler.plc = None
        serve_battery._TEST.stop()
        with _OdomFeeder():
            _post(server, "/api/battery_test/start", {"speed": 0.5})
            deadline = time.time() + 15
            cycles = 0
            while time.time() < deadline:
                _, s = _get(server, "/api/battery_test/status")
                cycles = s["cycles"]
                if cycles >= 1:
                    break
                time.sleep(0.1)
            _post(server, "/api/battery_test/stop", {})
        assert cycles >= 1
        assert s["augers"] == 0 and s["planters"] == 0


class TestManualActuatorRoutes:
    """The battery page's manual auger/planter buttons reuse the AMR routes."""

    def test_amr_write_pulse_fires(self, server):
        serve.Handler.chassis = chassis.load("agrobot")
        fake = _FakePlc()
        serve.Handler.plc = fake
        try:
            status, body = _post(server, "/api/amr/write",
                                 {"reg": 5110, "value": 1, "pulse": True})
            assert status == 200
            assert body.get("pulsed") is True
            assert ("auger", "FIRE") in fake.log
        finally:
            serve.Handler.plc = None

    def test_amr_poll_exposes_clear_of_ground(self, server):
        serve.Handler.chassis = chassis.load("agrobot")
        fake = _FakePlc()
        serve.Handler.plc = fake
        try:
            status, body = _get(server, "/api/amr/poll")
            assert status == 200
            # At rest both status words read "home": Clear-of-Ground (bit1) = 1.
            assert (body["mw5100"] >> 1) & 1 == 1
            assert (body["mw5101"] >> 1) & 1 == 1
        finally:
            serve.Handler.plc = None
