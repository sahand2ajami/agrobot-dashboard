#!/usr/bin/env python3
"""Battery drain-test dashboard server.

Extends serve.py by *registering* extra routes (Handler.add_route) — no
monkey-patching — and reuses the shared server-side auto-drive primitive
(serve.drive_distance) so the 2 m forward/backward legs are exactly the same
motion as the manual "2m Fwd / 2m Bwd" buttons.

  • POST /api/battery_test/start   {speed, stop_v?, plant_timeout?} — begin cycle
  • POST /api/battery_test/stop                      — kill everything, robot stops,
                                                        auger/planter command bits cleared
  • GET  /api/battery_test/status                    — {running, phase, plant_which,
                                                        forward, backward, cycles,
                                                        augers, planters, plant_timeouts,
                                                        speed}

The auto cycle runs on a background thread (a *server-side* loop, so the test
survives a browser hiccup / closed tab). One full pass is:

    forward 2 m → auger → planter → backward 2 m → auger → planter → (repeat)

Each phase is strictly sequential — the robot only drives when the actuators
are idle, and the actuators only run while the robot is stopped (no overlap).
The loop repeats until the battery pack voltage drops to the cutoff (`stop_v`,
default = the chassis's 0 %/`battery_min_v` point) or Stop is pressed.

The plant step drives the AMR↔PLC handshake (Handler.plc): it marks the AMR
stationary (%MW5112 = 1), fires a momentary auger/planter start pulse
(%MW5110/5111 written 1 → 0, so no latched bit → no free-run), and waits for the
cycle via the Clear-of-Ground bit (%MW5100/5101 bit1: 1 home → 0 working → 1
done). A start that never leaves home within `_PLANT_START_TIMEOUT`, and the
overall `plant_timeout`, are the hard safety nets so a machine that is not in
the AMR-gated cycle mode can never hang the endurance test — the plant is
skipped/timed-out and the drive legs keep draining the battery. When no PLC is
present (jackal, or PLC offline) the plant steps no-op and the test degrades to
pure 2 m forward/backward motion.

The same page also exposes manual auger / planter / auger+planter buttons (via
the shared /api/amr/* routes) so every step — forward, auger, planter, backward
— can be run by hand as well as by the Auto loop.

The root URL serves battery_test.html. Default HTTP port: 8770
(see launch_dashboard_battery_test.sh).
"""
import json
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
import serve as _serve
import serve_plc as _serve_plc   # reuse the AMR handshake routes + %MW5112 auto-writer


class BatteryTest:
    """Owns the auto-cycle background thread and its counters. One locked
    object; the HTTP routes and the worker thread are its only callers."""

    _PLANT_POLL          = 0.2   # s between handshake polls while a plant runs
    _PLANT_START_TIMEOUT = 8.0   # s: Clear-of-Ground must drop within this or the cycle never started
    _SETTLE              = 0.4   # s to let motion settle before/after a phase change

    def __init__(self):
        self.lock       = threading.Lock()
        self.thread     = None
        self.running    = False
        self.phase      = "idle"     # idle | forward | plant | backward
        self.plant_which= ""         # "auger" | "planter" | "" — detail for the UI
        self.forward    = 0          # completed 2 m forward drives
        self.backward   = 0          # completed 2 m backward drives
        self.cycles     = 0          # completed forward+backward pairs
        self.augers     = 0          # completed auger sequences
        self.planters   = 0          # completed planter sequences
        self.plant_timeouts = 0      # plant steps that hit the timeout (no completion seen)
        self.speed      = 0.0        # m/s used for the drives
        self.stop_v     = 0.0        # battery cutoff voltage
        self.plant_timeout = 60.0    # s: hard cap on one auger/planter cycle
        self._cancel    = False      # set by stop(); polled by the worker

    def status(self):
        with self.lock:
            return {
                "running":        self.running,
                "phase":          self.phase,
                "plant_which":    self.plant_which,
                "forward":        self.forward,
                "backward":       self.backward,
                "cycles":         self.cycles,
                "augers":         self.augers,
                "planters":       self.planters,
                "plant_timeouts": self.plant_timeouts,
                "speed":          round(self.speed, 3),
            }

    def start(self, speed, stop_v, plant_timeout=60.0):
        """Begin a run. Returns False if one is already active (idempotent)."""
        with self.lock:
            if self.running:
                return False
            prev = self.thread
        # A just-stopped worker may still be in its cleanup; join it before
        # we reset the counters so its `finally` can't clobber the new run.
        if prev is not None and prev.is_alive():
            prev.join(timeout=3)
        with self.lock:
            if self.running:
                return False
            self.running    = True
            self._cancel    = False
            self.phase      = "forward"
            self.plant_which= ""
            self.forward    = 0
            self.backward   = 0
            self.cycles     = 0
            self.augers     = 0
            self.planters   = 0
            self.plant_timeouts = 0
            self.speed      = speed
            self.stop_v     = stop_v
            self.plant_timeout = plant_timeout
            self.thread     = threading.Thread(
                target=self._run, daemon=True, name="battery-test")
            self.thread.start()
            return True

    def stop(self):
        """Kill the cycle: stop the robot and release the auger/planter bits."""
        with self.lock:
            self._cancel    = True
            self.running    = False
            self.phase      = "idle"
            self.plant_which= ""
        _serve.TELEM.vel.set_command(0.0, 0.0)
        self._clear_plc()

    def _cancelled(self):
        with self.lock:
            return self._cancel

    def _battery_empty(self):
        """True only on a *fresh* reading at/below the cutoff — a momentary
        telemetry gap must not end a multi-hour drain test prematurely."""
        v, last = _serve.TELEM.battery.snapshot()
        connected = last > 0 and (time.monotonic() - last) < 30.0
        return connected and v <= self.stop_v

    # ── PLC helpers (all best-effort; a missing/offline PLC never raises) ─────
    def _amr_moving(self, moving):
        """Tell the PLC the AMR is Moving (%MW5112=2) or Stationary (=1)."""
        plc = _serve.Handler.plc
        if plc is None:
            return
        try:
            plc.amr_set_moving(moving)
        except Exception:
            pass

    def _clear_plc(self):
        """Release both start-sequence command words (%MW5110/5111 → 0)."""
        plc = _serve.Handler.plc
        if plc is None:
            return
        try:
            plc.control_auger("STOP")
            plc.control_planter("STOP")
        except Exception:
            pass

    def _run_plant(self, which):
        """Fire `which` (auger|planter) as a momentary start pulse, then wait for
        the cycle to finish via the "Clear of Ground" handshake bit.

        Clear-of-Ground = %MW5100 bit1 (auger) / %MW5101 bit1 (planter): 1 while
        the implement is home/raised, 0 while it is down doing the cycle. So a
        full cycle is Clear-of-Ground 1 → 0 (started, left home) → 1 (finished,
        back home). Completion is accepted only after the drop was seen, so the
        resting value of 1 can't finish it instantly. The command word is not
        latched (the pulse self-clears), so the machine can't free-run.

        Returns 'done' | 'timeout' | 'cancelled' | 'skipped' (PLC absent/offline).
        A start that never leaves home within `_PLANT_START_TIMEOUT` returns
        'timeout' (the cycle did not start — e.g. not Enabled/Auto).
        """
        plc = _serve.Handler.plc
        if plc is None:
            return "skipped"
        reg      = 5110 if which == "auger" else 5111
        word_key = "mw5100" if which == "auger" else "mw5101"

        resp = plc.amr_write(reg, 1, pulse=True)   # momentary start request
        if not resp.get("connected"):
            return "skipped"

        deadline     = time.monotonic() + self.plant_timeout
        start_by     = time.monotonic() + self._PLANT_START_TIMEOUT
        left_home    = False          # seen Clear-of-Ground drop to 0 (cycle running)
        outcome      = "timeout"
        while time.monotonic() < deadline:
            if self._cancelled():
                outcome = "cancelled"
                break
            poll = plc.amr_poll()
            if not poll.get("connected"):
                outcome = "skipped"
                break
            word = poll.get(word_key)
            if word is not None:
                clear_of_ground = bool((word >> 1) & 1)
                if not clear_of_ground:
                    left_home = True                 # down / working
                elif left_home:
                    outcome = "done"                 # back home → cycle finished
                    break
            # It never left home in time → the cycle did not start; stop waiting.
            if not left_home and time.monotonic() > start_by:
                outcome = "timeout"
                break
            time.sleep(self._PLANT_POLL)
        return outcome

    def _plant_all(self):
        """Run auger to completion, then planter. Returns False if the run was
        cancelled or the battery hit the cutoff mid-way (caller breaks the loop)."""
        self._amr_moving(False)           # AMR is parked for the planting cycle
        time.sleep(self._SETTLE)          # let the chassis settle before actuating
        for which in ("auger", "planter"):
            if self._cancelled() or self._battery_empty():
                return False
            with self.lock:
                self.phase       = "plant"
                self.plant_which = which
            outcome = self._run_plant(which)
            with self.lock:
                if outcome == "done":
                    if which == "auger":
                        self.augers += 1
                    else:
                        self.planters += 1
                elif outcome == "timeout":
                    self.plant_timeouts += 1
            if outcome == "cancelled":
                return False
        return not self._cancelled()

    def _leg(self, direction):
        """Drive one 2 m leg. Returns the drive_distance result dict."""
        with self.lock:
            self.phase       = direction
            self.plant_which = ""
        self._amr_moving(True)
        return _serve.drive_distance(direction, self.speed, cancel=self._cancelled)

    def _run(self):
        try:
            while not self._cancelled() and not self._battery_empty():
                # ── forward 2 m ──────────────────────────────────────────────
                res = self._leg("forward")
                if self._cancelled() or res.get("error") or res.get("aborted"):
                    break
                if res.get("done"):
                    with self.lock:
                        self.forward += 1
                if self._cancelled() or self._battery_empty():
                    break

                # ── auger, then planter ─────────────────────────────────────
                if not self._plant_all():
                    break
                if self._cancelled() or self._battery_empty():
                    break
                time.sleep(self._SETTLE)

                # ── backward 2 m ────────────────────────────────────────────
                res = self._leg("backward")
                if self._cancelled() or res.get("error") or res.get("aborted"):
                    break
                if res.get("done"):
                    with self.lock:
                        self.backward += 1
                        self.cycles   += 1
                if self._cancelled() or self._battery_empty():
                    break

                # ── auger, then planter (after the return leg too) ──────────
                if not self._plant_all():
                    break
                time.sleep(self._SETTLE)
        finally:
            _serve.TELEM.vel.set_command(0.0, 0.0)
            self._clear_plc()
            with self.lock:
                self.running     = False
                self.phase       = "idle"
                self.plant_which = ""


_TEST = BatteryTest()


def _read_json_body(handler):
    try:
        length = int(handler.headers.get("Content-Length", 0))
        if length > _serve.MAX_POST_BYTES:
            handler._json_response(413, b'{"error":"Request too large"}')
            return None
        return json.loads(handler.rfile.read(length) if length else b"{}")
    except Exception:
        handler._json_response(400, b'{"error":"invalid JSON"}')
        return None


def _fwd2m_ok(handler):
    """The auto cycle relies on wheel encoders — same guard as /api/fwd2m."""
    ch = _serve.Handler.chassis
    if ch is not None and not ch.has_feature("fwd2m"):
        handler._json_response(503, b'{"error":"auto-drive not supported on this chassis"}')
        return False
    return True


def _serve_bt_start(handler):
    if not _fwd2m_ok(handler):
        return
    data = _read_json_body(handler)
    if data is None:
        return
    try:
        speed = float(data.get("speed", 0.5))
    except (TypeError, ValueError):
        handler._json_response(400, b'{"error":"invalid speed"}')
        return

    # Cutoff defaults to the chassis's 0 %/battery_min_v point ("empty").
    ch = _serve.Handler.chassis
    default_v = ch.battery_min_v if ch is not None else 42.0
    try:
        stop_v = float(data.get("stop_v", default_v))
    except (TypeError, ValueError):
        stop_v = default_v

    try:
        plant_timeout = float(data.get("plant_timeout", 60.0))
    except (TypeError, ValueError):
        plant_timeout = 60.0
    plant_timeout = max(1.0, min(600.0, plant_timeout))

    started = _TEST.start(speed, stop_v, plant_timeout)
    _serve.log.info("[battery_test] start speed=%.2f stop_v=%.1f plant_timeout=%.0f -> %s",
                    speed, stop_v, plant_timeout, "started" if started else "already running")
    handler._json_response(200, json.dumps(
        {"started": started, **_TEST.status()}).encode())


def _serve_bt_stop(handler):
    _TEST.stop()
    _serve.log.info("[battery_test] stop")
    handler._json_response(200, json.dumps({"stopped": True, **_TEST.status()}).encode())


def _serve_bt_status(handler):
    handler._json_response(200, json.dumps(_TEST.status()).encode())


def _register():
    _serve.Handler.INDEX_FILE = "battery_test.html"
    _serve.Handler.add_route("POST", "/api/battery_test/start",  _serve_bt_start)
    _serve.Handler.add_route("POST", "/api/battery_test/stop",   _serve_bt_stop)
    _serve.Handler.add_route("GET",  "/api/battery_test/status", _serve_bt_status)
    # Reuse the AMR handshake routes so the page's manual auger/planter/both
    # buttons fire the same momentary pulse and read the Clear-of-Ground bit as
    # the PLC tab. (503 on a chassis without a PLC — page hides the buttons.)
    _serve.Handler.add_route("GET",  "/api/amr/poll",  _serve_plc._serve_amr_poll)
    _serve.Handler.add_route("POST", "/api/amr/write", _serve_plc._serve_amr_write)


def main():
    _register()
    # %MW5112 auto-writer (Moving/Stationary from velocity), same as launch_dashboard_plc.
    threading.Thread(target=_serve_plc._amr_state_loop, daemon=True,
                     name="amr-state").start()
    _serve.main()


if __name__ == "__main__":
    main()
