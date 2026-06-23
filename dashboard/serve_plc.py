#!/usr/bin/env python3
"""Combined dashboard server: main teleoperation + AMR↔PLC handshake registers.

Extends serve.py (via monkey-patching) with:
  • /api/amr/poll   — read all handshake registers (%MW5100–5112) at once
  • /api/amr/ping   — latency check to the PLC
  • /api/amr/write  — write a handshake register (%MW5110/5111/5112)

  Auto-writes %MW5112 every time the AMR moving state changes:
    %MW5112 = 2  →  Bit 1 set — AMR is Moving
    %MW5112 = 1  →  Bit 0 set — AMR is Stationary

The root URL serves plc_combined.html (4-tab UI: Camera · GPS · Connectivity · PLC).
"""
import json
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
import serve as _serve

from amr_plc_serve import ModbusHelper, ALL_BASE, ALL_COUNT

# ── AMR handshake Modbus client ───────────────────────────────────────────────
# Lazily initialised from chassis config once Handler.chassis is set by main().

_amr_mb_lock = threading.Lock()
_amr_mb: ModbusHelper = None   # type: ignore[assignment]
_amr_moving = None             # None = unknown, True = Moving, False = Stationary


def _init_amr_mb() -> None:
    """Create the handshake ModbusHelper from chassis.plc_host/plc_port."""
    global _amr_mb
    chassis = _serve.Handler.chassis
    if chassis is None or not chassis.plc_enabled:
        return
    with _amr_mb_lock:
        if _amr_mb is not None:
            return
        _amr_mb = ModbusHelper(chassis.plc_host, chassis.plc_port)
        _serve.log.info(
            "[amr_plc] Handshake Modbus → %s:%d  (%%MW5112 auto-write active)",
            chassis.plc_host, chassis.plc_port,
        )


def _amr_state_loop() -> None:
    """Background thread: write %%MW5112 = 2 (Moving) or 1 (Stationary)
    whenever the AMR movement state changes, derived from Handler velocity state.
    """
    global _amr_moving
    while True:
        time.sleep(0.15)   # 6-7 Hz checks — fast enough, low overhead

        if _amr_mb is None:
            _init_amr_mb()
            continue

        # Read velocity state (Handler class-level locks)
        with _serve.Handler._vel_lock:
            lx     = _serve.Handler._vel_lin
            az     = _serve.Handler._vel_ang
            active = _serve.Handler._vel_active
            last_t = _serve.Handler._vel_last

        # AMR is moving when a fresh non-zero command is live
        moving = (
            active
            and (abs(lx) > 1e-4 or abs(az) > 1e-4)
            and (time.monotonic() - last_t) < _serve.Handler.VEL_TIMEOUT
        )

        if moving == _amr_moving:
            continue   # no state change — skip the Modbus write
        _amr_moving = moving

        val = 2 if moving else 1
        ok, msg = _amr_mb.write_word(5112, val)
        _serve.log.info(
            "[amr_plc] %%MW5112 = %d (%s)  %s",
            val,
            "Moving" if moving else "Stationary",
            "OK" if ok else f"FAIL: {msg}",
        )


threading.Thread(target=_amr_state_loop, daemon=True, name="amr-state").start()


# ── Monkey-patch: serve plc_combined.html at root ────────────────────────────

_orig_do_GET = _serve.Handler.do_GET


def _plc_do_GET(self):
    p = self.path.split("?")[0].split("#")[0]
    if p in ("/", "/index.html", "/plc_combined.html"):
        html = Path(__file__).parent / "plc_combined.html"
        try:
            body = html.read_bytes()
        except FileNotFoundError:
            self.send_error(404, "plc_combined.html not found")
            return
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            pass
    elif p == "/api/amr/poll":
        self._serve_amr_poll()
    elif p == "/api/amr/ping":
        self._serve_amr_ping()
    else:
        _orig_do_GET(self)


_serve.Handler.do_GET = _plc_do_GET


_orig_do_POST = _serve.Handler.do_POST


def _plc_do_POST(self):
    if self.path == "/api/amr/write":
        self._serve_amr_write()
    else:
        _orig_do_POST(self)


_serve.Handler.do_POST = _plc_do_POST


# ── AMR API methods added to Handler ─────────────────────────────────────────

def _serve_amr_poll(self):
    if _amr_mb is None:
        self._json_response(
            503,
            b'{"connected":false,"error":"PLC not enabled or not initialised yet"}',
        )
        return
    self._json_response(200, json.dumps(_amr_mb.poll()).encode())


def _serve_amr_ping(self):
    if _amr_mb is None:
        self._json_response(
            503, b'{"connected":false,"error":"PLC not enabled"}'
        )
        return
    t0 = time.monotonic()
    words, err = _amr_mb.read_words(ALL_BASE, 1)
    ms = round((time.monotonic() - t0) * 1000, 1)
    self._json_response(
        200,
        json.dumps(
            {
                "connected":  words is not None,
                "latency_ms": ms if words is not None else None,
                "message":    "OK" if words is not None else err,
            }
        ).encode(),
    )


def _serve_amr_write(self):
    if _amr_mb is None:
        self._json_response(503, b'{"error":"PLC not enabled"}')
        return
    length = int(self.headers.get("Content-Length", 0))
    raw = self.rfile.read(length) if length else b"{}"
    try:
        data = json.loads(raw)
    except Exception:
        self._json_response(400, b'{"error":"invalid JSON"}')
        return
    reg = data.get("reg")
    val = data.get("value")
    if reg is None or val is None:
        self._json_response(400, b'{"error":"reg and value required"}')
        return
    try:
        reg = int(reg)
        val = int(val) & 0xFFFF
    except (TypeError, ValueError):
        self._json_response(400, b'{"error":"reg and value must be integers"}')
        return

    ok, msg = _amr_mb.write_word(reg, val)
    rb, _   = _amr_mb.read_words(reg, 1)
    readback = rb[0] if rb else None
    self._json_response(
        200 if ok else 503,
        json.dumps(
            {
                "success":       ok,
                "message":       msg,
                "reg":           f"%MW{reg}",
                "value_written": val,
                "readback":      readback,
                "confirmed":     (readback == val) if readback is not None else None,
            }
        ).encode(),
    )


_serve.Handler._serve_amr_poll  = _serve_amr_poll
_serve.Handler._serve_amr_ping  = _serve_amr_ping
_serve.Handler._serve_amr_write = _serve_amr_write


if __name__ == "__main__":
    _serve.main()
