#!/usr/bin/env python3
"""Combined dashboard server: main teleoperation + AMR↔PLC handshake registers.

Extends serve.py by *registering* extra routes (Handler.add_route) — no
monkey-patching — and reuses the one PlcClient (Handler.plc) for the
handshake block, so there is a single Modbus socket to the PLC:

  • GET  /api/amr/poll   — read all handshake registers (%MW5100–5112) at once
  • GET  /api/amr/ping   — latency check to the PLC
  • POST /api/amr/write  — write a handshake register (%MW5110/5111/5112)

A background thread auto-writes %MW5112 whenever the AMR moving state changes:
  %MW5112 = 2 → Moving,  %MW5112 = 1 → Stationary

The root URL serves plc_combined.html (4-tab UI: Camera · GPS · Connectivity · PLC).
"""
import json
import sys
import threading
import time
from pathlib import Path
from urllib.parse import parse_qs, urlparse

sys.path.insert(0, str(Path(__file__).parent))
import serve as _serve


def _plc(handler):
    """The shared PlcClient, or None (chassis without PLC / not initialised)."""
    plc = _serve.Handler.plc
    if plc is None:
        handler._json_response(503, b'{"connected":false,"error":"PLC not enabled on this chassis"}')
        return None
    return plc


def _serve_amr_poll(handler):
    plc = _plc(handler)
    if plc is None:
        return
    handler._json_response(200, json.dumps(plc.amr_poll()).encode())


def _serve_amr_ping(handler):
    plc = _plc(handler)
    if plc is None:
        return
    result = plc.ping()
    handler._json_response(200, json.dumps({
        "connected":  result.get("connected", False),
        "latency_ms": result.get("latency_ms"),
        "message":    result.get("message", ""),
    }).encode())


def _serve_amr_write(handler):
    plc = _plc(handler)
    if plc is None:
        return
    try:
        length = int(handler.headers.get("Content-Length", 0))
        if length > _serve.MAX_POST_BYTES:
            handler._json_response(413, b'{"error":"Request too large"}')
            return
        data = json.loads(handler.rfile.read(length) if length else b"{}")
    except Exception:
        handler._json_response(400, b'{"error":"invalid JSON"}')
        return
    if data.get("reg") is None or data.get("value") is None:
        handler._json_response(400, b'{"error":"reg and value required"}')
        return
    result = plc.amr_write(data["reg"], data["value"])
    ok = result.get("connected") and result.get("success")
    handler._json_response(200 if ok else 503, json.dumps(result).encode())


def _serve_hmi_screens(handler):
    plc = _plc(handler)
    if plc is None:
        return
    handler._json_response(200, json.dumps(plc.hmi_screens_meta()).encode())


def _serve_hmi_read(handler):
    plc = _plc(handler)
    if plc is None:
        return
    q = parse_qs(urlparse(handler.path).query)
    screen = (q.get("screen") or [""])[0]
    if not screen:
        handler._json_response(400, b'{"error":"screen query param required"}')
        return
    out = plc.read_hmi_screen(screen)
    handler._json_response(404 if out.get("error") else 200, json.dumps(out).encode())


def _read_json_body(handler):
    """Parse a JSON POST body; sends the error response and returns None on failure."""
    try:
        length = int(handler.headers.get("Content-Length", 0))
        if length > _serve.MAX_POST_BYTES:
            handler._json_response(413, b'{"error":"Request too large"}')
            return None
        return json.loads(handler.rfile.read(length) if length else b"{}")
    except Exception:
        handler._json_response(400, b'{"error":"invalid JSON"}')
        return None


def _serve_hmi_press(handler):
    """Pulse a momentary axis/motor pushbutton. Body: {block, button}."""
    plc = _plc(handler)
    if plc is None:
        return
    data = _read_json_body(handler)
    if data is None:
        return
    block, button = data.get("block"), data.get("button")
    if not block or not button:
        handler._json_response(400, b'{"error":"block and button required"}')
        return
    result = plc.press_button(block, button)
    ok = result.get("connected") and result.get("success")
    handler._json_response(200 if ok else 400 if result.get("connected") else 503,
                           json.dumps(result).encode())


def _serve_hmi_jog(handler):
    """Hold-to-jog a continuous-motion button. Body: {block, button, action}."""
    plc = _plc(handler)
    if plc is None:
        return
    data = _read_json_body(handler)
    if data is None:
        return
    block, button = data.get("block"), data.get("button")
    action = data.get("action", "start")
    if not block or not button:
        handler._json_response(400, b'{"error":"block and button required"}')
        return
    result = plc.jog(block, button, action)
    ok = result.get("connected") and result.get("success")
    handler._json_response(200 if ok else 400 if result.get("connected") else 503,
                           json.dumps(result).encode())


def _jog_deadman_loop():
    """Clear a held jog whose browser refresh has lapsed (tab closed / connection
    lost). Cheap: no PLC I/O unless a jog is actually pending."""
    while True:
        time.sleep(0.1)   # 10 Hz — deadman resolves within ~_JOG_DEADMAN of last contact
        plc = _serve.Handler.plc
        if plc is not None:
            try:
                plc.jog_deadman()
            except Exception as exc:
                _serve.log.warning("[hmi_jog] deadman error: %s", exc)


def _amr_state_loop():
    """Write %MW5112 = 2 (Moving) / 1 (Stationary) on every movement-state
    change, derived from the dashboard's velocity state. Waits for main() to
    build the PlcClient; no-ops forever on a chassis without a PLC."""
    moving_prev = None
    while True:
        time.sleep(0.15)   # 6-7 Hz checks — fast enough, low overhead

        plc = _serve.Handler.plc
        if plc is None:
            continue

        vel = _serve.TELEM.vel
        with vel.lock:
            lx, az, active, last_t = vel.lin, vel.ang, vel.active, vel.last

        moving = (active
                  and (abs(lx) > 1e-4 or abs(az) > 1e-4)
                  and (time.monotonic() - last_t) < vel.timeout)

        if moving == moving_prev:
            continue   # no state change — skip the Modbus write
        moving_prev = moving

        result = plc.amr_set_moving(moving)
        ok = result.get("connected") and result.get("success")
        _serve.log.info("[amr_plc] %%MW5112 = %d (%s)  %s",
                        2 if moving else 1,
                        "Moving" if moving else "Stationary",
                        "OK" if ok else f"FAIL: {result.get('message')}")


def _register():
    _serve.Handler.INDEX_FILE = "plc_combined.html"
    _serve.Handler.add_route("GET",  "/api/amr/poll",  _serve_amr_poll)
    _serve.Handler.add_route("GET",  "/api/amr/ping",  _serve_amr_ping)
    _serve.Handler.add_route("POST", "/api/amr/write", _serve_amr_write)
    _serve.Handler.add_route("GET",  "/api/hmi/screens", _serve_hmi_screens)
    _serve.Handler.add_route("GET",  "/api/hmi/read",    _serve_hmi_read)
    _serve.Handler.add_route("POST", "/api/hmi/press",   _serve_hmi_press)
    _serve.Handler.add_route("POST", "/api/hmi/jog",     _serve_hmi_jog)


def main():
    _register()
    threading.Thread(target=_amr_state_loop, daemon=True, name="amr-state").start()
    threading.Thread(target=_jog_deadman_loop, daemon=True, name="hmi-jog-deadman").start()
    _serve.main()


if __name__ == "__main__":
    main()
