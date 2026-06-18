"""Standalone PLC HMI server.

Serves the auger/planter HMI on a dedicated port (default 8767), separate from the
main teleoperation dashboard (8766). No ROS required — talks to the PLC directly via
Modbus TCP through plc_client.PlcClient.

Architecture:
    Browser ──REST──► plc_hmi_serve.py :8767 ──Modbus TCP──► LS Electric PLC :502

Usage:
    python3 dashboard/plc_hmi_serve.py [--port 8767] [--plc-host 192.168.1.2]
"""

import argparse
import json
import logging
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from urllib.parse import urlparse

sys.path.insert(0, os.path.dirname(__file__))
import plc_client as plcmod

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%Y-%m-%dT%H:%M:%S",
)
log = logging.getLogger("plc_hmi")

# Jog watchdog: auto-stop if no keepalive arrives within this many seconds.
_JOG_TIMEOUT = 0.6


class _JogController:
    """Press-and-hold jog state with a watchdog that auto-stops on timeout or disconnect."""

    def __init__(self):
        self._lock = threading.Lock()
        self._direction = None       # "up" | "down" | None
        self._last_ping = 0.0
        self._thread = None

    def activate(self, direction):
        with self._lock:
            self._direction = direction
            self._last_ping = time.monotonic()
            if self._thread is None or not self._thread.is_alive():
                self._thread = threading.Thread(target=self._watch, daemon=True)
                self._thread.start()

    def deactivate(self):
        with self._lock:
            self._direction = None

    def keepalive(self, direction):
        with self._lock:
            if self._direction == direction:
                self._last_ping = time.monotonic()

    @property
    def direction(self):
        with self._lock:
            return self._direction

    def _watch(self):
        while True:
            time.sleep(0.1)
            with self._lock:
                if self._direction is None:
                    return
                if time.monotonic() - self._last_ping > _JOG_TIMEOUT:
                    log.info("Jog watchdog: auto-stop (%s timeout)", self._direction)
                    self._direction = None
                    return


_jog = _JogController()


class Handler(BaseHTTPRequestHandler):
    plc: plcmod.PlcClient = None  # set in main()

    def log_message(self, fmt, *args):
        pass  # silence default per-request logging

    # ── helpers ───────────────────────────────────────────────────────────────

    def _json(self, code, data):
        body = json.dumps(data).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _html(self, filename):
        path = os.path.join(os.path.dirname(__file__), filename)
        try:
            with open(path, "rb") as f:
                body = f.read()
        except FileNotFoundError:
            self.send_error(404, f"{filename} not found")
            return
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _body(self):
        length = int(self.headers.get("Content-Length", 0))
        raw = self.rfile.read(length) if length else b"{}"
        try:
            return json.loads(raw)
        except Exception:
            return {}

    # ── GET routes ────────────────────────────────────────────────────────────

    def do_GET(self):
        path = urlparse(self.path).path

        if path in ("/", "/index.html"):
            self._html("plc_hmi.html")

        elif path == "/api/hmi/status":
            self._json(200, self.plc.get_machine_status())

        elif path == "/api/hmi/auger/status":
            motor = self.plc.get_auger_motor_status()
            seq   = self.plc.get_sequence_detail()
            merged = {**motor, **seq}
            merged["connected"] = (
                motor.get("connected", False) and seq.get("connected", False)
            )
            self._json(200, merged)

        else:
            self.send_error(404)

    # ── POST routes ───────────────────────────────────────────────────────────

    def do_POST(self):
        path = urlparse(self.path).path
        data = self._body()

        if path == "/api/hmi/auger/jog":
            direction = (data.get("direction") or "").lower()
            active    = bool(data.get("active", True))

            if direction not in ("up", "down"):
                self._json(400, {"error": "direction must be 'up' or 'down'"})
                return

            if active:
                _jog.activate(direction)
            else:
                _jog.deactivate()

            # Jog register addresses are not yet confirmed with the PLC engineer.
            # Wire them up in plc_client._REG (AUGER_JOG_UP / AUGER_JOG_DOWN)
            # once the %MW addresses are known. The jog watchdog is already running.
            self._json(200, {
                "success":   False,
                "direction": direction,
                "active":    active,
                "message":   (
                    "Jog registers not yet configured. "
                    "Confirm auger jog %MW addresses with PLC engineer, "
                    "then add AUGER_JOG_UP / AUGER_JOG_DOWN to plc_client._REG."
                ),
            })

        else:
            self.send_error(404)


def main():
    parser = argparse.ArgumentParser(description="PLC HMI HTTP server")
    parser.add_argument("--port",     type=int, default=8767, help="HTTP listen port")
    parser.add_argument("--plc-host", default="192.168.1.2",  help="PLC Modbus TCP host")
    parser.add_argument("--plc-port", type=int, default=502,   help="PLC Modbus TCP port")
    args = parser.parse_args()

    plc = plcmod.PlcClient(host=args.plc_host, port=args.plc_port)
    Handler.plc = plc

    server = HTTPServer(("", args.port), Handler)
    log.info("PLC HMI server → http://localhost:%d  (PLC at %s:%d)",
             args.port, args.plc_host, args.plc_port)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        plc.close()
        log.info("PLC HMI stopped.")


if __name__ == "__main__":
    main()
