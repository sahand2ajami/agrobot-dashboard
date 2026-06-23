"""AMR ↔ PLC handshake register dashboard server.

Register layout
───────────────
  PLC → AMR  (read via FC04 — what the PLC tells us):
    %MW5100  Auger status   bit 0 Sequence Start Handshake
                            bit 1 Clear of Ground
                            bit 2 Cycle Complete
    %MW5101  Planter status bit 0 Sequence Start Handshake
                            bit 1 Clear of Ground
                            bit 2 Cycle Complete

  AMR → PLC  (write via FC06 — what we tell the PLC):
    %MW5110  Auger command  bit 0 Auger Start Sequence   (value 1 = active)
    %MW5111  Planter cmd    bit 0 Planter Start Sequence (value 1 = active)
    %MW5112  AMR state      bit 0 Stationary (value 1)
                            bit 1 Moving     (value 2)

FEnet offsets (default XG5000 Modbus Settings):
  FC04 input register N  = %MW(1000 + N)   → %MW5100 = FC04 reg 4100
  FC06 holding register N = %MW(5000 + N)  → %MW5110 = FC06 reg 110

Architecture:
    Browser ──REST──► amr_plc_serve.py :8768 ──Modbus TCP──► LS Electric PLC :502

Usage:
    python3 dashboard/amr_plc_serve.py [--port 8768] [--plc-host 192.168.1.2]
"""

import argparse
import json
import logging
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from urllib.parse import urlparse

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%Y-%m-%dT%H:%M:%S",
)
log = logging.getLogger("amr_plc")

# FEnet address → Modbus register offsets
READ_OFFSET  = 1000   # FC04 reg N reads %MW(1000+N)
WRITE_OFFSET = 5000   # FC06 reg N writes %MW(5000+N)

# One contiguous FC04 read covers the entire handshake region:
# %MW5100–5112 (13 registers; 5102–5109 are unused but read through)
ALL_BASE  = 5100
ALL_COUNT = 13        # covers %MW5100 through %MW5112

# Writable registers (AMR → PLC)
WRITABLE = {5110, 5111, 5112}


class ModbusHelper:
    """Thread-safe Modbus TCP client.  pymodbus is lazily imported."""

    # Short connect timeout + retry backoff so an unreachable PLC reports "offline"
    # almost instantly instead of blocking each poll for the full TCP timeout. A long
    # block made /api/amr/poll exceed the browser's fetch timeout → false "Server
    # unreachable" flapping whenever the PLC was down/unplugged.
    CONNECT_BACKOFF = 3.0   # seconds to wait between connect attempts after a failure

    def __init__(self, host, port, timeout=1.0):
        self.host    = host
        self.port    = int(port)
        self.timeout = float(timeout)
        self._lock   = threading.Lock()
        self._client = None
        self._import_error = None
        self._next_try = 0.0   # monotonic time; skip connect attempts until then

    def _ensure(self):
        if self._client is not None:
            return self._client
        if self._import_error:
            return None
        # Between retries, report unreachable instantly (no blocking connect).
        if time.monotonic() < self._next_try:
            return None
        try:
            from pymodbus.client import ModbusTcpClient
        except Exception as exc:
            self._import_error = str(exc)
            log.warning("pymodbus unavailable: %s", exc)
            return None
        c = ModbusTcpClient(self.host, port=self.port, timeout=self.timeout)
        if not c.connect():
            try:
                c.close()
            except Exception:
                pass
            self._next_try = time.monotonic() + self.CONNECT_BACKOFF
            return None
        self._client = c
        log.info("Modbus connected → %s:%d", self.host, self.port)
        return c

    def _reset(self):
        """Drop socket; next call reconnects. Caller holds _lock."""
        if self._client:
            try:
                self._client.close()
            except Exception:
                pass
        self._client = None

    def read_words(self, plc_base, count):
        """FC04 read `count` words starting at %MW{plc_base}.
        Returns (list[int], None) or (None, error_str).
        """
        reg = plc_base - READ_OFFSET
        if reg < 0:
            return None, f"%MW{plc_base} is below FC04 read base %MW{READ_OFFSET}"
        with self._lock:
            c = self._ensure()
            if c is None:
                return None, self._import_error or f"{self.host}:{self.port} unreachable"
            try:
                res = c.read_input_registers(reg, count=count)
                if res.isError():
                    return None, f"FC04 error at reg {reg}: {res}"
                return list(res.registers), None
            except ConnectionResetError:
                self._reset()
                return None, "Connection reset — will reconnect"
            except Exception as exc:
                self._reset()
                return None, str(exc)

    def write_word(self, plc_addr, value):
        """FC06 write one word to %MW{plc_addr}.
        Returns (True, msg) or (False, msg).
        """
        if plc_addr not in WRITABLE:
            return False, f"%MW{plc_addr} is not a writable handshake register"
        reg = plc_addr - WRITE_OFFSET
        if reg < 0:
            return False, f"%MW{plc_addr} is below FC06 write base %MW{WRITE_OFFSET}"
        with self._lock:
            c = self._ensure()
            if c is None:
                return False, self._import_error or f"{self.host}:{self.port} unreachable"
            try:
                res = c.write_register(reg, int(value) & 0xFFFF)
                if res.isError():
                    return False, f"FC06 error at reg {reg}: {res}"
                return True, f"wrote %MW{plc_addr} = {value}  (FC06 reg {reg})"
            except ConnectionResetError:
                self._reset()
                return False, "Connection reset"
            except Exception as exc:
                self._reset()
                return False, str(exc)

    def poll(self):
        """Read entire handshake region in one FC04 call.
        Returns dict with mw5100–5101 (PLC→AMR) and mw5110–5112 (AMR→PLC readback).
        """
        t0 = time.perf_counter()
        vals, err = self.read_words(ALL_BASE, ALL_COUNT)
        latency_ms = round((time.perf_counter() - t0) * 1000, 1)
        connected = vals is not None

        result = {
            "connected":  connected,
            "latency_ms": latency_ms if connected else None,
            "host":       self.host,
            "port":       self.port,
            "error":      err,
        }

        if vals is not None:
            # offsets: %MW5100=vals[0], %MW5101=vals[1], %MW5110=vals[10], etc.
            result["mw5100"] = vals[0]
            result["mw5101"] = vals[1]
            result["mw5110"] = vals[10]
            result["mw5111"] = vals[11]
            result["mw5112"] = vals[12]
        else:
            for key in ("mw5100", "mw5101", "mw5110", "mw5111", "mw5112"):
                result[key] = None

        return result

    def close(self):
        with self._lock:
            self._reset()


class Handler(BaseHTTPRequestHandler):
    mb: ModbusHelper = None

    def log_message(self, fmt, *args):
        pass

    def _json(self, code, data):
        body = json.dumps(data).encode()
        try:
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _html(self, filename):
        path = os.path.join(os.path.dirname(__file__), filename)
        try:
            with open(path, "rb") as f:
                body = f.read()
        except FileNotFoundError:
            self.send_error(404, f"{filename} not found")
            return
        try:
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _body(self):
        length = int(self.headers.get("Content-Length", 0))
        raw = self.rfile.read(length) if length else b"{}"
        try:
            return json.loads(raw)
        except Exception:
            return {}

    def do_GET(self):
        path = urlparse(self.path).path
        if path in ("/", "/index.html", "/amr_plc.html"):
            self._html("amr_plc.html")
        elif path == "/api/amr/poll":
            self._json(200, self.mb.poll())
        elif path == "/api/amr/ping":
            t0 = time.perf_counter()
            words, err = self.mb.read_words(ALL_BASE, 1)
            ms = round((time.perf_counter() - t0) * 1000, 1)
            self._json(200, {
                "connected":  words is not None,
                "latency_ms": ms if words is not None else None,
                "message":    "OK" if words is not None else err,
            })
        else:
            self.send_error(404)

    def do_POST(self):
        path = urlparse(self.path).path
        if path == "/api/amr/write":
            data = self._body()
            reg  = data.get("reg")
            val  = data.get("value")

            if reg is None or val is None:
                self._json(400, {"error": "'reg' and 'value' are required"})
                return
            try:
                reg = int(reg)
                val = int(val) & 0xFFFF
            except (TypeError, ValueError):
                self._json(400, {"error": "'reg' and 'value' must be integers"})
                return

            ok, msg = self.mb.write_word(reg, val)

            # Read back to confirm the write landed
            rb, _ = self.mb.read_words(reg, 1)
            readback = rb[0] if rb else None

            self._json(200 if ok else 503, {
                "success":       ok,
                "message":       msg,
                "reg":           f"%MW{reg}",
                "value_written": val,
                "readback":      readback,
                "confirmed":     (readback == val) if readback is not None else None,
            })
        else:
            self.send_error(404)


def main():
    parser = argparse.ArgumentParser(description="AMR ↔ PLC handshake dashboard")
    parser.add_argument("--port",     type=int, default=8768)
    parser.add_argument("--plc-host", default="192.168.1.2")
    parser.add_argument("--plc-port", type=int, default=502)
    args = parser.parse_args()

    mb = ModbusHelper(host=args.plc_host, port=args.plc_port)
    Handler.mb = mb

    server = HTTPServer(("", args.port), Handler)
    log.info("AMR-PLC dashboard → http://localhost:%d  (PLC at %s:%d)",
             args.port, args.plc_host, args.plc_port)
    log.info("FC04 read offset: %%MW%d  |  FC06 write offset: %%MW%d",
             READ_OFFSET, WRITE_OFFSET)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        mb.close()
        log.info("AMR-PLC server stopped.")


if __name__ == "__main__":
    main()
