"""Direct Modbus TCP client to the LS Electric PLC (no gRPC gateway).

The dashboard's HTTP server (serve.py) owns one PlcClient and exposes its operations
as REST endpoints. The browser can't speak Modbus, so serve.py is the Modbus client:

    Browser --REST--> serve.py --Modbus TCP--> LS Electric PLC @ host:port (502)

This used to relay through a separate gRPC gateway process (serve.py --gRPC--> gateway
--Modbus TCP--> PLC). The gateway ran on the same Jetson serving only this dashboard, so
the gRPC hop was pure overhead plus a protobuf-version mismatch that stopped it booting.
The register map, command bit-values, pulse pattern and the JSON shapes below are ported
verbatim from that gateway (config.LS_CONF + network/grpc_server.py), so serve.py and the
whole UI are unchanged — only the transport moved from gRPC to Modbus.

Design points (mirroring chassis.py / the camera threads):
- ``pymodbus`` is imported *lazily* inside the client, so this module imports fine — and
  ``pytest tests/`` runs — even where pymodbus is absent. Nothing pulls in hardware at
  import time.
- Every public method returns a plain ``dict`` and never raises into the HTTP handler: on
  an unreachable PLC it returns ``{"connected": False, "success": False, "message": ...}``.
  A short socket timeout keeps a dead PLC from stalling a request thread.
- ``success: true`` means the Modbus write landed, *not* that the machine moved (the PLC
  ladder gates real motion on Auto-mode/safety/enable). The UI confirms real completion by
  polling get_sequence_detail and watching ``*_in_cycle``.
"""

import logging
import threading
import time

log = logging.getLogger("plc_client")

# ── Command allow-lists ──────────────────────────────────────────────────────
# Validated before forwarding so the dashboard rejects garbage with a 400 rather
# than pulsing an unknown register. Mirrors the Gateway's accepted values
# (docs/sequence_api.md); the Gateway uppercases, so we compare upper-cased.
SEQUENCE_COMMANDS = frozenset({"START", "STOP"})

MACHINE_COMMANDS = frozenset({
    "START", "STOP", "HOME_ALL", "FAULT_RESET", "SET_AUTO", "SET_MANUAL",
    "ENABLE_AUGER", "DISABLE_AUGER", "ENABLE_PLANTER", "DISABLE_PLANTER",
    "RESET_AUGER", "RESET_PLANTER", "ENABLE_ROBOT", "DISABLE_ROBOT",
    "ENABLE_AMR", "DISABLE_AMR",
})

ROBOT_COMMANDS = frozenset({
    "HOME", "PAUSE", "CONTINUE", "MOTORS_ON", "MOTORS_OFF",
    "START", "STOP", "RESET", "SHUTDOWN",
})

_UNAVAILABLE = "PLC client unavailable"

# ── PLC tag / register reference ─────────────────────────────────────────────
# Curated map of the PLC symbols this integration actually touches, surfaced by
# GET /api/plc/tags for the dashboard's "PLC Reference" panel. For each *read* group,
# ``source`` names the read endpoint that returns its values (status / sequence /
# auger_motor) and every field ``key`` matches a JSON key in that response, so the UI
# can show live values next to the mapping. Each *write* group lists the command set
# that pulses its bits. Structs/addresses are verified against the PLC global symbol
# table (docs/plc/GTS_Tree_Planter_symbols.csv); the exact bit index within each shared
# word and the Modbus address base are the two live-PLC open items (docs/plc/README.md).
PLC_TAG_MAP = {
    "read": [
        {
            "symbol": "HMI_IND", "address": "%MW1000", "type": "ud_HMI_IND",
            "rpc": "GetMachineStatus", "api": "/api/plc/status", "source": "status",
            "desc": "Machine indicators — E-stop, safety gate, fault, mode and per-subsystem enables.",
            "fields": [
                {"key": "estop_ok",        "label": "E-stop OK",       "kind": "bool"},
                {"key": "gate_ok",         "label": "Safety gate OK",  "kind": "bool"},
                {"key": "faulted",         "label": "Machine faulted", "kind": "bool"},
                {"key": "mode_auto",       "label": "Auto mode",       "kind": "bool"},
                {"key": "mode_manual",     "label": "Manual mode",     "kind": "bool"},
                {"key": "auger_enabled",   "label": "Auger enabled",   "kind": "bool"},
                {"key": "planter_enabled", "label": "Planter enabled", "kind": "bool"},
                {"key": "robot_enabled",   "label": "Robot enabled",   "kind": "bool"},
                {"key": "amr_enabled",     "label": "AMR enabled",     "kind": "bool"},
            ],
        },
        {
            "symbol": "AugerSeq", "address": "%MW2700", "type": "ud_sequence",
            "rpc": "GetSequenceDetail", "api": "/api/plc/sequence", "source": "sequence",
            "desc": "Auger planting-sequence state machine.",
            "fields": [
                {"key": "auger_home",        "label": "At home",     "kind": "bool"},
                {"key": "auger_setup_ok",    "label": "Setup OK",    "kind": "bool"},
                {"key": "auger_ok_to_start", "label": "OK to start", "kind": "bool"},
                {"key": "auger_enabled",     "label": "Enabled",     "kind": "bool"},
                {"key": "auger_in_cycle",    "label": "In cycle",    "kind": "bool"},
                {"key": "auger_complete",    "label": "Complete",    "kind": "bool"},
                {"key": "auger_step",        "label": "Step #",      "kind": "uint"},
            ],
        },
        {
            "symbol": "PlanterSeq", "address": "%MW2800", "type": "ud_sequence",
            "rpc": "GetSequenceDetail", "api": "/api/plc/sequence", "source": "sequence",
            "desc": "Planter planting-sequence state machine.",
            "fields": [
                {"key": "planter_home",        "label": "At home",     "kind": "bool"},
                {"key": "planter_setup_ok",    "label": "Setup OK",    "kind": "bool"},
                {"key": "planter_ok_to_start", "label": "OK to start", "kind": "bool"},
                {"key": "planter_enabled",     "label": "Enabled",     "kind": "bool"},
                {"key": "planter_in_cycle",    "label": "In cycle",    "kind": "bool"},
                {"key": "planter_complete",    "label": "Complete",    "kind": "bool"},
                {"key": "planter_step",        "label": "Step #",      "kind": "uint"},
            ],
        },
        {
            "symbol": "HMI_IND_Auger", "address": "%MW2500", "type": "ud_HMI_MotorIND",
            "rpc": "GetAugerMotorStatus", "api": "/api/plc/auger_motor", "source": "auger_motor",
            "desc": "Auger VFD telemetry (velocities in raw drive units, 0–4096 typical).",
            "fields": [
                {"key": "running",         "label": "Running",          "kind": "bool"},
                {"key": "fwd_direction",   "label": "Forward dir",      "kind": "bool"},
                {"key": "faulted",         "label": "Drive faulted",    "kind": "bool"},
                {"key": "velocity_target", "label": "Vel target (raw)", "kind": "uint"},
                {"key": "velocity_actual", "label": "Vel actual (raw)", "kind": "uint"},
            ],
        },
    ],
    "write": [
        {
            "symbol": "HMI_PB", "address": "%MW5000", "type": "ud_HMI_PB",
            "rpc": "MachineCommand", "api": "/api/plc/machine",
            "desc": "Machine-level pushbuttons — mode, fault-reset, home, and subsystem enables.",
            "commands": sorted(MACHINE_COMMANDS),
        },
        {
            "symbol": "HMI_PB_Robot", "address": "%MW6200", "type": "ud_HMI_RobotPB",
            "rpc": "ControlRobot", "api": "/api/plc/robot",
            "desc": "Robot-arm manipulator pushbuttons.",
            "commands": sorted(ROBOT_COMMANDS),
        },
        {
            "symbol": "AMR_2_PLC[0]", "address": "%MW100", "type": "WORD (bit 0 = AMR_2_PLC[0].0)",
            "rpc": "—", "api": "/api/plc/auger",
            "desc": "Auger button → latches AMR_2_PLC[0].0 (write word 1/0, toggles). AMR→PLC handshake.",
            "commands": sorted(SEQUENCE_COMMANDS),
        },
        {
            "symbol": "AMR_2_PLC[1]", "address": "%MW101", "type": "WORD (bit 0 = AMR_2_PLC[1].0)",
            "rpc": "—", "api": "/api/plc/planter",
            "desc": "Planter button → latches AMR_2_PLC[1].0 (write word 1/0, toggles). AMR→PLC handshake.",
            "commands": sorted(SEQUENCE_COMMANDS),
        },
    ],
    "reserved": [
        {"symbol": "PLC_2_AMR", "address": "%MW200", "type": "ARRAY[0..9] OF WORD",
         "desc": "Reserved PLC→AMR handshake — declared in the PLC, not referenced in this ladder build."},
    ],
    "notes": {
        "verified": "Structs and %MW/%MX addresses verified against the PLC global symbol table.",
        "open_items": [
            "Exact bit index within each shared word (e.g. SET_AUTO / START / ENABLE_* in HMI_PB) is packed in the binary UDT — bench-confirm in XG5000.",
            "Modbus address base: the PLC's Modbus-TCP server must expose %MW/%MX at offset 0 (so %MW100 bit 0 = coil 1600) — bench-confirm.",
        ],
        "amr_bits": "The auger/planter buttons drive AMR_2_PLC[0].0 / [1].0 directly over Modbus TCP (no gRPC gateway). AugerSeq/PlanterSeq reads above are unchanged.",
    },
}


def symbol_roles():
    """Map PLC symbol name → role ('read' / 'write' / 'reserved') for annotating the
    full symbol table. Symbols absent from the map are used by the PLC but not by this
    integration."""
    roles = {}
    for grp in PLC_TAG_MAP["read"]:
        roles[grp["symbol"]] = "read"
    for grp in PLC_TAG_MAP["write"]:
        roles[grp["symbol"]] = "write"
    for grp in PLC_TAG_MAP["reserved"]:
        roles[grp["symbol"]] = "reserved"
    return roles



# ── FEnet Modbus address mapping ─────────────────────────────────────────────
# Configured in XG5000 → Online → Standard Settings → FEnet → Modbus Settings.
# The FEnet exposes its areas at non-zero bases; raw PLC addresses must be offset:
#
#   Word Read  (FC04 input regs):  Modbus reg = plc_addr - 1000  (base %MW1000)
#   Word Write (FC06/FC16):        Modbus reg = plc_addr - 5000  (base %MW5000)
#   Bit  Read  (FC02 disc inputs): Modbus reg = plc_addr         (base %MX0, no offset)
#   Bit  Write (FC05/FC15 coils):  Modbus reg = plc_addr - 1000  (base %MX1000)
#
# Critical: reads use FC04 (input registers), NOT FC03 (holding registers). FC03
# returns 0 for all addresses — confirmed 2026-06-17 bench session.
_FENET_READ_WORD_BASE  = 1000   # FC04 input reg 0  = PLC %MW1000
_FENET_WRITE_WORD_BASE = 5000   # FC06 reg 0         = PLC %MW5000
_FENET_WRITE_COIL_BASE = 1000   # FC05 coil 0        = PLC %MX1000

# HMI banner string addresses — PLC writes the active fault/warning text here.
# Fault_Result STRING @ %MW1014  → FC04 reg 14  (= 1014 - _FENET_READ_WORD_BASE)
# Warning_Result STRING @ %MW1030 → FC04 reg 30  (= 1030 - _FENET_READ_WORD_BASE)
_BANNER_CHARS = 32   # STRING[32]: 2 ASCII chars per 16-bit word, null-padded
_BANNER_WORDS = 16   # 32 chars / 2
_FAULT_WORD   = 1014
_WARNING_WORD = 1030

# ── PLC register map + command tables (ported from the gateway) ──────────────
# Logical name → LS Electric device address. Only the symbols this dashboard touches;
# the full table lives in docs/plc/. %MX<n> = M-area bit, %MW<n> = M-area word.
_REG = {
    # ── auger / planter activation words — AMR_2_PLC handshake (AMR→PLC @ %MW100) ──
    # AMR_2_PLC is ARRAY[0..9] OF WORD; element k = %MW(100+k). We (the AMR) own this array,
    # so we write the whole WORD register (FC16): value 1 sets bit 0 (= AMR_2_PLC[k].0), 0
    # clears it. Word/register access works where individual-coil bit writes don't on this PLC.
    "AUGER_AMR_WORD":   "%MW100",   # AMR_2_PLC[0]  — write 1 → bit 0 (AMR_2_PLC[0].0) set
    "PLANTER_AMR_WORD": "%MW101",   # AMR_2_PLC[1]  — write 1 → bit 0 (AMR_2_PLC[1].0) set
    # ── machine / robot pushbutton words (writes) — pulsed: write value → 100 ms → 0 ──
    "HMI_PB_MachineCtrl":  "%MW5000",   # machine PB word (bit values below)
    "HMI_PB_MachineCtrl2": "%MW5001",   # machine PB word 2 (ResetPlanterSeq/robot/AMR)
    "ROBOT_PB_CMD":        "%MW6200",   # HMI_PB_Robot word
    # ── machine status (HMI_IND @ %MW1000) ──
    "IND_MODE_STATUS":     "%MW1000",   # 0/1 = Manual, 2 = Auto
    "IND_ESTOP_OK_FL":     "%MX16032",
    "IND_GATE_OK":         "%MX16040",
    "IND_FAULTED":         "%MX16208",
    "IND_AUGER_ENABLED":   "%MX16044",
    "IND_PLANTER_ENABLED": "%MX16045",
    "IND_ROBOT_ENABLED":   "%MX16046",
    "IND_AMR_ENABLED":     "%MX16047",
    # ── AugerSeq (@ %MW2700 → bits %MX43200+) ──
    "AUGER_HOME":     "%MX43200",
    "AUGER_SETUP_OK": "%MX43201",
    "AUGER_OK_START": "%MX43202",
    "AUGER_ENABLED":  "%MX43203",
    "AUGER_IN_CYCLE": "%MX43204",
    "AUGER_COMPLETE": "%MX43205",
    "AUGER_STEP":     "%MW2701",
    # ── PlanterSeq (@ %MW2800 → bits %MX44800+) ──
    "PLANTER_HOME":     "%MX44800",
    "PLANTER_SETUP_OK": "%MX44801",
    "PLANTER_OK_START": "%MX44802",
    "PLANTER_ENABLED":  "%MX44803",
    "PLANTER_IN_CYCLE": "%MX44804",
    "PLANTER_COMPLETE": "%MX44805",
    "PLANTER_STEP":     "%MW2801",
    # ── Auger motor (HMI_IND_Auger @ %MW2500) ──
    "AUGER_MOTOR_VEL_TARGET": "%MW2500",
    "AUGER_MOTOR_VEL_ACTUAL": "%MW2501",
    "AUGER_MOTOR_RUN":        "%MX40032",
    "AUGER_MOTOR_FWD":        "%MX40033",
    "AUGER_MOTOR_FAULTED":    "%MX40034",
}

# MachineCommand → (pushbutton word, bit value). See config.LS_CONF %MW5000/%MW5001 bit maps.
_MACHINE_CMD_MAP = {
    "START":           ("HMI_PB_MachineCtrl",  64),     # bit 6  StartPVPB
    "STOP":            ("HMI_PB_MachineCtrl",  128),    # bit 7  StopPVPB
    "HOME_ALL":        ("HMI_PB_MachineCtrl",  16),     # bit 4  HomeAllPVPB
    "FAULT_RESET":     ("HMI_PB_MachineCtrl",  2),      # bit 1  FaultResetPVPB
    "SET_AUTO":        ("HMI_PB_MachineCtrl",  1),      # bit 0  AutoPVPB
    "SET_MANUAL":      ("HMI_PB_MachineCtrl",  32),     # bit 5  ManualPVPB
    "ENABLE_AUGER":    ("HMI_PB_MachineCtrl",  2048),   # bit 11 EnableAugerPVPB
    "DISABLE_AUGER":   ("HMI_PB_MachineCtrl",  4096),   # bit 12 DisableAugerPVPB
    "ENABLE_PLANTER":  ("HMI_PB_MachineCtrl",  8192),   # bit 13 EnablePlanterPVPB
    "DISABLE_PLANTER": ("HMI_PB_MachineCtrl",  16384),  # bit 14 DisablePlanterPVPB
    "RESET_AUGER":     ("HMI_PB_MachineCtrl",  32768),  # bit 15 ResetAugerSeqPVPB
    "RESET_PLANTER":   ("HMI_PB_MachineCtrl2", 1),      # bit 0  ResetPlanterSeqPVPB
    "ENABLE_ROBOT":    ("HMI_PB_MachineCtrl2", 8192),   # bit 13 EnableRobotPVPB
    "DISABLE_ROBOT":   ("HMI_PB_MachineCtrl2", 16384),  # bit 14 DisableRobotPVPB
    "ENABLE_AMR":      ("HMI_PB_MachineCtrl2", 2048),   # bit 11 EnableAMRPVPB
    "DISABLE_AMR":     ("HMI_PB_MachineCtrl2", 4096),   # bit 12 DisableAMRPVPB
}

# ControlRobot → ROBOT_PB_CMD (%MW6200) bit value.
_ROBOT_CMD_MAP = {
    "HOME": 1, "PAUSE": 2, "CONTINUE": 4, "MOTORS_ON": 8, "MOTORS_OFF": 16,
    "START": 32, "STOP": 64, "SHUTDOWN": 128, "RESET": 256,
}

class PlcClient:
    """Modbus TCP client to the LS Electric PLC. host/port point at the PLC's Modbus
    server (192.168.1.2:502 for agrobot). One socket, serialized by ``_lock`` so a 100 ms
    pulse and concurrent status polls don't interleave on the wire."""

    def __init__(self, host="127.0.0.1", port=502, timeout=2.0):
        self.host    = host
        self.port    = int(port)
        self.timeout = float(timeout)          # Modbus socket timeout (s)
        self._lock   = threading.Lock()
        self._client = None                     # pymodbus ModbusTcpClient (lazy)
        self._import_error = None               # set once if pymodbus is unavailable

    @property
    def target(self):
        return f"{self.host}:{self.port}"

    # -- connection lifecycle (caller holds _lock) ----------------------------
    def _ensure_client(self):
        """Lazily import pymodbus and open the TCP connection. Returns the client, or
        None if pymodbus is missing or the PLC is unreachable."""
        if self._client is not None:
            return self._client
        if self._import_error is not None:
            return None
        try:
            from pymodbus.client import ModbusTcpClient
        except Exception as exc:                # pymodbus not installed
            self._import_error = str(exc)
            log.warning("PLC client disabled — pymodbus unavailable: %s", exc)
            return None
        client = ModbusTcpClient(self.host, port=self.port, timeout=self.timeout)
        if not client.connect():
            try:
                client.close()
            except Exception:
                pass
            log.warning("PLC Modbus connect failed → %s", self.target)
            return None
        self._client = client
        log.info("PLC Modbus client connected → %s", self.target)
        return self._client

    def _reset(self):
        """Drop the socket so the next call reconnects (e.g. PLC power-cycled). Caller
        holds _lock."""
        if self._client is not None:
            try:
                self._client.close()
            except Exception:
                pass
        self._client = None

    # -- raw device access (caller holds _lock; client is live) ---------------
    @staticmethod
    def _parse(device):
        """%MX<n> → ('coil', n);  %MW<n> → ('hr', n).  Raises on anything else."""
        raw = device.strip().upper().lstrip("%")
        if raw.startswith("MX") and raw[2:].isdigit():
            return "coil", int(raw[2:])
        if raw.startswith("MW") and raw[2:].isdigit():
            return "hr", int(raw[2:])
        raise ValueError(f"unsupported device '{device}'")

    def _read(self, name):
        """Read a logical var → bool (bit) / int (word). None on Modbus error;
        raises on transport exception (so _op resets + reconnects on next call)."""
        func, addr = self._parse(_REG[name])
        if func == "coil":
            # FC02 discrete inputs — FEnet bit read area, no offset (%MX0 = reg 0)
            res = self._client.read_discrete_inputs(addr, count=1)
            return None if res.isError() else bool(res.bits[0])
        # FC04 input registers — FEnet word read area base = %MW1000
        modbus_reg = addr - _FENET_READ_WORD_BASE
        if modbus_reg < 0:
            log.warning("_read: %s (%s) below FEnet read area (min %%MW%d)",
                        name, _REG[name], _FENET_READ_WORD_BASE)
            return None
        res = self._client.read_input_registers(modbus_reg, count=1)
        return None if res.isError() else int(res.registers[0])

    def _write(self, name, value):
        """Write a logical var. Bit (%MX) → FC05 write_coil; word (%MW) → FC06
        write_register. Returns True on success, False if address is out of range."""
        func, addr = self._parse(_REG[name])
        if func == "coil":
            modbus_coil = addr - _FENET_WRITE_COIL_BASE
            res = self._client.write_coil(modbus_coil, bool(value))
        else:
            modbus_reg = addr - _FENET_WRITE_WORD_BASE
            if modbus_reg < 0:
                log.warning("_write: %s (%s) below FEnet write area (min %%MW%d) — "
                            "PLC engineer must lower FEnet Word Write Area to %%MW0",
                            name, _REG[name], _FENET_WRITE_WORD_BASE)
                return False
            res = self._client.write_register(modbus_reg, int(value) & 0xFFFF)
        return not res.isError()

    def _pulse(self, name, press_val=1):
        """Momentary pushbutton: write press_val → hold ≥1 PLC scan (100 ms) → write 0."""
        if not self._write(name, press_val):
            return False, f"PLC write failed for {name}"
        time.sleep(0.1)
        self._write(name, 0)
        return True, f"pulsed {name}: {press_val} → 0"

    def _read_words_raw(self, plc_addr, count):
        """Read `count` consecutive FC04 input registers starting at %MW<plc_addr>.
        Returns a list of unsigned 16-bit ints, or None on Modbus error."""
        modbus_reg = plc_addr - _FENET_READ_WORD_BASE
        if modbus_reg < 0:
            return None
        res = self._client.read_input_registers(modbus_reg, count=count)
        return None if res.isError() else list(res.registers)

    @staticmethod
    def _decode_banner_string(words):
        """Decode 16-bit words (big-endian: high byte = first char) into an ASCII string."""
        out = []
        for w in words:
            out.append((w >> 8) & 0xFF)
            out.append(w & 0xFF)
        return bytes(out)[:_BANNER_CHARS].split(b'\x00', 1)[0].decode('ascii', 'replace').strip()

    def _machine_status(self):
        """HMI_IND safety/enable/fault/mode snapshot (caller holds _lock; client live)."""
        rb = lambda n: bool(self._read(n))
        mode = int(self._read("IND_MODE_STATUS") or 0)
        return {
            "estop_ok":         rb("IND_ESTOP_OK_FL"),
            "gate_ok":          rb("IND_GATE_OK"),
            "faulted":          rb("IND_FAULTED"),
            "auger_enabled":    rb("IND_AUGER_ENABLED"),
            "planter_enabled":  rb("IND_PLANTER_ENABLED"),
            "robot_enabled":    rb("IND_ROBOT_ENABLED"),
            "amr_enabled":      rb("IND_AMR_ENABLED"),
            "auger_in_cycle":   rb("AUGER_IN_CYCLE"),
            "planter_in_cycle": rb("PLANTER_IN_CYCLE"),
            "mode_auto":        (mode == 2),
            "mode_manual":      (mode in (0, 1)),
        }

    def _op(self, fn):
        """Run fn() under the lock with uniform connect/error handling. fn returns a
        dict of payload fields; _op adds ``connected`` or maps failures to the standard
        unreachable dict. Never raises into the HTTP handler."""
        with self._lock:
            client = self._ensure_client()
            if client is None:
                return {"connected": False, "success": False,
                        "message": f"{_UNAVAILABLE}: {self._import_error or self.target} unreachable"}
            try:
                out = fn()
                out["connected"] = True
                return out
            except Exception as exc:
                self._reset()
                log.warning("PLC operation error: %s", exc)
                return {"connected": False, "success": False, "message": str(exc)}

    # -- write operations -----------------------------------------------------
    # START → write 1 to the AMR_2_PLC word (sets bit 0); STOP → write 0 (clears it).
    # No read-first toggle: the JS caller already decides START vs STOP based on
    # auger_in_cycle, so writing unconditionally is correct and avoids writing the
    # wrong value when the register was left non-zero from a prior session.
    def control_auger(self, command):
        new = 0 if (command or "").upper() == "STOP" else 1
        def fn():
            ok = self._write("AUGER_AMR_WORD", new)
            log.debug("Auger: write %MW100=%d ok=%s", new, ok)
            return {"success": ok, "message": f"Auger: AMR_2_PLC[0].0 → {new}",
                    "auger_active": bool(new), "planter_active": False}
        return self._op(fn)

    def control_planter(self, command):
        new = 0 if (command or "").upper() == "STOP" else 1
        def fn():
            ok = self._write("PLANTER_AMR_WORD", new)
            log.debug("Planter: write %MW101=%d ok=%s", new, ok)
            return {"success": ok, "message": f"Planter: AMR_2_PLC[1].0 → {new}",
                    "auger_active": False, "planter_active": bool(new)}
        return self._op(fn)

    def control_both(self, command):
        new = 0 if (command or "").upper() == "STOP" else 1
        def fn():
            ok_a = self._write("AUGER_AMR_WORD", new)
            ok_p = self._write("PLANTER_AMR_WORD", new)
            log.debug("Both: write %MW100/%MW101=%d ok=%s/%s", new, ok_a, ok_p)
            return {"success": (ok_a and ok_p),
                    "message": f"Both: AMR_2_PLC[0].0/[1].0 → {new}",
                    "auger_active": bool(new), "planter_active": bool(new)}
        return self._op(fn)

    def machine_command(self, command):
        cmd = (command or "").upper()
        def fn():
            if cmd not in _MACHINE_CMD_MAP:
                return {"success": False, "message": f"unknown command '{cmd}'"}
            var, bit = _MACHINE_CMD_MAP[cmd]
            ok, detail = self._pulse(var, bit)
            status = self._machine_status()
            status["success"] = ok
            status["message"] = f"{cmd}: {detail}"
            return status
        return self._op(fn)

    def control_robot(self, command):
        cmd = (command or "").upper()
        def fn():
            if cmd not in _ROBOT_CMD_MAP:
                return {"success": False, "message": f"unknown robot command '{cmd}'"}
            ok, detail = self._pulse("ROBOT_PB_CMD", _ROBOT_CMD_MAP[cmd])
            status = self._machine_status()
            status["success"] = ok
            status["message"] = f"Robot {cmd}: {detail}"
            return status
        return self._op(fn)

    # -- read operations ------------------------------------------------------
    def get_machine_status(self):
        def fn():
            status = self._machine_status()
            status["success"] = True
            status["message"] = "OK"
            return status
        return self._op(fn)

    def get_sequence_detail(self):
        def fn():
            rb = lambda n: bool(self._read(n))
            ri = lambda n: int(self._read(n) or 0)
            return {
                "auger_home":          rb("AUGER_HOME"),
                "auger_setup_ok":      rb("AUGER_SETUP_OK"),
                "auger_ok_to_start":   rb("AUGER_OK_START"),
                "auger_enabled":       rb("AUGER_ENABLED"),
                "auger_in_cycle":      rb("AUGER_IN_CYCLE"),
                "auger_complete":      rb("AUGER_COMPLETE"),
                "auger_step":          ri("AUGER_STEP"),
                "planter_home":        rb("PLANTER_HOME"),
                "planter_setup_ok":    rb("PLANTER_SETUP_OK"),
                "planter_ok_to_start": rb("PLANTER_OK_START"),
                "planter_enabled":     rb("PLANTER_ENABLED"),
                "planter_in_cycle":    rb("PLANTER_IN_CYCLE"),
                "planter_complete":    rb("PLANTER_COMPLETE"),
                "planter_step":        ri("PLANTER_STEP"),
            }
        return self._op(fn)

    def get_auger_motor_status(self):
        def fn():
            rb = lambda n: bool(self._read(n))
            ri = lambda n: int(self._read(n) or 0)
            return {
                "success": True, "message": "OK",
                "running":         rb("AUGER_MOTOR_RUN"),
                "fwd_direction":   rb("AUGER_MOTOR_FWD"),
                "faulted":         rb("AUGER_MOTOR_FAULTED"),
                "velocity_target": ri("AUGER_MOTOR_VEL_TARGET"),
                "velocity_actual": ri("AUGER_MOTOR_VEL_ACTUAL"),
            }
        return self._op(fn)

    def get_banner(self):
        """Read the PLC's active fault and warning banner strings (Fault_Result @
        %MW1014, Warning_Result @ %MW1030) — the same text the HMI banner shows.
        An empty fault string means nothing is currently faulted."""
        def fn():
            fault_words   = self._read_words_raw(_FAULT_WORD,   _BANNER_WORDS)
            warning_words = self._read_words_raw(_WARNING_WORD, _BANNER_WORDS)
            fault   = self._decode_banner_string(fault_words)   if fault_words   else ''
            warning = self._decode_banner_string(warning_words) if warning_words else ''
            if fault.lower() in ('no fault', ''):
                fault = ''
            return {'success': True, 'message': 'OK', 'fault': fault, 'warning': warning}
        return self._op(fn)

    def ping(self):
        """Lightweight connectivity check — connect and read one register (FC04 reg 0 =
        %MW1000). Returns {connected, latency_ms, host, port}. latency_ms is None when
        unreachable. Callers should poll this every few seconds for a health indicator."""
        t0 = time.perf_counter()
        def fn():
            res = self._client.read_input_registers(0, count=1)  # %MW1000, reg = 1000-1000 = 0
            latency_ms = round((time.perf_counter() - t0) * 1000, 1)
            if res.isError():
                return {"success": False, "latency_ms": latency_ms, "message": "Modbus read error"}
            return {"success": True, "latency_ms": latency_ms, "message": "OK"}
        out = self._op(fn)
        if not out.get("connected"):
            out["latency_ms"] = None
        out["host"] = self.host
        out["port"] = self.port
        return out

    def close(self):
        with self._lock:
            self._reset()
