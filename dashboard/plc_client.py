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
import struct
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
    "ENABLE_AMR", "DISABLE_AMR", "JAW_METHOD_1", "JAW_METHOD_2", "AMR_OK_TO_PLANT",
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
            "symbol": "AMR_CMD_AUGER", "address": "%MW5110", "type": "WORD (bit 0 = Start Sequence)",
            "rpc": "—", "api": "/api/plc/auger",
            "desc": "Auger button → writes %MW5110 bit 0 (1 = start, 0 = clear). AMR→PLC handshake.",
            "commands": sorted(SEQUENCE_COMMANDS),
        },
        {
            "symbol": "AMR_CMD_PLANTER", "address": "%MW5111", "type": "WORD (bit 0 = Start Sequence)",
            "rpc": "—", "api": "/api/plc/planter",
            "desc": "Planter button → writes %MW5111 bit 0 (1 = start, 0 = clear). AMR→PLC handshake.",
            "commands": sorted(SEQUENCE_COMMANDS),
        },
        {
            "symbol": "AMR_STATE", "address": "%MW5112", "type": "WORD (1 = Stationary, 2 = Moving)",
            "rpc": "—", "api": "/api/amr/write",
            "desc": "AMR movement state, auto-written on every moving/stationary transition.",
            "commands": [],
        },
    ],
    "reserved": [
        {"symbol": "AMR_STATUS_AUGER", "address": "%MW5100", "type": "WORD",
         "desc": "PLC→AMR auger status: bit 0 Sequence Start Handshake, bit 1 Clear of Ground, bit 2 Cycle Complete. Read via /api/amr/poll."},
        {"symbol": "AMR_STATUS_PLANTER", "address": "%MW5101", "type": "WORD",
         "desc": "PLC→AMR planter status: same bit layout as %MW5100. Read via /api/amr/poll."},
    ],
    "notes": {
        "verified": "Structs and %MW/%MX addresses verified against the PLC global symbol table; "
                    "handshake block %MW5100–5112 confirmed on the bench (the older %MW100/101 map was wrong — "
                    "those addresses sit below the FEnet write base and cannot be written over Modbus).",
        "open_items": [
            "Exact bit index within each shared word (e.g. SET_AUTO / START / ENABLE_* in HMI_PB) is packed in the binary UDT — bench-confirm in XG5000.",
        ],
        "amr_bits": "The auger/planter buttons drive %MW5110/%MW5111 bit 0 directly over Modbus TCP. AugerSeq/PlanterSeq reads above are unchanged.",
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

# AMR ↔ PLC handshake block. One contiguous FC04 read covers the whole region:
# %MW5100–5112 (13 registers; 5102–5109 unused but read through).
#   PLC → AMR: %MW5100 auger status, %MW5101 planter status
#              (bit 0 Sequence Start Handshake, bit 1 Clear of Ground, bit 2 Cycle Complete)
#   AMR → PLC: %MW5110 auger cmd, %MW5111 planter cmd (bit 0 Start Sequence),
#              %MW5112 AMR state (1 = Stationary, 2 = Moving)
_AMR_HS_BASE   = 5100
_AMR_HS_COUNT  = 13
_AMR_HS_KEYS   = ("mw5100", "mw5101", "mw5110", "mw5111", "mw5112")
AMR_WRITABLE   = frozenset({5110, 5111, 5112})   # the only AMR-owned words

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
    # ── auger / planter start-sequence words — AMR↔PLC handshake @ %MW5110/5111 ──
    # Confirmed register block (bench + field): PLC→AMR status at %MW5100/5101,
    # AMR→PLC commands at %MW5110–5112. We own the command words: writing 1 sets
    # bit 0 (Start Sequence), 0 clears it. An older map used %MW100/101 — those
    # addresses sit below the FEnet write base (%MW5000) and can never be written
    # over Modbus, so they were wrong; do not reintroduce them.
    "AUGER_AMR_WORD":   "%MW5110",   # AMR→PLC: bit 0 = Auger Start Sequence
    "PLANTER_AMR_WORD": "%MW5111",   # AMR→PLC: bit 0 = Planter Start Sequence
    "AMR_STATE_WORD":   "%MW5112",   # AMR→PLC: 1 = Stationary, 2 = Moving
    # ── machine / robot pushbutton words (writes) — pulsed: write value → 100 ms → 0 ──
    "HMI_PB_MachineCtrl":  "%MW5000",   # machine PB word (bit values below)
    "HMI_PB_MachineCtrl2": "%MW5001",   # machine PB word 2 (ResetPlanterSeq/robot/AMR)
    "HMI_PB_MachineCtrl3": "%MW5002",   # machine PB word 3 (AMRokToPlant @4.1 → bit1)
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
    "JAW_METHOD_1":    ("HMI_PB_MachineCtrl2", 128),    # bit 7  JawMethod1PB @2.7
    "JAW_METHOD_2":    ("HMI_PB_MachineCtrl2", 256),    # bit 8  JawMethod2PB @3.0
    "AMR_OK_TO_PLANT": ("HMI_PB_MachineCtrl3", 2),      # bit 1  AMRokToPlant @4.1
}

# ControlRobot → ROBOT_PB_CMD (%MW6200) bit value. This map is identical to the
# ud_HMI_RobotPB bit layout (HMI_screen_and_tags.pdf p.72), which cross-confirms
# that PDF PB layout — the same source used for the axis PB words below.
_ROBOT_CMD_MAP = {
    "HOME": 1, "PAUSE": 2, "CONTINUE": 4, "MOTORS_ON": 8, "MOTORS_OFF": 16,
    "START": 32, "STOP": 64, "SHUTDOWN": 128, "RESET": 256,
}

# ── HMI control writes (pushbutton words) ─────────────────────────────────────
# Momentary pushbutton (PVPB) words the physical HMI writes. Each button is one
# bit of a word instance; a press pulses the whole word to (1<<bit) → 100 ms → 0
# (matching machine/robot buttons). Instance bases come from the program local
# variables (pp.40-62); bit layouts from the PB UDTs (pp.70-72). Only the machine
# (%MW5000/1) and robot (%MW6200) words are bench-confirmed; the axis words
# (%MW5400-6500) are from the PDF and want a live pulse-check per axis.
_HMI_PB_BITS = {
    # ud_HMI_TeknicPB (p.70)
    "teknic": {"ServoOn": 1, "ServoOff": 2, "JogFwd": 4, "JogRev": 8, "HomePos": 16,
               "ApproachPos": 32, "WorkPos": 64, "SetValues": 128, "StartHoming": 256},
    # ud_HMI_LA36PB (p.71) — adds Recovery in/out
    "la36": {"ServoOn": 1, "ServoOff": 2, "JogFwd": 4, "JogRev": 8, "HomePos": 16,
             "ApproachPos": 32, "WorkPos": 64, "SetValues": 128, "StartHoming": 256,
             "RecoveryIn": 512, "RecoveryOut": 1024},
    # ud_HMI_MotorPB (p.72)
    "motor": {"On": 1, "Off": 2, "CW": 4, "CCW": 8},
    # ud_HMI_RobotPB (p.72) — same as _ROBOT_CMD_MAP
    "robot": {"GoHome": 1, "Pause": 2, "Continue": 4, "MotorsOn": 8, "MotorsOff": 16,
              "Start": 32, "Stop": 64, "Shutdown": 128, "Reset": 256},
}
# PB instance → (kind, %MW base). All buttons fall in word 0 of the instance.
_HMI_PB_BLOCKS = {
    "HMI_PB_AugerSlide":     ("teknic", 5400),
    "HMI_PB_PlanterSlide":   ("teknic", 5500),
    "HMI_PB_AugerGimbalX":   ("la36",   5600),
    "HMI_PB_AugerGimbalY":   ("la36",   5700),
    "HMI_PB_PlanterGimbalX": ("la36",   5800),
    "HMI_PB_PlanterGimbalY": ("la36",   5900),
    "HMI_PB_PlanterJawVert": ("la36",   6000),
    "HMI_PB_PlanterTamper":  ("la36",   6100),
    "HMI_PB_Robot":          ("robot",  6200),
    "HMI_PB_Jaw1":           ("la36",   6300),
    "HMI_PB_Jaw2":           ("la36",   6400),
    "HMI_PB_Auger":          ("motor",  6500),
}
# Continuous-motion buttons: held (hold-to-jog), not pulsed. Everything else pulses.
_HMI_JOG_BUTTONS = {"JogFwd", "JogRev", "RecoveryIn", "RecoveryOut"}
_JOG_DEADMAN = 0.5          # s: a held jog auto-clears if not refreshed within this


def _resolve_pb(block, button):
    """(block, button) → (%MW addr, bit_value) if allow-listed, else None."""
    spec = _HMI_PB_BLOCKS.get(block)
    if spec is None:
        return None
    kind, base = spec
    bit = _HMI_PB_BITS[kind].get(button)
    if bit is None:
        return None
    return base, bit

# ── HMI live-mirror (read-only) ──────────────────────────────────────────────
# The physical HMI reads a set of UDT instances in the PLC's M area and shows
# their members on ~24 screens. We mirror those values read-only over the same
# FC04 word-read area used everywhere else in this module.
#
# UDT member layouts (member name, datatype, "byte.bit" offset) are transcribed
# verbatim from the XG5000 UDT editor (HMI_screen_and_tags.pdf pp.63-79). LS
# addresses are BYTE.bit within the struct, so:
#     word_offset = byte // 2 ;  bit_in_word = (byte % 2) * 8 + bit
# Cross-checked against _REG: EstopOkFL @4.0 in ud_HMI_IND → %MW1000 word 2
# bit 0 → %MX16032, which equals _REG["IND_ESTOP_OK_FL"]. STRING members
# (FaultBanner/MessageBanner) are intentionally omitted — get_banner() serves
# those separately.
#
# 32-bit word order: LS stores DWORD/DINT/REAL low-word-first (little-endian
# word order). Flip _HMI_WORD_LOW_FIRST if a bench check shows swapped halves.
_HMI_WORD_LOW_FIRST = True

_HMI_TYPE_WORDS = {"bool": 1, "word": 1, "uint": 1, "int": 1,
                   "udint": 2, "dint": 2, "real": 2}

HMI_UDT = {
    "ud_HMI_IND": [
        ("ModeStatus", "uint", "0.0"), ("CycleStatus", "uint", "2.0"),
        ("EstopOkFL", "bool", "4.0"), ("EstopOkFR", "bool", "4.1"),
        ("EstopOkRL", "bool", "4.2"), ("EstopOkRR", "bool", "4.3"),
        ("EstopOkRbt", "bool", "4.4"), ("EstopOkAMR", "bool", "4.5"),
        ("ScanOkFront", "bool", "4.6"), ("ScanOkRear", "bool", "4.7"),
        ("GateOk", "bool", "5.0"), ("JawMethod1", "bool", "5.1"),
        ("JawMethod2", "bool", "5.2"), ("AlwaysOn", "bool", "5.3"),
        ("AugerEnabled", "bool", "5.4"), ("PlanterEnabled", "bool", "5.5"),
        ("RobotEnabled", "bool", "5.6"), ("AMREnabled", "bool", "5.7"),
        ("EstopStatus", "uint", "6.0"), ("GateStatus", "uint", "8.0"),
        ("DryCycleEnabled", "bool", "10.0"),
        ("PitchDisplay", "real", "12.0"), ("PitchGauge", "uint", "16.0"),
        ("RollDisplay", "real", "20.0"), ("RollGauge", "uint", "24.0"),
        ("Faulted", "bool", "26.0"), ("AMROktoPlant", "bool", "26.1"),
        # FaultBanner STRING @28.0, MessageBanner STRING @60.0 — via get_banner()
        ("HomeStatus", "uint", "92.0"), ("Save", "bool", "94.0"),
        ("ParameterChanged", "bool", "94.1"), ("TrayPopupIND", "bool", "94.2"),
        ("AugerDistance", "real", "96.0"), ("PlanterDistance", "real", "100.0"),
    ],
    # NB: word order here follows the BENCH-CONFIRMED _REG map (AUGER_MOTOR_*),
    # NOT p.73 of HMI_screen_and_tags.pdf. The PDF lists the status bits first
    # then the velocities; the live PLC (per _REG) has velocities first and the
    # status bits at word +2, with Run/Fwd/Faulted at bits 0/1/2. Only those
    # three bits are bench-confirmed, so the PDF's Rev/On/Off/CW/CCW (positions
    # unknown here) are intentionally omitted rather than guessed. A test in
    # tests/test_plc_client.py pins these against _REG. See docs/hmi.md.
    "ud_HMI_MotorIND": [
        ("VelocityTarget", "uint", "0.0"),    # %MW2500  = _REG["AUGER_MOTOR_VEL_TARGET"]
        ("VelocityMeasured", "uint", "2.0"),  # %MW2501  = _REG["AUGER_MOTOR_VEL_ACTUAL"]
        ("Run", "bool", "4.0"),               # %MX40032 = _REG["AUGER_MOTOR_RUN"]
        ("Fwd", "bool", "4.1"),               # %MX40033 = _REG["AUGER_MOTOR_FWD"]
        ("Faulted", "bool", "4.2"),           # %MX40034 = _REG["AUGER_MOTOR_FAULTED"]
    ],
    "ud_HMI_LA36IND": [
        ("AtHome", "bool", "0.0"), ("AtApproach", "bool", "0.1"),
        ("AtWork", "bool", "0.2"), ("ReadyStart", "bool", "0.3"),
        ("NotRunning", "bool", "0.4"), ("Moving", "bool", "0.5"),
        ("RunningIn", "bool", "0.6"), ("RunningOut", "bool", "0.7"),
        ("EndReachedIn", "bool", "1.0"), ("EndReachedOut", "bool", "1.1"),
        ("PositionLost", "bool", "1.2"), ("Overcurrent", "bool", "1.3"),
        ("Faulted", "bool", "1.4"),
        ("PositionTarget", "real", "4.0"), ("PositionMeasured", "real", "8.0"),
        ("VelocityTarget", "dint", "12.0"), ("VelocityMeasured", "dint", "16.0"),
        ("TorqueMeasured", "uint", "20.0"), ("Error", "udint", "24.0"),
    ],
    "ud_HMI_TeknicIND": [
        ("ServoOn", "bool", "0.0"), ("ServoOff", "bool", "0.1"),
        ("AtHome", "bool", "0.2"), ("AtApproach", "bool", "0.3"),
        ("AtWork", "bool", "0.4"), ("ReadyForCMD", "bool", "0.5"),
        ("InRange", "bool", "0.6"), ("CMDComplete", "bool", "0.7"),
        ("Settled", "bool", "1.0"), ("AtSpeed", "bool", "1.1"),
        ("Homing", "bool", "1.2"), ("BrkReleased", "bool", "1.3"),
        ("AtTargetPos", "bool", "1.4"), ("ErrorResetAck", "bool", "1.5"),
        ("MotionBlocked", "bool", "1.6"), ("SoftwarePosLim", "bool", "1.7"),
        ("SoftwareNegLim", "bool", "2.0"), ("MotorConnected", "bool", "2.1"),
        ("FaultPresent", "bool", "2.2"), ("WarningPresent", "bool", "2.3"),
        ("PositionTarget", "real", "4.0"), ("PositionMeasured", "real", "8.0"),
        ("VelocityTarget", "dint", "12.0"), ("VelocityMeasured", "dint", "16.0"),
        ("TorqueMeasured", "uint", "20.0"), ("Error", "udint", "24.0"),
        ("Warning", "udint", "28.0"), ("HomingStatus", "uint", "32.0"),
    ],
    "ud_HMI_RobotIND": [
        ("doReady", "bool", "0.0"), ("doRunning", "bool", "0.1"),
        ("doPaused", "bool", "0.2"), ("doError", "bool", "0.3"),
        ("doEstopOn", "bool", "0.4"), ("doSafeguardOn", "bool", "0.5"),
        ("doSError", "bool", "0.6"), ("doWarning", "bool", "0.7"),
        ("doMotorsOn", "bool", "1.0"), ("doAtHome", "bool", "1.1"),
        ("doCmdRunning", "bool", "1.2"), ("doCmdError", "bool", "1.3"),
        ("doAutoMode", "bool", "1.4"), ("doTeachMode", "bool", "1.5"),
        ("doEnableOn", "bool", "1.6"), ("diStart", "bool", "1.7"),
        ("diStop", "bool", "2.0"), ("diPause", "bool", "2.1"),
        ("diContinue", "bool", "2.2"), ("diReset", "bool", "2.3"),
        ("diShutdown", "bool", "2.4"), ("diSetMotorsOn", "bool", "2.5"),
        ("diSetMotorsOff", "bool", "2.6"), ("diHome", "bool", "2.7"),
        ("diProgSel", "uint", "4.0"), ("doCmdProg", "uint", "6.0"),
        ("HomeStatus", "uint", "8.0"), ("ErrorCode", "uint", "10.0"),
    ],
    "ud_HMI_Gripper": [
        ("PositionModePVPB", "bool", "0.0"), ("JogModePVPB", "bool", "0.1"),
        ("InPositionMode", "bool", "0.2"), ("InJogMode", "bool", "0.3"),
        ("JogToBasePVPB", "bool", "0.4"), ("JogToWorkPVPB", "bool", "0.5"),
        ("JogBaseActive", "bool", "0.6"), ("JogWorkActive", "bool", "0.7"),
        ("MoveToBasePVPB", "bool", "1.0"), ("MoveToWorkPVPB", "bool", "1.1"),
        ("AtBase", "bool", "1.2"), ("AtWork", "bool", "1.3"),
        ("ActPos", "real", "4.0"),
    ],
    "ud_sequence": [
        ("Home", "bool", "0.0"), ("SetupOk", "bool", "0.1"),
        ("OkToStart", "bool", "0.2"), ("Enabled", "bool", "0.3"),
        ("InCycle", "bool", "0.4"), ("Complete", "bool", "0.5"),
        ("Reset", "bool", "0.6"), ("Step", "uint", "2.0"),
    ],
    "ud_HMI_IO": [
        ("LocalIn", "word", "0.0"), ("LocalOut", "word", "2.0"),
        ("LocalAnologIn1", "int", "4.0"), ("LocalAnologIn2", "int", "6.0"),
        ("LocalAnologOut1", "int", "8.0"), ("LocalAnologOut2", "int", "10.0"),
    ],
    "ud_HMI_Parameters": [
        ("AugerBladeJogSpeed", "udint", "0.0"), ("AugerBladeRunSpeed", "udint", "4.0"),
        ("AugerDigDepth", "real", "8.0"), ("AugerGimbalJogSpeed", "uint", "12.0"),
        ("AugerGimbalRunSpeed", "uint", "14.0"), ("AugerGimbalXHomePos", "real", "16.0"),
        ("AugerGimbalXSoftLimNeg", "real", "20.0"), ("AugerGimbalXSoftLimPos", "real", "24.0"),
        ("AugerGimbalYHomePos", "real", "28.0"), ("AugerGimbalYSoftLimNeg", "real", "32.0"),
        ("AugerGimbalYSoftLimPos", "real", "36.0"), ("AugerSlideSensorOffset", "real", "40.0"),
        ("AugerSlideAppPosition", "real", "44.0"), ("AugerSlideAppSpeed", "udint", "48.0"),
        ("AugerSlideClearPosition", "real", "52.0"), ("AugerSlideHomePosition", "real", "56.0"),
        ("AugerSlideHomeSpeed", "udint", "60.0"), ("AugerSlideJogSpeed", "udint", "64.0"),
        ("AugerSlideSoftLimNeg", "real", "68.0"), ("AugerSlideSoftLimPos", "real", "72.0"),
        ("AugerSlideWorkSpeed", "udint", "76.0"), ("PlanterDigDepth", "real", "80.0"),
        ("PlanterGimbalJogSpeed", "uint", "84.0"), ("PlanterGimbalRunSpeed", "uint", "86.0"),
        ("PlanterGimbalXHomePos", "real", "88.0"), ("PlanterGimbalXSoftLimNeg", "real", "92.0"),
        ("PlanterGimbalXSoftLimPos", "real", "96.0"), ("PlanterGimbalYHomePos", "real", "100.0"),
        ("PlanterGimbalYSoftLimNeg", "real", "104.0"), ("PlanterGimbalYSoftLimPos", "real", "108.0"),
        ("PlanterJawsJogSpeed", "uint", "112.0"), ("PlanterJawsRunSpeed", "uint", "114.0"),
        ("PlanterJawsHomePosition", "real", "116.0"), ("PlanterJawsSoftlimNeg", "real", "120.0"),
        ("PlanterJawsSoftlimPos", "real", "124.0"), ("PlanterJawsWorkPosition", "real", "128.0"),
        ("PlanterSlideSensorOffset", "real", "132.0"), ("PlanterSlideAppPosition", "real", "136.0"),
        ("PlanterSlideAppSpeed", "udint", "140.0"), ("PlanterSlideClearPosition", "real", "144.0"),
        ("PlanterSlideHomePosition", "real", "148.0"), ("PlanterSlideHomeSpeed", "udint", "152.0"),
        ("PlanterSlideJogSpeed", "udint", "156.0"), ("PlanterSlideSoftLimNeg", "real", "160.0"),
        ("PlanterSlideSoftLimPos", "real", "164.0"), ("PlanterSlideWorkSpeed", "udint", "168.0"),
        ("PlanterTampersJogSpeed", "uint", "172.0"), ("PlanterTampersRunSpeed", "uint", "174.0"),
        ("PlanterTampersHomePosition", "real", "176.0"), ("PlanterTampersSoftlimNeg", "real", "180.0"),
        ("PlanterTampersSoftlimPos", "real", "184.0"), ("PlanterTampersWorkPosition", "real", "188.0"),
        ("PlanterVertJawJogSpeed", "uint", "192.0"), ("PlanterVertJawRunSpeed", "uint", "194.0"),
        ("PlanterVertJawHomePosition", "real", "196.0"), ("PlanterVertJawSoftlimNeg", "real", "200.0"),
        ("PlanterVertJawSoftlimPos", "real", "204.0"), ("PlanterVertJawWorkPosition", "real", "208.0"),
        ("PositionTolLA14", "real", "212.0"), ("PositionTolLA36", "real", "216.0"),
        ("PositionTolSlides", "real", "220.0"), ("LA36RampUp", "uint", "224.0"),
        ("LA36RampDown", "uint", "226.0"), ("LA36CurrentLim", "uint", "228.0"),
        ("ZimmerOpenPos", "real", "232.0"), ("ZimmerClosePos", "real", "236.0"),
        ("PositionTolZimmer", "real", "240.0"), ("SlideAccel", "udint", "244.0"),
        ("SlideDecel", "udint", "248.0"), ("PlanterVertJawReleasePos", "real", "252.0"),
        ("AugerSlideOutSpeed", "udint", "256.0"),
    ],
}

# Read-relevant UDT instances (symbol → (udt, PLC %MW base)). Mirrors the read
# rows of GTS_Tree_Planter_symbols.csv; the write-only *PB instances are omitted.
HMI_BLOCKS = {
    "HMI_IND":                 ("ud_HMI_IND", 1000),
    "AugerSeq":                ("ud_sequence", 2700),
    "PlanterSeq":              ("ud_sequence", 2800),
    "HMI_IND_Auger":           ("ud_HMI_MotorIND", 2500),
    "HMI_IND_AugerGimbalX":    ("ud_HMI_LA36IND", 1600),
    "HMI_IND_AugerGimbalY":    ("ud_HMI_LA36IND", 1700),
    "HMI_IND_PlanterGimbalX":  ("ud_HMI_LA36IND", 1800),
    "HMI_IND_PlanterGimbalY":  ("ud_HMI_LA36IND", 1900),
    "HMI_IND_PlanterJawVert":  ("ud_HMI_LA36IND", 2000),
    "HMI_IND_PlanterTamper":   ("ud_HMI_LA36IND", 2100),
    "HMI_IND_Jaw1":            ("ud_HMI_LA36IND", 2300),
    "HMI_IND_Jaw2":            ("ud_HMI_LA36IND", 2400),
    "HMI_IND_AugerSlide":      ("ud_HMI_TeknicIND", 1400),
    "HMI_IND_PlanterSlide":    ("ud_HMI_TeknicIND", 1500),
    "HMI_IND_Robot":           ("ud_HMI_RobotIND", 2200),
    "HMI_Gripper":             ("ud_HMI_Gripper", 6600),
    "HMI_IO":                  ("ud_HMI_IO", 3000),
    "HMI_Parameters":          ("ud_HMI_Parameters", 5200),
}

# Standalone tags not inside an HMI UDT. NodeCommsNOk (%MW1048, UINT) packs one
# "not-OK" bit per EtherNet/IP node; invert so the mirror shows comms-OK green.
# (Bit→device order confirmed against the Communications screen, pp.3/36.)
HMI_SINGLES = {
    # name: (plc_word, bit, invert)
    "Node0CommsOk": (1048, 0, True), "Node1CommsOk": (1048, 1, True),
    "Node2CommsOk": (1048, 2, True), "Node3CommsOk": (1048, 3, True),
    "Node4CommsOk": (1048, 4, True), "Node5CommsOk": (1048, 5, True),
    "Node6CommsOk": (1048, 6, True), "Node7CommsOk": (1048, 7, True),
    "Node8CommsOk": (1048, 8, True), "Node9CommsOk": (1048, 9, True),
}


# ── C-more display formatting (fractional digits) ────────────────────────────
# The physical HMI (C-more) shows each numeric field with a fixed number of
# fractional digits. For an INTEGER PLC tag that is an *implied decimal* — the
# value is raw / 10**frac (raw 800, 2 → 8.00; raw 600000, 2 → 6000.00). For a
# REAL tag the value is already in engineering units, so frac only sets rounding.
# Keyed by block INSTANCE + member because the same UDT is shown with different
# precision on different screens (LA36 PositionMeasured: 2 on gimbals, 1 on the
# jaw-feedback screen). Source: the C-more project's Numeric object formats.
# The entry min/max limits from that export are not stored here — they clamp
# operator entry on the panel and don't change a displayed value, and this
# mirror is read-only. A test asserts every key below is a real UDT member.
_HMI_INT_TYPES = {"uint", "int", "word", "udint", "dint"}

# Teknic slides carry a Warning field; LA36 axes don't (see the two IND UDTs).
_TEKNIC_DEC = {"PositionTarget": 2, "PositionMeasured": 2, "VelocityTarget": 2,
               "VelocityMeasured": 2, "TorqueMeasured": 2, "Error": 0, "Warning": 0}
_LA36_DEC = {k: v for k, v in _TEKNIC_DEC.items() if k != "Warning"}
HMI_DECIMALS = {
    "HMI_IND_AugerSlide":     dict(_TEKNIC_DEC),
    "HMI_IND_PlanterSlide":   dict(_TEKNIC_DEC),
    "HMI_IND_AugerGimbalX":   dict(_LA36_DEC),
    "HMI_IND_AugerGimbalY":   dict(_LA36_DEC),
    "HMI_IND_PlanterGimbalX": dict(_LA36_DEC),
    "HMI_IND_PlanterGimbalY": dict(_LA36_DEC),
    "HMI_IND_PlanterJawVert": dict(_LA36_DEC),
    "HMI_IND_PlanterTamper":  dict(_LA36_DEC),
    "HMI_IND_Jaw1": {"PositionMeasured": 1},
    "HMI_IND_Jaw2": {"PositionMeasured": 1},
    "HMI_IND_Auger": {"VelocityTarget": 0, "VelocityMeasured": 0},
    "HMI_Parameters": {
        # Slide speeds are UDINT shown ×0.01; gimbal/blade/jaw/tamper speeds are
        # integer (0 frac); positions/limits/tolerances are REAL (round to frac).
        "AugerSlideJogSpeed": 2, "AugerSlideAppSpeed": 2, "AugerSlideWorkSpeed": 2,
        "AugerSlideOutSpeed": 2, "AugerSlideHomeSpeed": 2, "AugerSlideClearPosition": 2,
        "AugerSlideHomePosition": 2, "AugerSlideSensorOffset": 2, "AugerDigDepth": 2,
        "AugerBladeJogSpeed": 0, "AugerBladeRunSpeed": 0,
        "AugerGimbalJogSpeed": 0, "AugerGimbalRunSpeed": 0,
        "AugerGimbalXHomePos": 2, "AugerGimbalYHomePos": 2,
        "PlanterSlideHomePosition": 2, "PlanterSlideSensorOffset": 2,
        "PlanterSlideClearPosition": 2, "PlanterDigDepth": 2,
        "PlanterSlideJogSpeed": 2, "PlanterSlideAppSpeed": 2,
        "PlanterSlideWorkSpeed": 2, "PlanterSlideHomeSpeed": 2,
        "PlanterGimbalJogSpeed": 0, "PlanterGimbalRunSpeed": 0,
        "PlanterGimbalXHomePos": 2, "PlanterGimbalYHomePos": 2,
        "PlanterVertJawHomePosition": 2, "PlanterVertJawReleasePos": 2,
        "PlanterVertJawWorkPosition": 2, "PlanterVertJawJogSpeed": 0,
        "PlanterVertJawRunSpeed": 0,
        "PlanterTampersHomePosition": 2, "PlanterTampersWorkPosition": 2,
        "PlanterTampersJogSpeed": 0, "PlanterTampersRunSpeed": 0,
        "PlanterJawsHomePosition": 0, "PlanterJawsWorkPosition": 0,
        "ZimmerOpenPos": 2, "ZimmerClosePos": 2,
        "AugerSlideSoftLimNeg": 2, "AugerSlideSoftLimPos": 2,
        "PlanterSlideSoftLimNeg": 2, "PlanterSlideSoftLimPos": 2,
        "AugerGimbalXSoftLimNeg": 2, "AugerGimbalXSoftLimPos": 2,
        "AugerGimbalYSoftLimNeg": 2, "AugerGimbalYSoftLimPos": 2,
        "PlanterGimbalXSoftLimNeg": 2, "PlanterGimbalXSoftLimPos": 2,
        "PlanterGimbalYSoftLimNeg": 2, "PlanterGimbalYSoftLimPos": 2,
        "PlanterVertJawSoftlimNeg": 2, "PlanterVertJawSoftlimPos": 2,
        "PlanterTampersSoftlimNeg": 2, "PlanterTampersSoftlimPos": 2,
        "PositionTolSlides": 2, "PositionTolLA36": 2,
    },
}


def _hmi_fmt_value(block, member, raw):
    """Apply the C-more display format → a fixed-decimals string, or return `raw`
    unchanged when the field has no format spec or isn't a plain number. Integer
    tags divide by 10**frac (implied decimal); REAL tags only round."""
    dec = HMI_DECIMALS.get(block, {}).get(member)
    if dec is None or isinstance(raw, bool) or not isinstance(raw, (int, float)):
        return raw
    udt, _base = HMI_BLOCKS[block]
    dt = next((d for n, d, _a in HMI_UDT[udt] if n == member), None)
    val = raw / (10 ** dec) if dt in _HMI_INT_TYPES else float(raw)
    return f"{val:.{dec}f}"


def _hmi_addr(addr):
    """'byte.bit' struct offset → (word_offset, bit_in_word)."""
    b, _, k = str(addr).partition(".")
    b, k = int(b), int(k or 0)
    return b // 2, (b % 2) * 8 + k


def _hmi_block_words(udt):
    """Total FC04 word span needed to cover every member of a UDT."""
    count = 0
    for _, dt, addr in HMI_UDT[udt]:
        w, _b = _hmi_addr(addr)
        count = max(count, w + _HMI_TYPE_WORDS[dt])
    return count


def _hmi_decode(words, dt, w, bit):
    """Decode one member from a block's raw FC04 words. None if out of range."""
    if w >= len(words):
        return None
    if dt == "bool":
        return bool((words[w] >> bit) & 1)
    if dt in ("uint", "word"):
        return words[w]
    if dt == "int":
        v = words[w]
        return v - 0x10000 if v & 0x8000 else v
    if w + 1 >= len(words):
        return None
    lo, hi = (words[w], words[w + 1]) if _HMI_WORD_LOW_FIRST else (words[w + 1], words[w])
    if dt == "udint":
        return (hi << 16) | lo
    if dt == "dint":
        v = (hi << 16) | lo
        return v - 0x100000000 if v & 0x80000000 else v
    if dt == "real":
        return round(struct.unpack("<f", struct.pack("<HH", lo, hi))[0], 3)
    return None


def _hmi_unit(name, dt):
    """Cosmetic unit for a member, inferred from its name/type (display only)."""
    if dt == "bool":
        return ""
    n = name.lower()
    if "pitch" in n or "roll" in n:
        return "°" if "gauge" not in n else ""
    if "torque" in n:
        return "Nm"
    if "velocity" in n or "speed" in n:
        return "RPM" if ("blade" in n or dt == "uint" and "gimbal" not in n and "slide" not in n) else "mm/s"
    if any(k in n for k in ("position", "pos", "home", "offset", "depth",
                            "distance", "softlim", "clear", "release", "actpos")):
        return "mm"
    return ""


HMI_SCREENS = [
    # ── Overview ──
    {"id": "main", "title": "Main", "section": "Overview", "panels": [
        {"title": "Auger Sequence", "block": "AugerSeq"},
        {"title": "Planter Sequence", "block": "PlanterSeq"},
        {"title": "Mode & Cycle", "block": "HMI_IND",
         "members": ["ModeStatus", "CycleStatus", "HomeStatus", "Faulted", "AMROktoPlant"]},
        {"title": "Safety", "block": "HMI_IND",
         "members": ["EstopStatus", "GateStatus", "EstopOkFL", "EstopOkFR", "EstopOkRL",
                     "EstopOkRR", "EstopOkRbt", "EstopOkAMR", "ScanOkFront", "ScanOkRear", "GateOk"]},
    ]},
    # ── Status ──
    {"id": "io_digital", "title": "PLC Digital I/O", "section": "Status", "panels": [
        {"title": "Inputs (LocalIn)", "rows": [
            {"label": "IN00 Spare", "ref": "HMI_IO.LocalIn#0", "kind": "bool"},
            {"label": "IN01 Spare", "ref": "HMI_IO.LocalIn#1", "kind": "bool"},
            {"label": "IN02 Gate Locked", "ref": "HMI_IO.LocalIn#2", "kind": "bool"},
            {"label": "IN03 Spare", "ref": "HMI_IO.LocalIn#3", "kind": "bool"},
            {"label": "IN04 Auger DC Drive Speed Pulse", "ref": "HMI_IO.LocalIn#4", "kind": "bool"},
            {"label": "IN05 Auger DC Drive Alarm", "ref": "HMI_IO.LocalIn#5", "kind": "bool"},
            {"label": "IN06 Spare", "ref": "HMI_IO.LocalIn#6", "kind": "bool"},
            {"label": "IN07 Spare", "ref": "HMI_IO.LocalIn#7", "kind": "bool"},
            {"label": "IN08 Spare", "ref": "HMI_IO.LocalIn#8", "kind": "bool"},
            {"label": "IN09 Spare", "ref": "HMI_IO.LocalIn#9", "kind": "bool"},
            {"label": "IN10 Spare", "ref": "HMI_IO.LocalIn#10", "kind": "bool"},
            {"label": "IN11 Spare", "ref": "HMI_IO.LocalIn#11", "kind": "bool"},
            {"label": "IN12 Spare", "ref": "HMI_IO.LocalIn#12", "kind": "bool"},
            {"label": "IN13 Spare", "ref": "HMI_IO.LocalIn#13", "kind": "bool"},
            {"label": "IN14 Spare", "ref": "HMI_IO.LocalIn#14", "kind": "bool"},
            {"label": "IN15 Spare", "ref": "HMI_IO.LocalIn#15", "kind": "bool"},
        ]},
        {"title": "Outputs (LocalOut)", "rows": [
            {"label": "OUT00 Spare", "ref": "HMI_IO.LocalOut#0", "kind": "bool"},
            {"label": "OUT01 Spare", "ref": "HMI_IO.LocalOut#1", "kind": "bool"},
            {"label": "OUT02 Gate Lock", "ref": "HMI_IO.LocalOut#2", "kind": "bool"},
            {"label": "OUT03 Spare", "ref": "HMI_IO.LocalOut#3", "kind": "bool"},
            {"label": "OUT04 Spare", "ref": "HMI_IO.LocalOut#4", "kind": "bool"},
            {"label": "OUT05 Spare", "ref": "HMI_IO.LocalOut#5", "kind": "bool"},
            {"label": "OUT06 Auger DC Drive Fwd/Rev", "ref": "HMI_IO.LocalOut#6", "kind": "bool"},
            {"label": "OUT07 Auger DC Drive Run/Stop", "ref": "HMI_IO.LocalOut#7", "kind": "bool"},
            {"label": "OUT08 Auger DC Drive Brake", "ref": "HMI_IO.LocalOut#8", "kind": "bool"},
            {"label": "OUT09 Robot Safeguard Latch Release", "ref": "HMI_IO.LocalOut#9", "kind": "bool"},
            {"label": "OUT10 Planter Jaw 1 Extend", "ref": "HMI_IO.LocalOut#10", "kind": "bool"},
            {"label": "OUT11 Planter Jaw 1 Retract", "ref": "HMI_IO.LocalOut#11", "kind": "bool"},
            {"label": "OUT12 Planter Jaw 2 Extend", "ref": "HMI_IO.LocalOut#12", "kind": "bool"},
            {"label": "OUT13 Planter Jaw 2 Retract", "ref": "HMI_IO.LocalOut#13", "kind": "bool"},
            {"label": "OUT14 Spare", "ref": "HMI_IO.LocalOut#14", "kind": "bool"},
            {"label": "OUT15 Spare", "ref": "HMI_IO.LocalOut#15", "kind": "bool"},
        ]},
    ]},
    {"id": "io_analog", "title": "PLC Analog I/O", "section": "Status", "panels": [
        {"title": "Analog", "rows": [
            {"label": "AD0 Jaw 1 Feedback", "ref": "HMI_IO.LocalAnologIn1", "kind": "num", "unit": ""},
            {"label": "AD1 Jaw 2 Feedback", "ref": "HMI_IO.LocalAnologIn2", "kind": "num", "unit": ""},
            {"label": "DA0 Auger DC Drive Speed", "ref": "HMI_IO.LocalAnologOut1", "kind": "num", "unit": ""},
            {"label": "DA1 Spare", "ref": "HMI_IO.LocalAnologOut2", "kind": "num", "unit": ""},
        ]},
    ]},
    {"id": "safety_layout", "title": "Safety Layout", "section": "Status", "panels": [
        {"title": "E-Stops", "block": "HMI_IND",
         "members": ["EstopOkFL", "EstopOkFR", "EstopOkRL", "EstopOkRR", "EstopOkRbt", "EstopOkAMR"]},
        {"title": "Scanners & Gate", "block": "HMI_IND",
         "members": ["ScanOkFront", "ScanOkRear", "GateOk"]},
    ]},
    {"id": "gauges", "title": "Gauges", "section": "Status", "panels": [
        {"title": "Inclinometer", "block": "HMI_IND",
         "members": ["PitchDisplay", "PitchGauge", "RollDisplay", "RollGauge"]},
        {"title": "Distances", "block": "HMI_IND",
         "members": ["AugerDistance", "PlanterDistance"]},
    ]},
    {"id": "communications", "title": "Ethernet I/P Communications", "section": "Status",
     "layout": "comms", "panels": [
        {"title": "EtherNet/IP Nodes", "rows": [
            {"label": "KEYENCE GC1000 - SAFETY PLC", "ip": "192.168.1.4", "ref": "single:Node0CommsOk", "kind": "bool"},
            {"label": "TURCK IO LINK MASTER", "ip": "192.168.1.6", "ref": "single:Node1CommsOk", "kind": "bool"},
            {"label": "TEKNIC IO HUB - MAIN SLIDES", "ip": "192.168.1.10", "ref": "single:Node2CommsOk", "kind": "bool"},
            {"label": "LINAK LA36 - AUGER GIMBAL X AXIS", "ip": "192.168.1.11", "ref": "single:Node3CommsOk", "kind": "bool"},
            {"label": "LINAK LA36 - AUGER GIMBAL Y AXIS", "ip": "192.168.1.12", "ref": "single:Node4CommsOk", "kind": "bool"},
            {"label": "LINAK LA36 - PLANTER GIMBAL X AXIS", "ip": "192.168.1.13", "ref": "single:Node5CommsOk", "kind": "bool"},
            {"label": "LINAK LA36 - PLANTER GIMBAL Y AXIS", "ip": "192.168.1.14", "ref": "single:Node6CommsOk", "kind": "bool"},
            {"label": "LINAK LA36 - PLANTER VERTICAL JAW SLIDE", "ip": "192.168.1.15", "ref": "single:Node7CommsOk", "kind": "bool"},
            {"label": "LINAK LA36 - PLANTER TAMPERS", "ip": "192.168.1.16", "ref": "single:Node8CommsOk", "kind": "bool"},
            {"label": "EPSON VT6 ROBOT", "ip": "192.168.1.20", "ref": "single:Node9CommsOk", "kind": "bool"},
        ]},
    ]},
    # Robot I/O lists (reached from the I/O sub-menu → ROBOT · INPUTS / OUTPUTS).
    {"id": "robot_inputs", "title": "Robot Inputs", "section": "I/O", "panels": [
        {"title": "Robot Inputs (PLC → Robot)", "block": "HMI_IND_Robot", "members": [
            "diStart", "diStop", "diPause", "diContinue", "diReset", "diShutdown",
            "diSetMotorsOn", "diSetMotorsOff", "diHome"]},
        {"title": "Program", "block": "HMI_IND_Robot", "members": ["diProgSel"]},
    ]},
    {"id": "robot_outputs", "title": "Robot Outputs", "section": "I/O", "panels": [
        {"title": "Robot Outputs (Robot → PLC)", "block": "HMI_IND_Robot", "members": [
            "doReady", "doRunning", "doPaused", "doError", "doEstopOn", "doSafeguardOn",
            "doSError", "doWarning", "doMotorsOn", "doAtHome", "doCmdRunning", "doCmdError",
            "doAutoMode", "doTeachMode", "doEnableOn"]},
        {"title": "Echo", "block": "HMI_IND_Robot",
         "members": ["doCmdProg", "HomeStatus", "ErrorCode"]},
    ]},
    # ── Auger Controls ──
    {"id": "auger_main_slide", "title": "Auger Main Slide", "section": "Auger Controls",
     "panels": [{"title": "Auger Main Slide (Teknic)", "block": "HMI_IND_AugerSlide"}]},
    {"id": "auger_gimbal_x", "title": "Auger Gimbal X", "section": "Auger Controls",
     "panels": [{"title": "Auger Gimbal X Axis (LA36)", "block": "HMI_IND_AugerGimbalX"}]},
    {"id": "auger_gimbal_y", "title": "Auger Gimbal Y", "section": "Auger Controls",
     "panels": [{"title": "Auger Gimbal Y Axis (LA36)", "block": "HMI_IND_AugerGimbalY"}]},
    {"id": "auger_motor", "title": "Auger Motor", "section": "Auger Controls",
     "panels": [{"title": "Main Auger Motor (DC Drive)", "block": "HMI_IND_Auger"}]},
    # ── Main Controls ──
    {"id": "epson_robot", "title": "Epson Robot", "section": "Main Controls", "panels": [
        {"title": "Robot I/O (VT6)", "block": "HMI_IND_Robot"},
        {"title": "Gripper", "block": "HMI_Gripper"},
    ]},
    {"id": "amr", "title": "AMR", "section": "Main Controls", "panels": [
        {"title": "AMR", "block": "HMI_IND", "members": ["AMROktoPlant"]},
    ]},
    # ── Planter Controls ──
    {"id": "planter_main_slide", "title": "Planter Main Slide", "section": "Planter Controls",
     "panels": [{"title": "Planter Main Slide (Teknic)", "block": "HMI_IND_PlanterSlide"}]},
    {"id": "planter_gimbal_x", "title": "Planter Gimbal X", "section": "Planter Controls",
     "panels": [{"title": "Planter Gimbal X Axis (LA36)", "block": "HMI_IND_PlanterGimbalX"}]},
    {"id": "planter_gimbal_y", "title": "Planter Gimbal Y", "section": "Planter Controls",
     "panels": [{"title": "Planter Gimbal Y Axis (LA36)", "block": "HMI_IND_PlanterGimbalY"}]},
    {"id": "planter_jaw_vertical", "title": "Planter Jaw Vertical", "section": "Planter Controls",
     "panels": [{"title": "Planter Jaw Vertical Slide (LA36)", "block": "HMI_IND_PlanterJawVert"}]},
    {"id": "planter_jaws", "title": "Auger Jaw Controls", "section": "Planter Controls", "panels": [
        {"title": "Jaw 1 (LA36)", "block": "HMI_IND_Jaw1"},
        {"title": "Jaw 2 (LA36)", "block": "HMI_IND_Jaw2"},
        {"title": "Control Method", "block": "HMI_IND", "members": ["JawMethod1", "JawMethod2"]},
        {"title": "Feedback & Outputs", "rows": [
            {"label": "Jaw 1 Position Measured", "ref": "HMI_IO.LocalAnologIn1", "kind": "num", "unit": "%"},
            {"label": "Jaw 2 Position Measured", "ref": "HMI_IO.LocalAnologIn2", "kind": "num", "unit": "%"},
            {"label": "Jaw 1 Close Output", "ref": "HMI_IO.LocalOut#10", "kind": "bool"},
            {"label": "Jaw 1 Open Output", "ref": "HMI_IO.LocalOut#11", "kind": "bool"},
            {"label": "Jaw 2 Close Output", "ref": "HMI_IO.LocalOut#12", "kind": "bool"},
            {"label": "Jaw 2 Open Output", "ref": "HMI_IO.LocalOut#13", "kind": "bool"},
        ]},
    ]},
    {"id": "planter_tampers", "title": "Planter Tampers", "section": "Planter Controls",
     "panels": [{"title": "Planter Tampers (LA36)", "block": "HMI_IND_PlanterTamper"}]},
    # ── Parameters (display-only; friendly labels mirror the PDF screens) ──
    {"id": "auger_params_1", "title": "Auger Parameters 1", "section": "Parameters", "panels": [
        {"title": "Main Slide Velocity", "rows": [
            {"label": "Jogging", "ref": "HMI_Parameters.AugerSlideJogSpeed", "kind": "num", "unit": "mm/s"},
            {"label": "Run Approach", "ref": "HMI_Parameters.AugerSlideAppSpeed", "kind": "num", "unit": "mm/s"},
            {"label": "Run Dig", "ref": "HMI_Parameters.AugerSlideWorkSpeed", "kind": "num", "unit": "mm/s"},
            {"label": "Run Dig Out", "ref": "HMI_Parameters.AugerSlideOutSpeed", "kind": "num", "unit": "mm/s"},
            {"label": "Run Return", "ref": "HMI_Parameters.AugerSlideHomeSpeed", "kind": "num", "unit": "mm/s"},
        ]},
        {"title": "Blade Velocity", "rows": [
            {"label": "Jogging", "ref": "HMI_Parameters.AugerBladeJogSpeed", "kind": "num", "unit": "RPM"},
            {"label": "Run", "ref": "HMI_Parameters.AugerBladeRunSpeed", "kind": "num", "unit": "RPM"},
        ]},
        {"title": "Positions", "rows": [
            {"label": "Home Position", "ref": "HMI_Parameters.AugerSlideHomePosition", "kind": "num", "unit": "mm"},
            {"label": "Sensor to Blade Offset", "ref": "HMI_Parameters.AugerSlideSensorOffset", "kind": "num", "unit": "mm"},
            {"label": "Safe Move Height", "ref": "HMI_Parameters.AugerSlideClearPosition", "kind": "num", "unit": "mm"},
            {"label": "Dig Depth", "ref": "HMI_Parameters.AugerDigDepth", "kind": "num", "unit": "mm"},
        ]},
    ]},
    {"id": "auger_params_2", "title": "Auger Parameters 2", "section": "Parameters", "panels": [
        {"title": "Gimbal Velocity", "rows": [
            {"label": "Jogging", "ref": "HMI_Parameters.AugerGimbalJogSpeed", "kind": "num", "unit": "%"},
            {"label": "Run", "ref": "HMI_Parameters.AugerGimbalRunSpeed", "kind": "num", "unit": "%"},
        ]},
        {"title": "Home Positions", "rows": [
            {"label": "Y Axis Home Position", "ref": "HMI_Parameters.AugerGimbalYHomePos", "kind": "num", "unit": "mm"},
            {"label": "X Axis Home Position", "ref": "HMI_Parameters.AugerGimbalXHomePos", "kind": "num", "unit": "mm"},
        ]},
    ]},
    {"id": "planter_params_1", "title": "Planter Parameters 1", "section": "Parameters", "panels": [
        {"title": "Main Slide Velocity", "rows": [
            {"label": "Jogging", "ref": "HMI_Parameters.PlanterSlideJogSpeed", "kind": "num", "unit": "mm/s"},
            {"label": "Run Approach", "ref": "HMI_Parameters.PlanterSlideAppSpeed", "kind": "num", "unit": "mm/s"},
            {"label": "Run Work", "ref": "HMI_Parameters.PlanterSlideWorkSpeed", "kind": "num", "unit": "mm/s"},
            {"label": "Run Return", "ref": "HMI_Parameters.PlanterSlideHomeSpeed", "kind": "num", "unit": "mm/s"},
        ]},
        {"title": "Positions", "rows": [
            {"label": "Home Position", "ref": "HMI_Parameters.PlanterSlideHomePosition", "kind": "num", "unit": "mm"},
            {"label": "Sensor to Spade Offset", "ref": "HMI_Parameters.PlanterSlideSensorOffset", "kind": "num", "unit": "mm"},
            {"label": "Safe Move Height", "ref": "HMI_Parameters.PlanterSlideClearPosition", "kind": "num", "unit": "mm"},
            {"label": "Dig Depth", "ref": "HMI_Parameters.PlanterDigDepth", "kind": "num", "unit": "mm"},
        ]},
    ]},
    {"id": "planter_params_2", "title": "Planter Parameters 2", "section": "Parameters", "panels": [
        {"title": "Gimbal Velocity", "rows": [
            {"label": "Jogging", "ref": "HMI_Parameters.PlanterGimbalJogSpeed", "kind": "num", "unit": "%"},
            {"label": "Run", "ref": "HMI_Parameters.PlanterGimbalRunSpeed", "kind": "num", "unit": "%"},
        ]},
        {"title": "Home Positions", "rows": [
            {"label": "Y Axis Home Position", "ref": "HMI_Parameters.PlanterGimbalYHomePos", "kind": "num", "unit": "mm"},
            {"label": "X Axis Home Position", "ref": "HMI_Parameters.PlanterGimbalXHomePos", "kind": "num", "unit": "mm"},
        ]},
    ]},
    {"id": "planter_params_3", "title": "Planter Parameters 3", "section": "Parameters", "panels": [
        {"title": "Jaws Vert Velocity", "rows": [
            {"label": "Jogging", "ref": "HMI_Parameters.PlanterVertJawJogSpeed", "kind": "num", "unit": "%"},
            {"label": "Run", "ref": "HMI_Parameters.PlanterVertJawRunSpeed", "kind": "num", "unit": "%"},
        ]},
        {"title": "Tampers Velocity", "rows": [
            {"label": "Jogging", "ref": "HMI_Parameters.PlanterTampersJogSpeed", "kind": "num", "unit": "%"},
            {"label": "Run", "ref": "HMI_Parameters.PlanterTampersRunSpeed", "kind": "num", "unit": "%"},
        ]},
        {"title": "Jaws Vert Positions", "rows": [
            {"label": "Jaws Vert Home Position", "ref": "HMI_Parameters.PlanterVertJawHomePosition", "kind": "num", "unit": "mm"},
            {"label": "Jaws Vert Release", "ref": "HMI_Parameters.PlanterVertJawReleasePos", "kind": "num", "unit": "mm"},
            {"label": "Jaws Vert Clear", "ref": "HMI_Parameters.PlanterVertJawWorkPosition", "kind": "num", "unit": "mm"},
        ]},
        {"title": "Tamper / Jaws Positions", "rows": [
            {"label": "Tamper Home Position", "ref": "HMI_Parameters.PlanterTampersHomePosition", "kind": "num", "unit": "mm"},
            {"label": "Tamper Work Position", "ref": "HMI_Parameters.PlanterTampersWorkPosition", "kind": "num", "unit": "mm"},
            {"label": "Jaws Home Position", "ref": "HMI_Parameters.PlanterJawsHomePosition", "kind": "num", "unit": "%"},
            {"label": "Jaws Work Position", "ref": "HMI_Parameters.PlanterJawsWorkPosition", "kind": "num", "unit": "%"},
        ]},
    ]},
    {"id": "robot_params", "title": "Robot Parameters", "section": "Parameters", "panels": [
        {"title": "Gripper (Zimmer)", "rows": [
            {"label": "Open Position", "ref": "HMI_Parameters.ZimmerOpenPos", "kind": "num", "unit": "mm"},
            {"label": "Close Position", "ref": "HMI_Parameters.ZimmerClosePos", "kind": "num", "unit": "mm"},
        ]},
    ]},
    {"id": "tolerances", "title": "Tolerances & Limits", "section": "Parameters", "panels": [
        {"title": "Auger Soft Limits", "rows": [
            {"label": "Auger Main Slide Soft Limit (-)", "ref": "HMI_Parameters.AugerSlideSoftLimNeg", "kind": "num", "unit": "mm"},
            {"label": "Auger Main Slide Soft Limit (+)", "ref": "HMI_Parameters.AugerSlideSoftLimPos", "kind": "num", "unit": "mm"},
            {"label": "Auger Gimbal X Axis Soft Limit (-)", "ref": "HMI_Parameters.AugerGimbalXSoftLimNeg", "kind": "num", "unit": "mm"},
            {"label": "Auger Gimbal X Axis Soft Limit (+)", "ref": "HMI_Parameters.AugerGimbalXSoftLimPos", "kind": "num", "unit": "mm"},
            {"label": "Auger Gimbal Y Axis Soft Limit (-)", "ref": "HMI_Parameters.AugerGimbalYSoftLimNeg", "kind": "num", "unit": "mm"},
            {"label": "Auger Gimbal Y Axis Soft Limit (+)", "ref": "HMI_Parameters.AugerGimbalYSoftLimPos", "kind": "num", "unit": "mm"},
        ]},
        {"title": "Planter Soft Limits", "rows": [
            {"label": "Planter Main Slide Soft Limit (-)", "ref": "HMI_Parameters.PlanterSlideSoftLimNeg", "kind": "num", "unit": "mm"},
            {"label": "Planter Main Slide Soft Limit (+)", "ref": "HMI_Parameters.PlanterSlideSoftLimPos", "kind": "num", "unit": "mm"},
            {"label": "Planter Gimbal X Axis Soft Limit (-)", "ref": "HMI_Parameters.PlanterGimbalXSoftLimNeg", "kind": "num", "unit": "mm"},
            {"label": "Planter Gimbal X Axis Soft Limit (+)", "ref": "HMI_Parameters.PlanterGimbalXSoftLimPos", "kind": "num", "unit": "mm"},
            {"label": "Planter Gimbal Y Axis Soft Limit (-)", "ref": "HMI_Parameters.PlanterGimbalYSoftLimNeg", "kind": "num", "unit": "mm"},
            {"label": "Planter Gimbal Y Axis Soft Limit (+)", "ref": "HMI_Parameters.PlanterGimbalYSoftLimPos", "kind": "num", "unit": "mm"},
            {"label": "Planter Vert Jaw Slide Soft Limit (-)", "ref": "HMI_Parameters.PlanterVertJawSoftlimNeg", "kind": "num", "unit": "mm"},
            {"label": "Planter Vert Jaw Slide Soft Limit (+)", "ref": "HMI_Parameters.PlanterVertJawSoftlimPos", "kind": "num", "unit": "mm"},
            {"label": "Planter Tampers Soft Limit (-)", "ref": "HMI_Parameters.PlanterTampersSoftlimNeg", "kind": "num", "unit": "mm"},
            {"label": "Planter Tampers Soft Limit (+)", "ref": "HMI_Parameters.PlanterTampersSoftlimPos", "kind": "num", "unit": "mm"},
        ]},
        {"title": "Position Tolerances", "rows": [
            {"label": "Main Slides Position Tolerance", "ref": "HMI_Parameters.PositionTolSlides", "kind": "num", "unit": "mm"},
            {"label": "LA36 Actuators Position Tolerance", "ref": "HMI_Parameters.PositionTolLA36", "kind": "num", "unit": "mm"},
            {"label": "LA14 Actuators Position Tolerance", "ref": "HMI_Parameters.PositionTolLA14", "kind": "num", "unit": "mm"},
            {"label": "Gripper Position Tolerance", "ref": "HMI_Parameters.PositionTolZimmer", "kind": "num", "unit": "mm"},
        ]},
    ]},
    {"id": "enable_disable", "title": "Feature Enable / Disable", "section": "Parameters",
     "layout": "enable", "panels": [
        {"title": "Feature Enables", "block": "HMI_IND",
         "members": ["AugerEnabled", "PlanterEnabled", "RobotEnabled", "AMREnabled", "DryCycleEnabled"]},
    ]},
]

HMI_SCREEN_BY_ID = {s["id"]: s for s in HMI_SCREENS}

# Physical HMI navigation. `root` mirrors the MENU screen (pp.2): six button
# columns; each button targets a data screen ("screen:<id>") or a sub-menu
# ("menu:<key>"). `io` mirrors the I/O sub-menu (pp.6). Button labels carry
# newlines exactly as they wrap on the panel.
HMI_MENU = {
    "root": {"title": "MENU", "columns": [
        {"header": "STATUS", "buttons": [
            {"label": "I/O", "target": "menu:io"},
            {"label": "SAFETY", "target": "screen:safety_layout"},
            {"label": "GAUGES", "target": "screen:gauges"},
            {"label": "COMMUNICATIONS", "target": "screen:communications"},
        ]},
        {"header": "PARAMETERS", "buttons": [
            {"label": "AUGER\nPARAMETERS 1", "target": "screen:auger_params_1"},
            {"label": "AUGER\nPARAMETERS 2", "target": "screen:auger_params_2"},
            {"label": "PLANTER\nPARAMETERS 1", "target": "screen:planter_params_1"},
            {"label": "PLANTER\nPARAMETERS 2", "target": "screen:planter_params_2"},
            {"label": "PLANTER\nPARAMETERS 3", "target": "screen:planter_params_3"},
            {"label": "ROBOT\nPARAMETERS", "target": "screen:robot_params"},
        ]},
        {"header": "AUGER CONTROLS", "buttons": [
            {"label": "AUGER\nMAIN SLIDE", "target": "screen:auger_main_slide"},
            {"label": "AUGER\nGIMBAL X-AXIS", "target": "screen:auger_gimbal_x"},
            {"label": "AUGER\nGIMBAL Y-AXIS", "target": "screen:auger_gimbal_y"},
            {"label": "AUGER\nBLADE", "target": "screen:auger_motor"},
        ]},
        {"header": "MAIN CONTROLS", "buttons": [
            {"label": "EPSON\nROBOT", "target": "screen:epson_robot"},
            {"label": "AMR", "target": "screen:amr"},
        ]},
        {"header": "PLANTER CONTROLS", "buttons": [
            {"label": "PLANTER\nMAIN SLIDE", "target": "screen:planter_main_slide"},
            {"label": "PLANTER\nGIMBAL X-AXIS", "target": "screen:planter_gimbal_x"},
            {"label": "PLANTER\nGIMBAL Y-AXIS", "target": "screen:planter_gimbal_y"},
            {"label": "PLANTER\nJAW VERTICAL", "target": "screen:planter_jaw_vertical"},
            {"label": "PLANTER\nJAWS", "target": "screen:planter_jaws"},
            {"label": "PLANTER\nTAMPERS", "target": "screen:planter_tampers"},
        ]},
        {"header": "", "buttons": [
            {"label": "TOLERANCES &\nLIMITS", "target": "screen:tolerances"},
            {"label": "ENABLE /\nDISABLE", "target": "screen:enable_disable"},
            {"label": "MAIN\nSCREEN", "target": "screen:main"},
        ]},
    ]},
    "io": {"title": "IO MENU", "parent": "root", "columns": [
        {"header": "PLC", "buttons": [
            {"label": "DIGITAL I/O", "target": "screen:io_digital"},
            {"label": "ANALOG I/O", "target": "screen:io_analog"},
        ]},
        {"header": "ROBOT", "buttons": [
            {"label": "INPUTS", "target": "screen:robot_inputs"},
            {"label": "OUTPUTS", "target": "screen:robot_outputs"},
        ]},
        {"header": "", "buttons": [
            {"label": "MAIN\nSCREEN", "target": "screen:main"},
        ]},
    ]},
}

# Visual template each screen renders with in the browser (mirrors the PDF).
# "panels" = generic lamp/value cards; the rest are bespoke HMI layouts. A screen
# may also carry its own "layout" key (wins over this map); anything unlisted → "panels".
HMI_LAYOUT = {
    "main": "main", "gauges": "gauges", "amr": "amr",
    "auger_motor": "motor", "epson_robot": "robot", "planter_jaws": "jaws",
    "auger_main_slide": "motion", "planter_main_slide": "motion",
    "auger_gimbal_x": "motion", "auger_gimbal_y": "motion",
    "planter_gimbal_x": "motion", "planter_gimbal_y": "motion",
    "planter_jaw_vertical": "motion", "planter_tampers": "motion",
    # everything else (params, I/O, safety, robot I/O lists) → "panels"
}


def _hmi_expand_layout(screen):
    """Turn a screen definition into concrete panels/rows (labels, refs, kinds,
    units). No PLC access — pure structure, safe when the PLC is down."""
    panels = []
    for p in screen["panels"]:
        rows = []
        if "block" in p:
            sym = p["block"]
            udt, _base = HMI_BLOCKS[sym]
            wanted = p.get("members")
            for name, dt, _addr in HMI_UDT[udt]:
                if wanted is not None and name not in wanted:
                    continue
                rows.append({"label": name, "ref": f"{sym}.{name}",
                             "kind": "bool" if dt == "bool" else "num",
                             "unit": _hmi_unit(name, dt)})
            if wanted is not None:      # preserve the screen's member order
                order = {m: i for i, m in enumerate(wanted)}
                rows.sort(key=lambda r: order.get(r["label"], 1_000))
        else:
            for r in p["rows"]:
                row = {"label": r["label"], "ref": r["ref"],
                       "kind": r.get("kind", "bool"), "unit": r.get("unit", "")}
                if "ip" in r:               # Communications screen: device IP
                    row["ip"] = r["ip"]
                rows.append(row)
        panels.append({"title": p["title"], "rows": rows})
    return {"screen": screen["id"], "title": screen["title"],
            "section": screen["section"],
            "layout": screen.get("layout") or HMI_LAYOUT.get(screen["id"], "panels"),
            "panels": panels}


class PlcClient:
    """Modbus TCP client to the LS Electric PLC. host/port point at the PLC's Modbus
    server (192.168.1.2:502 for agrobot). One socket, serialized by ``_lock`` so a 100 ms
    pulse and concurrent status polls don't interleave on the wire."""

    _CONNECT_COOLDOWN = 3.0                      # s to wait before re-attempting a failed connect

    def __init__(self, host="127.0.0.1", port=502, timeout=2.0):
        self.host    = host
        self.port    = int(port)
        self.timeout = float(timeout)          # Modbus socket timeout (s)
        self._lock   = threading.Lock()
        self._client = None                     # pymodbus ModbusTcpClient (lazy)
        self._import_error = None               # set once if pymodbus is unavailable
        self._next_retry = 0.0                  # monotonic time before which we don't reconnect
        self._jog = None                        # (addr, bitval, expiry) of a held jog, or None

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
        # Negative cache: after a failed connect, don't re-attempt (a blocking
        # ~`timeout`s TCP connect) for _CONNECT_COOLDOWN seconds. Without this,
        # every poll on every tab — all serialized on _lock — stacks up 2 s
        # connects when the PLC is down and starves each other past the client
        # fetch timeout. Fast-fail keeps a downed PLC a normal, snappy 200.
        if time.monotonic() < self._next_retry:
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
            self._next_retry = time.monotonic() + self._CONNECT_COOLDOWN
            log.warning("PLC Modbus connect failed → %s (retry in %.0fs)",
                        self.target, self._CONNECT_COOLDOWN)
            return None
        self._client = client
        self._next_retry = 0.0
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
        self._next_retry = 0.0                   # allow an immediate reconnect after a drop

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

    def _write_word(self, plc_addr, value):
        """FC06 write to %MW<plc_addr>. Refuses anything below the FEnet write base
        (%MW5000) — a hard floor so a bad address can never scribble on the read area."""
        reg = plc_addr - _FENET_WRITE_WORD_BASE
        if reg < 0:
            log.warning("_write_word: %%MW%d below FEnet write area (min %%MW%d)",
                        plc_addr, _FENET_WRITE_WORD_BASE)
            return False
        res = self._client.write_register(reg, int(value) & 0xFFFF)
        return not res.isError()

    def _pulse_word(self, plc_addr, bitval):
        """Momentary: pulse a single PB word to `bitval` → 100 ms → 0."""
        if not self._write_word(plc_addr, bitval):
            return False
        time.sleep(0.1)
        self._write_word(plc_addr, 0)
        return True

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
        """Decode 16-bit words into an ASCII string. The LS PLC packs strings
        little-endian within each word (low byte = first char) — confirmed on the
        live PLC, where high-byte-first produced pair-swapped gibberish
        ('lPna tlSdi...' instead of 'Planter Slide Motor Not Connected')."""
        out = []
        for w in words:
            out.append(w & 0xFF)
            out.append((w >> 8) & 0xFF)
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
    # START → write 1 to the handshake word (sets bit 0); STOP → write 0 (clears it).
    # No read-first toggle: the JS caller already decides START vs STOP based on
    # auger_in_cycle, so writing unconditionally is correct and avoids writing the
    # wrong value when the register was left non-zero from a prior session.
    def control_auger(self, command):
        new = 0 if (command or "").upper() == "STOP" else 1
        def fn():
            ok = self._write("AUGER_AMR_WORD", new)
            log.debug("Auger: write %%MW5110=%d ok=%s", new, ok)
            return {"success": ok, "message": f"Auger: %MW5110 bit 0 → {new}",
                    "auger_active": bool(new), "planter_active": False}
        return self._op(fn)

    def control_planter(self, command):
        new = 0 if (command or "").upper() == "STOP" else 1
        def fn():
            ok = self._write("PLANTER_AMR_WORD", new)
            log.debug("Planter: write %%MW5111=%d ok=%s", new, ok)
            return {"success": ok, "message": f"Planter: %MW5111 bit 0 → {new}",
                    "auger_active": False, "planter_active": bool(new)}
        return self._op(fn)

    def control_both(self, command):
        new = 0 if (command or "").upper() == "STOP" else 1
        def fn():
            ok_a = self._write("AUGER_AMR_WORD", new)
            ok_p = self._write("PLANTER_AMR_WORD", new)
            log.debug("Both: write %%MW5110/%%MW5111=%d ok=%s/%s", new, ok_a, ok_p)
            return {"success": (ok_a and ok_p),
                    "message": f"Both: %MW5110/%MW5111 bit 0 → {new}",
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

    # -- HMI control-page writes (axis/motor pushbuttons) ----------------------
    def press_button(self, block, button):
        """Pulse a momentary axis/motor pushbutton (allow-listed). Jog buttons are
        rejected here — they must go through jog() so the deadman applies."""
        def fn():
            spec = _resolve_pb(block, button)
            if spec is None:
                return {"success": False, "message": f"unknown button {block}.{button}"}
            if button in _HMI_JOG_BUTTONS:
                return {"success": False, "message": f"{button} is a jog button — use /api/hmi/jog"}
            addr, bitval = spec
            ok = self._pulse_word(addr, bitval)
            return {"success": ok, "message": f"{block}.{button}: pulsed %MW{addr} bit {bitval}"}
        return self._op(fn)

    def jog(self, block, button, action):
        """Hold-to-jog a continuous-motion button. action 'start'/'refresh' sets the
        bit and (re)arms a deadman; 'stop' clears it. If refreshes lapse (tab closed,
        connection lost) jog_deadman() clears the bit. Only one jog is held at a time."""
        action = (action or "").lower()
        def fn():
            spec = _resolve_pb(block, button)
            if spec is None or button not in _HMI_JOG_BUTTONS:
                return {"success": False, "message": f"{block}.{button} is not a jog button"}
            addr, bitval = spec
            if action == "stop":
                self._write_word(addr, 0)
                if self._jog and self._jog[0] == addr:
                    self._jog = None
                return {"success": True, "message": f"{block}.{button}: jog stop"}
            # start / refresh — if switching axes, drop the previous jog's bit first
            if self._jog and self._jog[0] != addr:
                self._write_word(self._jog[0], 0)
            ok = self._write_word(addr, bitval)
            self._jog = (addr, bitval, time.monotonic() + _JOG_DEADMAN)
            return {"success": ok, "message": f"{block}.{button}: jog {action}"}
        return self._op(fn)

    def jog_deadman(self):
        """Background-thread tick (~10 Hz): clear a held jog whose refresh has lapsed.
        Cheap no-op (no PLC I/O) unless a jog is actually pending."""
        if self._jog is None:
            return
        def fn():
            j = self._jog
            if j and time.monotonic() > j[2]:
                self._write_word(j[0], 0)
                self._jog = None
                log.warning("jog deadman: cleared held jog at %%MW%d (refresh lapsed)", j[0])
                return {"success": True, "message": "jog deadman cleared"}
            return {"success": True, "message": "ok"}
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

    # -- HMI live mirror (read-only) -------------------------------------------
    @staticmethod
    def hmi_screens_meta():
        """Navigation for the HMI mirror: the physical MENU (pp.2) button columns
        plus the I/O sub-menu (pp.6). Buttons target either a data screen
        (``screen:<id>``) or another menu (``menu:<key>``). Also returns a flat
        {id: title} map so the browser can label any screen. No PLC access."""
        titles = {s["id"]: s["title"] for s in HMI_SCREENS}
        return {"menus": HMI_MENU, "root": "root", "titles": titles}

    def _read_hmi_words(self, base, count):
        """FC04 batch read of `count` words from %MW<base>, chunked to stay under
        the ~125-register FC04 limit. Returns a word list or None on any error."""
        out, off = [], 0
        while off < count:
            n = min(120, count - off)
            chunk = self._read_words_raw(base + off, n)
            if chunk is None:
                return None
            out.extend(chunk)
            off += n
        return out

    def read_hmi_screen(self, screen_id):
        """Return the screen's layout with a live value on every row. Structure is
        always present; values are None when the PLC is unreachable (connected
        False). Unknown screen → {'error': ...}."""
        screen = HMI_SCREEN_BY_ID.get(screen_id)
        if screen is None:
            return {"connected": False, "error": f"unknown screen '{screen_id}'"}
        layout = _hmi_expand_layout(screen)

        blocks, singles = set(), set()
        for panel in layout["panels"]:
            for row in panel["rows"]:
                ref = row["ref"]
                if ref.startswith("single:"):
                    singles.add(ref[len("single:"):])
                else:
                    blocks.add(ref.split(".", 1)[0].split("#", 1)[0])

        values, connected = {}, False
        with self._lock:
            client = self._ensure_client()
            if client is not None:
                connected = True
                # Read each block/single behind its own guard. Two failure modes,
                # handled differently:
                #   • Modbus *exception response* (isError) → _read_* returns None:
                #     a bad address, but the socket is fine — yield None, keep going.
                #   • Transport error (broken pipe/reset/timeout) → raises: the
                #     shared pymodbus socket is dead. Stop, and _reset() below so
                #     the next poll reconnects — the same recovery _op gives every
                #     other read path (without this, the HMI stays stuck on a dead
                #     socket while /api/amr/poll self-heals).
                transport_error = False
                for sym in blocks:
                    udt, base = HMI_BLOCKS[sym]
                    words = None
                    if not transport_error:
                        try:
                            words = self._read_hmi_words(base, _hmi_block_words(udt))
                        except Exception as exc:
                            log.warning("HMI block %s read failed, dropping socket: %s", sym, exc)
                            transport_error = True
                    for name, dt, addr in HMI_UDT[udt]:
                        w, bit = _hmi_addr(addr)
                        values[f"{sym}.{name}"] = (
                            None if words is None else _hmi_decode(words, dt, w, bit))
                for nm in singles:
                    word_addr, bit, invert = HMI_SINGLES[nm]
                    w = None
                    if not transport_error:
                        try:
                            w = self._read_words_raw(word_addr, 1)
                        except Exception as exc:
                            log.warning("HMI single %s read failed, dropping socket: %s", nm, exc)
                            transport_error = True
                    if w is None:
                        values[f"single:{nm}"] = None
                    else:
                        v = bool((w[0] >> bit) & 1)
                        values[f"single:{nm}"] = (not v) if invert else v
                if transport_error:
                    self._reset()

        for panel in layout["panels"]:
            for row in panel["rows"]:
                ref = row["ref"]
                if "#" in ref and not ref.startswith("single:"):
                    base_ref, _, bs = ref.partition("#")
                    v = values.get(base_ref)
                    row["value"] = None if v is None else bool((int(v) >> int(bs)) & 1)
                elif "." in ref and not ref.startswith("single:"):
                    blk, mem = ref.split(".", 1)
                    row["value"] = _hmi_fmt_value(blk, mem, values.get(ref))
                else:
                    row["value"] = values.get(ref)

        # Block/single values keyed for the bespoke HMI templates. Numeric fields
        # with a C-more format spec are rendered as fixed-decimals strings (in
        # engineering units); bools and unspecced numbers pass through raw.
        blocks_out = {}
        for sym in blocks:
            udt, _base = HMI_BLOCKS[sym]
            blocks_out[sym] = {m[0]: _hmi_fmt_value(sym, m[0], values.get(f"{sym}.{m[0]}"))
                               for m in HMI_UDT[udt]}
        layout["blocks"] = blocks_out
        layout["singles"] = {nm: values.get(f"single:{nm}") for nm in singles}
        layout["connected"] = connected
        return layout

    # -- AMR handshake block (%MW5100–5112) ------------------------------------
    def amr_poll(self):
        """Read the whole AMR↔PLC handshake block in one FC04 read.
        Response shape matches the old standalone handshake server so
        plc_combined.html works unchanged: {connected, latency_ms, host, port,
        error, mw5100, mw5101, mw5110, mw5111, mw5112}."""
        t0 = time.perf_counter()
        def fn():
            vals = self._read_words_raw(_AMR_HS_BASE, _AMR_HS_COUNT)
            latency_ms = round((time.perf_counter() - t0) * 1000, 1)
            if vals is None:
                return {"success": False, "latency_ms": latency_ms,
                        "error": f"FC04 read error at %MW{_AMR_HS_BASE}",
                        **{k: None for k in _AMR_HS_KEYS}}
            return {"success": True, "latency_ms": latency_ms, "error": None,
                    "mw5100": vals[0], "mw5101": vals[1],
                    "mw5110": vals[10], "mw5111": vals[11], "mw5112": vals[12]}
        out = self._op(fn)
        if not out.get("connected"):
            out.setdefault("error", out.get("message"))
            out["latency_ms"] = None
            for k in _AMR_HS_KEYS:
                out.setdefault(k, None)
        out["host"] = self.host
        out["port"] = self.port
        return out

    def amr_write(self, plc_addr, value):
        """FC06 write one AMR-owned handshake word, then read it back to confirm.
        Only %MW5110/5111/5112 are writable — everything else is PLC-owned."""
        try:
            plc_addr = int(plc_addr)
            value    = int(value) & 0xFFFF
        except (TypeError, ValueError):
            return {"connected": True, "success": False,
                    "message": "'reg' and 'value' must be integers"}
        if plc_addr not in AMR_WRITABLE:
            return {"connected": True, "success": False,
                    "message": f"%MW{plc_addr} is not a writable handshake register"}
        def fn():
            reg = plc_addr - _FENET_WRITE_WORD_BASE
            res = self._client.write_register(reg, value)
            ok  = not res.isError()
            rb  = self._read_words_raw(plc_addr, 1)
            readback = rb[0] if rb else None
            return {"success": ok,
                    "message": (f"wrote %MW{plc_addr} = {value} (FC06 reg {reg})"
                                if ok else f"FC06 error at reg {reg}: {res}"),
                    "reg":           f"%MW{plc_addr}",
                    "value_written": value,
                    "readback":      readback,
                    "confirmed":     (readback == value) if readback is not None else None}
        return self._op(fn)

    def amr_set_moving(self, moving):
        """Write the AMR movement state word (%MW5112): 2 = Moving, 1 = Stationary."""
        return self.amr_write(5112, 2 if moving else 1)

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
