"""Thin gRPC client to the PLC Gateway (RobotService on :50051).

The dashboard's HTTP server (serve.py) owns one PlcClient and exposes its RPCs as
REST endpoints, so the browser never speaks gRPC directly:

    Browser --REST--> serve.py --gRPC--> PLC Gateway --Modbus TCP--> LS Electric PLC

Design points (mirroring chassis.py / the camera threads):
- ``grpc`` and the vendored stubs are imported *lazily* inside the client, so this
  module imports fine — and ``pytest tests/`` runs — even where grpc / the stubs are
  absent. Nothing here pulls in hardware at import time.
- Every public method returns a plain ``dict`` (never a protobuf) and never raises into
  the HTTP handler: on a Gateway that is offline / unreachable it returns
  ``{"connected": False, "success": False, "message": ...}``. A short per-call deadline
  keeps a dead Gateway from stalling a request thread.
- ``success: true`` from the Gateway means the Modbus write landed, *not* that the
  machine moved (the PLC ladder gates real motion on mode/safety/preconditions). The UI
  confirms real completion by polling GetSequenceDetail / GetMachineStatus.
"""

import logging
import threading

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
            "symbol": "HMI_PB_Auger", "address": "%MW6500", "type": "ud_HMI_MotorPB",
            "rpc": "ControlAuger", "api": "/api/plc/auger",
            "desc": "Auger sequence start/stop pushbuttons.",
            "commands": sorted(SEQUENCE_COMMANDS),
        },
    ],
    "reserved": [
        {"symbol": "AMR_2_PLC", "address": "%MW100", "type": "ARRAY[0..9] OF WORD",
         "desc": "Reserved AMR→PLC handshake — declared in the PLC, not referenced in this ladder build."},
        {"symbol": "PLC_2_AMR", "address": "%MW200", "type": "ARRAY[0..9] OF WORD",
         "desc": "Reserved PLC→AMR handshake — declared in the PLC, not referenced in this ladder build."},
    ],
    "notes": {
        "verified": "Structs and %MW addresses verified against the PLC global symbol table.",
        "open_items": [
            "Exact bit index within each shared word (e.g. SET_AUTO / START / ENABLE_* in HMI_PB) is packed in the binary UDT — bench-confirm in XG5000.",
            "Modbus address base: the PLC's Modbus-TCP/FEnet server must expose %MW at the offsets the gateway assumes.",
        ],
        "planter_seq": "ControlPlanter / ControlBoth are gateway-mapped sequence RPCs; their pushbutton register is internal to the gateway and not separately listed here.",
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



def _msg_to_dict(msg):
    """Flatten a protobuf response into a JSON-friendly dict (scalar fields only)."""
    return {f.name: getattr(msg, f.name) for f in msg.DESCRIPTOR.fields}


class PlcClient:
    def __init__(self, host="127.0.0.1", port=50051, timeout=2.0):
        self.host    = host
        self.port    = int(port)
        self.timeout = float(timeout)          # per-call gRPC deadline (s)
        self._lock   = threading.Lock()
        self._grpc   = None                     # the grpc module (lazy)
        self._pb     = None                     # robot_control_pb2 (lazy)
        self._channel = None
        self._stub   = None
        self._import_error = None               # set once if grpc/stubs are unavailable

    @property
    def target(self):
        return f"{self.host}:{self.port}"

    # -- channel / stub lifecycle --------------------------------------------
    def _ensure_stub(self):
        """Lazily import grpc + stubs and build the channel/stub. Returns the stub,
        or None if grpc / the vendored stubs are unavailable. Caller holds _lock."""
        if self._stub is not None:
            return self._stub
        if self._import_error is not None:
            return None
        try:
            import grpc
            try:                                # cwd == dashboard/  (python3 dashboard/serve.py)
                from plc import robot_control_pb2 as pb
                from plc import robot_control_pb2_grpc as pbg
            except ImportError:                 # imported as a package (dashboard.plc)
                from dashboard.plc import robot_control_pb2 as pb
                from dashboard.plc import robot_control_pb2_grpc as pbg
        except Exception as exc:                # grpc missing, stubs missing, etc.
            self._import_error = str(exc)
            log.warning("PLC client disabled — grpc/stubs unavailable: %s", exc)
            return None
        self._grpc    = grpc
        self._pb      = pb
        self._channel = grpc.insecure_channel(self.target)
        self._stub    = pbg.RobotServiceStub(self._channel)
        log.info("PLC gRPC client bound to %s", self.target)
        return self._stub

    def _reset_channel(self):
        """Tear down the channel so a later call reconnects (e.g. after the Gateway
        restarts under us). Acquires the lock itself."""
        with self._lock:
            if self._channel is not None:
                try:
                    self._channel.close()
                except Exception:
                    pass
            self._channel = None
            self._stub    = None

    def _invoke(self, method_name, build_request):
        """Run one RPC and return a uniform dict. ``build_request(pb)`` constructs the
        request message. The gRPC network call happens *outside* the lock so concurrent
        status polls don't queue behind a slow write."""
        with self._lock:
            stub = self._ensure_stub()
            if stub is None:
                return {"connected": False, "success": False,
                        "message": f"{_UNAVAILABLE}: {self._import_error or 'not initialised'}"}
            grpc    = self._grpc
            method  = getattr(stub, method_name)
            try:
                request = build_request(self._pb)
            except Exception as exc:
                return {"connected": False, "success": False, "message": f"bad request: {exc}"}

        try:
            resp = method(request, timeout=self.timeout)
        except grpc.RpcError as exc:
            code = exc.code() if hasattr(exc, "code") else None
            details = exc.details() if hasattr(exc, "details") else str(exc)
            self._reset_channel()
            log.warning("PLC RPC %s failed (%s): %s", method_name, code, details)
            return {"connected": False, "success": False,
                    "message": f"Gateway unreachable ({code})"}
        except Exception as exc:
            self._reset_channel()
            log.warning("PLC RPC %s error: %s", method_name, exc)
            return {"connected": False, "success": False, "message": str(exc)}

        out = _msg_to_dict(resp)
        out["connected"] = True
        return out

    # -- write RPCs -----------------------------------------------------------
    def control_auger(self, command):
        return self._invoke("ControlAuger", lambda pb: pb.SequenceCommandRequest(command=command))

    def control_planter(self, command):
        return self._invoke("ControlPlanter", lambda pb: pb.SequenceCommandRequest(command=command))

    def control_both(self, command):
        return self._invoke("ControlBoth", lambda pb: pb.SequenceCommandRequest(command=command))

    def machine_command(self, command):
        return self._invoke("MachineCommand", lambda pb: pb.MachineCommandRequest(command=command))

    def control_robot(self, command):
        return self._invoke("ControlRobot", lambda pb: pb.MachineCommandRequest(command=command))

    # -- read RPCs ------------------------------------------------------------
    def get_machine_status(self):
        return self._invoke("GetMachineStatus", lambda pb: pb.Empty())

    def get_sequence_detail(self):
        return self._invoke("GetSequenceDetail", lambda pb: pb.Empty())

    def get_auger_motor_status(self):
        return self._invoke("GetAugerMotorStatus", lambda pb: pb.Empty())

    def close(self):
        self._reset_channel()
