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
