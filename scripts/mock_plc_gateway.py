#!/usr/bin/env python3
"""Software emulator of the PLC Gateway's RobotService — test the dashboard PLC
integration with NO PLC and NO real gateway in the loop.

It speaks the same gRPC service (RobotService on :50051) as ~/plc_gateway, using the
dashboard's own vendored stubs (dashboard/plc/), so it runs in the dashboard's normal
Python — just `grpcio` (already a dashboard dependency). It emulates the parts of the PLC
that matter for exercising the UI:

  • Mode + safety gating — auger/planter START only takes effect in AUTO mode with the
    subsystem enabled (mirrors the real PLC ladder preconditions).
  • Sequence cycles — a START flips *_in_cycle true, then auto-clears after --cycle-secs,
    so the dashboard's completion poll fires ("Auger complete ✓", seedling pin, etc.).
  • Fault / E-stop / gate states you can toggle from the CLI to test those UI badges.

Usage:
    python3 scripts/mock_plc_gateway.py              # idle machine; set up from the UI
    python3 scripts/mock_plc_gateway.py --auto        # boot in AUTO + everything enabled
    python3 scripts/mock_plc_gateway.py --cycle-secs 6
    python3 scripts/mock_plc_gateway.py --fault        # boot faulted (red badge)
    python3 scripts/mock_plc_gateway.py --estop        # boot with E-stop tripped

Then run the dashboard against it (default host/port already point at localhost:50051):
    ./launch_dashboard.sh --chassis agrobot
"""
import argparse
import logging
import sys
import threading
import time
from concurrent import futures
from pathlib import Path

# Import the vendored client stubs (same .proto as the real gateway).
_DASH = Path(__file__).resolve().parent.parent / "dashboard"
sys.path.insert(0, str(_DASH))
import grpc                                   # noqa: E402
from plc import robot_control_pb2 as pb       # noqa: E402
from plc import robot_control_pb2_grpc as pbg # noqa: E402

log = logging.getLogger("mock_plc")

_STATE_KEYS = ("estop_ok", "gate_ok", "faulted", "auger_enabled", "planter_enabled",
               "robot_enabled", "amr_enabled", "auger_in_cycle", "planter_in_cycle",
               "mode_auto", "mode_manual")


class MockMachine:
    def __init__(self, cycle_secs=3.0):
        self.cycle_secs = cycle_secs
        self.lock = threading.Lock()
        self.s = dict(estop_ok=True, gate_ok=True, faulted=False,
                      auger_enabled=False, planter_enabled=False,
                      robot_enabled=False, amr_enabled=False,
                      auger_in_cycle=False, planter_in_cycle=False,
                      mode_auto=False, mode_manual=True)

    def _finish_after(self, key):
        def run():
            time.sleep(self.cycle_secs)
            with self.lock:
                self.s[key] = False
            log.info("%s cycle complete", key)
        threading.Thread(target=run, daemon=True).start()

    # Returns True if the START was accepted (preconditions met).
    def start_sequence(self, key):
        with self.lock:
            ready = self.s["mode_auto"] and self.s[key.split("_")[0] + "_enabled"] \
                    and self.s["estop_ok"] and self.s["gate_ok"] and not self.s["faulted"]
            if ready and not self.s[key]:
                self.s[key] = True
                self._finish_after(key)
            return ready

    def stop_sequence(self, key):
        with self.lock:
            self.s[key] = False

    def machine_status(self, success=True, message="OK"):
        with self.lock:
            return pb.MachineStatusResponse(success=success, message=message,
                                            **{k: self.s[k] for k in _STATE_KEYS})


class Servicer(pbg.RobotServiceServicer):
    def __init__(self, machine):
        self.m = machine

    def _seq(self, key, label, request):
        cmd = request.command.upper()
        if cmd == "START":
            ok = self.m.start_sequence(key)
            msg = f"{label} started" if ok else f"{label} START written (PLC gated: needs AUTO + enabled + safety OK)"
        else:
            self.m.stop_sequence(key)
            msg = f"{label} stopped"
        log.info("Control%s %s -> %s", label, cmd, msg)
        active = self.m.s[key]
        is_auger = key.startswith("auger")
        return pb.SequenceStatusResponse(success=True, message=msg,
                                         auger_active=active if is_auger else self.m.s["auger_in_cycle"],
                                         planter_active=active if not is_auger else self.m.s["planter_in_cycle"])

    def ControlAuger(self, request, context):
        return self._seq("auger_in_cycle", "Auger", request)

    def ControlPlanter(self, request, context):
        return self._seq("planter_in_cycle", "Planter", request)

    def ControlBoth(self, request, context):
        self._seq("auger_in_cycle", "Auger", request)
        self._seq("planter_in_cycle", "Planter", request)
        return pb.SequenceStatusResponse(success=True, message="Both pulsed",
                                         auger_active=self.m.s["auger_in_cycle"],
                                         planter_active=self.m.s["planter_in_cycle"])

    def MachineCommand(self, request, context):
        cmd = request.command.upper()
        with self.m.lock:
            s = self.m.s
            if cmd == "SET_AUTO":      s["mode_auto"], s["mode_manual"] = True, False
            elif cmd == "SET_MANUAL":  s["mode_auto"], s["mode_manual"] = False, True
            elif cmd == "FAULT_RESET": s["faulted"] = False
            elif cmd.startswith("ENABLE_"):  s[cmd[len("ENABLE_"):].lower() + "_enabled"] = True
            elif cmd.startswith("DISABLE_"): s[cmd[len("DISABLE_"):].lower() + "_enabled"] = False
            # HOME_ALL / START / STOP / RESET_* are accepted as no-ops here.
        log.info("MachineCommand %s", cmd)
        return self.m.machine_status(message=f"{cmd} ok")

    def ControlRobot(self, request, context):
        log.info("ControlRobot %s", request.command.upper())
        return self.m.machine_status(message=f"robot {request.command.upper()} ok")

    def GetMachineStatus(self, request, context):
        return self.m.machine_status()

    def GetSequenceDetail(self, request, context):
        with self.m.lock:
            s = self.m.s
            return pb.SequenceDetailResponse(
                auger_home=not s["auger_in_cycle"], auger_setup_ok=True,
                auger_ok_to_start=s["mode_auto"] and s["auger_enabled"],
                auger_enabled=s["auger_enabled"], auger_in_cycle=s["auger_in_cycle"],
                auger_complete=False, auger_step=0x20 if s["auger_in_cycle"] else 0,
                planter_home=not s["planter_in_cycle"], planter_setup_ok=True,
                planter_ok_to_start=s["mode_auto"] and s["planter_enabled"],
                planter_enabled=s["planter_enabled"], planter_in_cycle=s["planter_in_cycle"],
                planter_complete=False, planter_step=0x65 if s["planter_in_cycle"] else 0)

    def GetAugerMotorStatus(self, request, context):
        with self.m.lock:
            running = self.m.s["auger_in_cycle"]
        return pb.AugerMotorStatusResponse(success=True, message="OK", running=running,
                                           fwd_direction=True, faulted=False,
                                           velocity_target=92, velocity_actual=87 if running else 0)


def main():
    ap = argparse.ArgumentParser(description="Mock PLC gateway (RobotService emulator)")
    ap.add_argument("--port", type=int, default=50051)
    ap.add_argument("--cycle-secs", type=float, default=3.0,
                    help="how long an auger/planter cycle runs before auto-completing")
    ap.add_argument("--auto", action="store_true",
                    help="boot in AUTO mode with auger/planter/robot/amr enabled (skip UI setup)")
    ap.add_argument("--fault", action="store_true", help="boot in a faulted state")
    ap.add_argument("--estop", action="store_true", help="boot with E-stop tripped")
    ap.add_argument("--gate-open", action="store_true", help="boot with the safety gate open")
    args = ap.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(asctime)s [mock_plc] %(message)s",
                        datefmt="%H:%M:%S")

    machine = MockMachine(cycle_secs=args.cycle_secs)
    if args.auto:
        machine.s.update(mode_auto=True, mode_manual=False, auger_enabled=True,
                         planter_enabled=True, robot_enabled=True, amr_enabled=True)
    if args.fault:     machine.s["faulted"] = True
    if args.estop:     machine.s["estop_ok"] = False
    if args.gate_open: machine.s["gate_ok"] = False

    server = grpc.server(futures.ThreadPoolExecutor(max_workers=8))
    pbg.add_RobotServiceServicer_to_server(Servicer(machine), server)
    server.add_insecure_port(f"[::]:{args.port}")
    server.start()
    log.info("RobotService emulator on :%d  (cycle=%.1fs, auto=%s) — Ctrl-C to stop",
             args.port, args.cycle_secs, args.auto)
    try:
        server.wait_for_termination()
    except KeyboardInterrupt:
        log.info("shutting down")
        server.stop(0)


if __name__ == "__main__":
    main()
