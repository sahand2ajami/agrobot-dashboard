#!/usr/bin/env bash
# launch_dashboard_plc.sh — Combined AMR teleoperation + PLC handshake dashboard.
#
# The full chassis stack (camera, WASD, GPS) plus the AMR ↔ PLC handshake
# registers, in a single 4-tab web page:
#
#   📷  Camera       — live camera stream + W/A/S/D keyboard controls
#   🗺  GPS          — OpenStreetMap with live position + coordinate details
#   📡  Connectivity — live status of PLC, cameras, GPS, robot base, battery
#   ⚙   PLC Handshake — %MW5100–5112 register monitor + write panel + event log
#
# AMR state is automatically written to %MW5112:
#   2 (Bit 1 set) → AMR Moving      (WASD or joystick active)
#   1 (Bit 0 set) → AMR Stationary  (no recent velocity command)
#
# The PLC handshake uses the same host/port — and the same Modbus socket — as
# the main PLC client (from agrobot.yaml). Default HTTP port: 8769.
#
# Usage (same flags as launch_dashboard.sh):
#   ./launch_dashboard_plc.sh
#   ./launch_dashboard_plc.sh --chassis agrobot --port 8766
#   ./launch_dashboard_plc.sh --headless
#
# Options (all forwarded to launch_dashboard.sh):
#   --chassis <name>    agrobot | jackal  (default from active_chassis.yaml)
#   --port N            HTTP listen port  (default: 8766)
#   --headless          Serve only; don't open a local browser
#   --no-headless       Force-open a browser (default behaviour)
#   --rear-camera <src> Pass through to serve.py

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Default to port 8769 so it can run alongside the main dashboard (8766).
# User can override with --port N; since launch_dashboard.sh takes the last --port
# it sees, prepending means the user's explicit flag always wins.
SERVE_PY="dashboard/serve_plc.py" exec "$SCRIPT_DIR/launch_dashboard.sh" --port 8769 "$@"
