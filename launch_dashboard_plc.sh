#!/usr/bin/env bash
# launch_dashboard_plc.sh — Combined AMR teleoperation + PLC handshake dashboard.
#
# Merges launch_dashboard (camera, WASD, GPS, full chassis stack) with
# launch_plc2 (AMR ↔ PLC handshake registers) into a single 4-tab web page:
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
# The PLC handshake uses the same host/port as the main PLC (from agrobot.yaml).
# No separate server or port is needed — everything runs on port 8766 (default).
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
# Default to port 8769 (unique: 8766=dashboard/wide, 8767=plc, 8768=plc2).
# User can override with --port N; since launch_dashboard.sh takes the last --port
# it sees, prepending means the user's explicit flag always wins.
SERVE_PY="dashboard/serve_plc.py" exec "$SCRIPT_DIR/launch_dashboard.sh" --port 8769 "$@"
