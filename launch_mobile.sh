#!/usr/bin/env bash
# launch_mobile.sh — Full dashboard for driving from the wireless remote.
#
# Same combined dashboard as launch_dashboard_plc.sh (cameras, GPS, connectivity,
# PLC handshake + control keys, detection, recording) with ONE difference: the
# Jetson never commands the wheels, so the T10 wireless remote keeps full control.
#
# Two things are turned off, together:
#   • DASHBOARD_NO_BASE=1 → robot_base_node is NOT started. Nothing writes the
#     chassis Modbus speed bus, so the wireless remote owns the drive.
#   • --no-teleop         → the dashboard hides the WASD keys, speed presets and
#     2 m Fwd/Bwd buttons and never POSTs /api/cmd_vel. (REC / Det stay.)
#
# The PLC panel and its control keys are unaffected — the PLC is reached over
# Modbus TCP on the LAN, independent of the chassis serial bus.
#
# Trade-off: with robot_base_node down there is no chassis battery / wheel-odom
# telemetry in the dashboard (both come from that node). The PLC's own screens
# are unaffected.
#
# NOTE: this assumes the chassis returns wheel control to the wireless remote once
# nothing is writing its Modbus speed registers. If the remote is still locked out
# after launching this (i.e. the chassis stays latched in Modbus mode), the base
# needs an explicit control-mode reset — tell me and we'll add a one-shot for it.
#
# Usage (same flags as launch_dashboard.sh):
#   ./launch_mobile.sh
#   ./launch_mobile.sh --chassis agrobot --port 8766
#   ./launch_mobile.sh --headless

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Default port 8771 so it can run alongside the main (8766), PLC (8769) and
# battery-test (8770) dashboards. The user's explicit --port always wins
# (launch_dashboard.sh takes the last --port it sees).
DASHBOARD_NO_BASE=1 SERVE_PY="dashboard/serve_plc.py" SERVE_EXTRA="--no-teleop" \
  exec "$SCRIPT_DIR/launch_dashboard.sh" --port 8771 "$@"
