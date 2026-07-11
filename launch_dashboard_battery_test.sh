#!/usr/bin/env bash
# launch_dashboard_battery_test.sh — Battery drain-test dashboard.
#
# A focused single-page UI for running the robot on a repeating 2 m
# forward / 2 m backward cycle until the battery is empty, to characterise
# pack runtime. The page has:
#
#   • 2m Fwd / 2m Bwd  — the same server-side encoder drives as the main dashboard
#   • W / A / S / D     — manual teleop (shares /js/teleop.js and its safety clamp)
#   • Battery %         — live pack state of charge
#   • Speed setting     — drive speed for both manual and auto cycles
#   • Auto / Stop       — Auto loops 2 m Fwd ⇄ 2 m Bwd forever until the battery
#                         hits the cutoff; Stop halts everything immediately
#   • Counter           — forward / backward legs and completed cycles
#
# The auto cycle runs SERVER-SIDE (serve_battery.py background thread), so the
# test survives a browser hiccup or a closed tab. Default HTTP port: 8770.
#
# Usage (same flags as launch_dashboard.sh):
#   ./launch_dashboard_battery_test.sh
#   ./launch_dashboard_battery_test.sh --chassis agrobot --port 8770
#   ./launch_dashboard_battery_test.sh --headless

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Default to port 8770 so it can run alongside the main (8766) and PLC (8769)
# dashboards. Prepending --port means a user's explicit --port still wins
# (launch_dashboard.sh takes the last --port it sees).
SERVE_PY="dashboard/serve_battery.py" exec "$SCRIPT_DIR/launch_dashboard.sh" --port 8770 "$@"
