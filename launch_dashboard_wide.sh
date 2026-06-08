#!/usr/bin/env bash
# Wide-angle variant of the dashboard launcher.
#
# Identical to launch_dashboard.sh in every way (same chassis selection, same
# services) except it runs dashboard/serve_wide.py, which serves index_wide.html
# (object-fit: contain, no cropping) and opens the ZED 2i at HD2K (full FOV).
#
# Usage (same flags as launch_dashboard.sh):
#   ./launch_dashboard_wide.sh --chassis jackal
#   ./launch_dashboard_wide.sh --chassis agrobot --port 8080

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SERVE_PY="dashboard/serve_wide.py" exec "$SCRIPT_DIR/launch_dashboard.sh" "$@"
