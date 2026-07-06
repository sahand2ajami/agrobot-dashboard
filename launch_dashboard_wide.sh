#!/usr/bin/env bash
# Wide-angle variant of the dashboard launcher.
#
# Identical to launch_dashboard.sh in every way (same chassis selection, same
# services, same index.html) except serve.py runs with --wide: the front ZED 2i
# opens at HD2K (full ~110° FOV, 15 fps) and the UI letterboxes video instead
# of cropping. The old serve_wide.py/index_wide.html fork is gone.
#
# Usage (same flags as launch_dashboard.sh):
#   ./launch_dashboard_wide.sh --chassis jackal
#   ./launch_dashboard_wide.sh --chassis agrobot --port 8080

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SERVE_EXTRA="--wide" exec "$SCRIPT_DIR/launch_dashboard.sh" "$@"
