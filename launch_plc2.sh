#!/usr/bin/env bash
# launch_plc2.sh — AMR ↔ PLC handshake register dashboard
#
# Serves a live HMI for the Modbus handshake registers on port 8768:
#
#   PLC → AMR  (read via FC04)
#     %MW5100   Auger status   bit0=Seq Start Handshake, bit1=Clear of Ground, bit2=Cycle Complete
#     %MW5101   Planter status bit0=Seq Start Handshake, bit1=Clear of Ground, bit2=Cycle Complete
#
#   AMR → PLC  (write via FC06)
#     %MW5110   Auger command  bit1=Auger Start Sequence   (value 2 = active)
#     %MW5111   Planter cmd    bit1=Planter Start Sequence (value 2 = active)
#     %MW5112   AMR state      bit0=Stationary (1), bit1=Moving (2)
#
#   FEnet offsets (default XG5000):
#     FC04 reg N  = %MW(1000+N)   → %MW5100 = reg 4100
#     FC06 reg N  = %MW(5000+N)   → %MW5110 = reg 110
#
# Usage:
#   ./launch_plc2.sh
#   ./launch_plc2.sh --port 8769
#   ./launch_plc2.sh --plc-host 192.168.1.2 --headless
#
# Options:
#   --port N        HTTP listen port        (default: 8768)
#   --plc-host H    PLC Modbus TCP host     (default: 192.168.1.2)
#   --plc-port N    PLC Modbus TCP port     (default: 502)
#   --headless      Skip opening a browser  (or set DASHBOARD_HEADLESS=1)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

PORT=8768
PLC_HOST="192.168.1.2"
PLC_PORT=502
HEADLESS="${DASHBOARD_HEADLESS:-0}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --port)        PORT="$2";     shift 2 ;;
    --plc-host)    PLC_HOST="$2"; shift 2 ;;
    --plc-port)    PLC_PORT="$2"; shift 2 ;;
    --headless)    HEADLESS=1;    shift ;;
    --no-headless) HEADLESS=0;    shift ;;
    *)
      echo "Unknown option: $1"
      echo "Usage: $0 [--port N] [--plc-host HOST] [--plc-port N] [--headless]"
      exit 1
      ;;
  esac
done

JETSON_IP="$(hostname -I 2>/dev/null | awk '{print $1}' || echo "localhost")"

echo "╔═════════════════════════════════════════════════════════════╗"
echo "║      Agrobot — AMR ↔ PLC Handshake Dashboard          ║"
echo "╚═════════════════════════════════════════════════════════════╝"
echo ""
echo "  PLC → AMR  (read via FC04):"
echo "    %MW5100  Auger status   bit0=Seq Start, bit1=Clear of Ground, bit2=Cycle Complete"
echo "    %MW5101  Planter status bit0=Seq Start, bit1=Clear of Ground, bit2=Cycle Complete"
echo ""
echo "  AMR → PLC  (write via FC06):"
echo "    %MW5110  Auger command  bit1=Auger Start Sequence"
echo "    %MW5111  Planter cmd    bit1=Planter Start Sequence"
echo "    %MW5112  AMR state      bit0=Stationary, bit1=Moving"
echo ""
echo "  PLC:        $PLC_HOST:$PLC_PORT"
echo "  Local:      http://localhost:$PORT"
echo "  Network:    http://$JETSON_IP:$PORT"
echo ""

if [[ "$HEADLESS" != "1" ]]; then
  URL="http://localhost:$PORT"
  if command -v xdg-open &>/dev/null && [[ -n "${DISPLAY:-}" ]]; then
    (sleep 1.5 && xdg-open "$URL") &
  elif command -v gnome-open &>/dev/null; then
    (sleep 1.5 && gnome-open "$URL") &
  fi
fi

exec python3 "$SCRIPT_DIR/dashboard/amr_plc_serve.py" \
  --port     "$PORT"     \
  --plc-host "$PLC_HOST" \
  --plc-port "$PLC_PORT"
