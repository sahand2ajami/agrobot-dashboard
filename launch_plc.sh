#!/usr/bin/env bash
# PLC HMI dashboard — standalone launcher.
#
# Runs a dedicated PLC HMI server on port 8767 (distinct from the main teleoperation
# dashboard on 8766 and the wide-angle dashboard). No ROS required — talks directly
# to the LS Electric PLC over Modbus TCP on the 192.168.1.0/24 LAN.
#
# Usage:
#   ./launch_plc.sh
#   ./launch_plc.sh --port 8770
#   ./launch_plc.sh --plc-host 192.168.1.2
#   ./launch_plc.sh --headless          # skip opening a local browser

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ── Defaults ──────────────────────────────────────────────────────────────────
PORT=8767
PLC_HOST="192.168.1.2"
PLC_PORT=502
IFACE="eno1"
HOST_IP="192.168.1.100/24"
HEADLESS=0

# ── Arg parsing ───────────────────────────────────────────────────────────────
prev=""
for arg in "$@"; do
  case "$arg" in
    --port=*)      PORT="${arg#--port=}"         ;;
    --plc-host=*)  PLC_HOST="${arg#--plc-host=}" ;;
    --headless)    HEADLESS=1                    ;;
    --no-headless) HEADLESS=0                    ;;
  esac
  [[ "$prev" == "--port"     ]] && PORT="$arg"
  [[ "$prev" == "--plc-host" ]] && PLC_HOST="$arg"
  prev="$arg"
done

log() { echo "[$(date '+%Y-%m-%d %H:%M:%S')] $*"; }

log "=============================="
log "  PLC HMI dashboard"
log "  PLC at ${PLC_HOST}:${PLC_PORT}"
log "  HTTP port: ${PORT}"
log "=============================="

# ── Ensure Jetson has an IP on the PLC subnet ─────────────────────────────────
PLC_SUBNET="${PLC_HOST%.*}."
if ! ip addr show "$IFACE" 2>/dev/null | grep -q "$PLC_SUBNET"; then
  log "Adding $HOST_IP to $IFACE (to reach PLC at $PLC_HOST)"
  sudo ip addr add "$HOST_IP" dev "$IFACE" 2>/dev/null || \
    log "  (already present or sudo unavailable — skipping)"
fi

# ── Cleanup ───────────────────────────────────────────────────────────────────
cleanup() {
  trap '' INT TERM EXIT
  echo ""
  log "Stopping PLC HMI..."
  pkill -TERM -f "plc_hmi_serve.py" 2>/dev/null || true
  sleep 1
  pkill -KILL -f "plc_hmi_serve.py" 2>/dev/null || true
  exit 0
}
trap cleanup INT TERM EXIT

# ── Free port if already in use ───────────────────────────────────────────────
if command -v fuser &>/dev/null && ss -ltn 2>/dev/null | grep -q ":$PORT "; then
  log "Port $PORT in use — stopping previous instance..."
  fuser -k -TERM "$PORT/tcp" 2>/dev/null || true
  for _ in $(seq 1 8); do
    ss -ltn 2>/dev/null | grep -q ":$PORT " || break
    sleep 0.5
  done
  ss -ltn 2>/dev/null | grep -q ":$PORT " && \
    fuser -k -KILL "$PORT/tcp" 2>/dev/null || true
  sleep 0.5
fi

# ── Start the HMI server ──────────────────────────────────────────────────────
URL="http://localhost:${PORT}"
log "Starting PLC HMI server → $URL"
python3 -u "$SCRIPT_DIR/dashboard/plc_hmi_serve.py" \
  --port "$PORT" \
  --plc-host "$PLC_HOST" \
  --plc-port "$PLC_PORT" &
SERVER_PID=$!

# ── Wait for ready ────────────────────────────────────────────────────────────
READY=0
for _ in $(seq 1 20); do
  if curl -s --max-time 1 "$URL" >/dev/null 2>&1; then READY=1; break; fi
  sleep 1
done

NET_IP="$(hostname -I 2>/dev/null | awk '{print $1}')"

if [[ "$READY" == "1" ]]; then
  if [[ "$HEADLESS" == "1" ]]; then
    log "Server ready — headless mode."
    for _ip in $(hostname -I 2>/dev/null); do
      case "$_ip" in 127.*|169.254.*|172.1[67].*|172.18.*) continue ;; esac
      log "  →  http://${_ip}:${PORT}"
    done
  else
    log "Server ready — opening browser at $URL"
    log "  Network → http://${NET_IP}:${PORT}"
    if command -v firefox &>/dev/null; then
      setsid firefox --new-window "$URL" </dev/null &>/dev/null &
    elif command -v firefox-esr &>/dev/null; then
      setsid firefox-esr --new-window "$URL" </dev/null &>/dev/null &
    elif command -v chromium-browser &>/dev/null; then
      setsid chromium-browser "$URL" </dev/null &>/dev/null &
    elif command -v xdg-open &>/dev/null; then
      setsid xdg-open "$URL" </dev/null &>/dev/null &
    else
      log "No browser found — open $URL manually"
    fi
  fi
else
  log "Server did not respond in 20 s — check output above."
fi

log "Press Ctrl-C to stop."
wait "$SERVER_PID"
