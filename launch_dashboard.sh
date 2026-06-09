#!/usr/bin/env bash
# Unified dual-robot teleoperation dashboard launcher.
#
# One dashboard, two chassis. The active chassis decides which services start and
# how the dashboard talks to the base:
#
#   agrobot   — RealSense D435i + GNSS + YOLOv8 detector + robot_base_node (Modbus RTU
#            over serial) + rosbridge + dashboard HTTP server.
#            cmd chain: browser -> serve.py -> /avatar_robot/speed_cmd -> robot_base_node -> Modbus
#
#   jackal — LAN setup (host IP on eno1, ROS domain) + GNSS + dashboard HTTP server.
#            The Jackal's onboard stack drives the motors and publishes its camera.
#            cmd chain: browser -> serve.py -> /jackal1/cmd_vel (geometry_msgs/Twist) -> Jackal
#
# Chassis selection (highest priority first):
#   1. --chassis <name>        (this script / serve.py)
#   2. $ROBOT_CHASSIS
#   3. config/active_chassis.yaml
#
# Usage:
#   ./launch_dashboard.sh                      # use config/active_chassis.yaml
#   ./launch_dashboard.sh --chassis jackal
#   ./launch_dashboard.sh --chassis agrobot --port 8080

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Which dashboard server module to run (overridden by launch_dashboard_wide.sh).
SERVE_PY="${SERVE_PY:-dashboard/serve.py}"

# ── Parse --chassis / --port / --rear-camera (both "--x val" and "--x=val") ───
CLI_CHASSIS=""
CLI_REAR=""
PORT=8766
prev=""
for arg in "$@"; do
  case "$arg" in
    --chassis=*)      CLI_CHASSIS="${arg#--chassis=}" ;;
    --port=*)         PORT="${arg#--port=}" ;;
    --rear-camera=*)  CLI_REAR="${arg#--rear-camera=}" ;;
  esac
  [[ "$prev" == "--chassis" ]]     && CLI_CHASSIS="$arg"
  [[ "$prev" == "--port"    ]]     && PORT="$arg"
  [[ "$prev" == "--rear-camera" ]] && CLI_REAR="$arg"
  prev="$arg"
done

# ── ROS 2 environment ─────────────────────────────────────────────────────────
set +u
[[ -z "${ROS_DISTRO:-}" ]] && source /opt/ros/humble/setup.bash
[[ -f "$SCRIPT_DIR/install/setup.bash" ]]   && source "$SCRIPT_DIR/install/setup.bash"
[[ -f "$HOME/ros2_ws/install/setup.bash" ]] && source "$HOME/ros2_ws/install/setup.bash"
set -u

# ── Resolve the active chassis and read its parameters (via chassis.py) ───────
CHASSIS="$(python3 -c "import sys; sys.path.insert(0, '$SCRIPT_DIR/dashboard'); import chassis; print(chassis.resolve_name(sys.argv[1] or None))" "$CLI_CHASSIS")"
export ROBOT_CHASSIS="$CHASSIS"

eval "$(python3 - "$SCRIPT_DIR/dashboard" "$CHASSIS" <<'PY'
import sys
sys.path.insert(0, sys.argv[1])
import chassis
c = chassis.load(sys.argv[2])
vals = {
    'CH_COMMS':   c.comms,
    'CH_DOMAIN':  c.ros_domain_id,
    'CH_IFACE':   c.host_iface,
    'CH_HOSTIP':  c.host_ip,
    'CH_ROBOTIP': c.robot_ip,
    'CH_VARIANT': c.chassis_variant,
    'CH_DESC':    c.description,
}
for k, v in vals.items():
    print(f"{k}='{'' if v is None else v}'")
PY
)"

# ── Logging ───────────────────────────────────────────────────────────────────
LOG_DIR="$SCRIPT_DIR/logs/dashboard"
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/$(date '+%Y-%m-%d_%H-%M-%S')_${CHASSIS}_dashboard.log"
exec > >(tee -a "$LOG_FILE") 2>&1
log() { echo "[$(date '+%Y-%m-%d %H:%M:%S')] $*"; }

log "=================================="
log "  Dual-Robot Dashboard — chassis: $CHASSIS"
log "  $CH_DESC"
log "=================================="
log "  Log file : $LOG_FILE"
log "  Server   : $SERVE_PY  (port $PORT)"

# ── Cleanup on exit (kills the superset of services; harmless if absent) ──────
cleanup() {
  trap '' INT TERM EXIT
  exec >/dev/tty 2>&1 || exec >/dev/null 2>&1
  echo ""
  echo "[$(date '+%Y-%m-%d %H:%M:%S')] Stopping services..."
  for proc in "dashboard/serve" "gnss_p9_read.py" "object_detector.py" \
              "robot_base_node" "realsense2_camera_node" "rosbridge_websocket"; do
    pkill -TERM -f "$proc" 2>/dev/null || true
  done
  sleep 2
  for proc in "dashboard/serve" "gnss_p9_read.py" "object_detector.py" \
              "robot_base_node" "realsense2_camera_node" "rosbridge_websocket"; do
    pkill -KILL -f "$proc" 2>/dev/null || true
  done
  exit 0
}
trap cleanup INT TERM EXIT

# ── Pre-flight: free port $PORT so re-running this script cleanly takes over an
#    existing dashboard instead of crashing with "Address already in use" (which
#    leaves the page with both camera feeds black). Port-specific so a second
#    dashboard on another --port is left untouched.
if command -v fuser &>/dev/null && ss -ltn 2>/dev/null | grep -q ":$PORT "; then
  log "Port $PORT already in use — stopping the previous instance on it..."
  fuser -k -TERM "$PORT/tcp" 2>/dev/null || true
  for _ in $(seq 1 10); do
    ss -ltn 2>/dev/null | grep -q ":$PORT " || break
    sleep 0.5
  done
  ss -ltn 2>/dev/null | grep -q ":$PORT " && fuser -k -KILL "$PORT/tcp" 2>/dev/null || true
  sleep 1
fi

# ── Logo (gitignored brand asset — copy from Downloads if present) ────────────
LOGO_SRC="$HOME/Downloads/logo_agrobot_robotics/svg/white_transparent.svg"
LOGO_DST="$SCRIPT_DIR/dashboard/logo/svg/white_transparent.svg"
if [[ ! -f "$LOGO_DST" && -f "$LOGO_SRC" ]]; then
  mkdir -p "$(dirname "$LOGO_DST")"
  cp "$LOGO_SRC" "$LOGO_DST"
  sed -i 's/width="2400" height="2400"/width="408" height="58"/' "$LOGO_DST" 2>/dev/null || true
  sed -i 's/viewBox="0 0 630 430"/viewBox="108 183 408 58"/'     "$LOGO_DST" 2>/dev/null || true
  log "Logo copied from Downloads."
fi

# ── Shared: start the GNSS reader (best-effort; skipped if no receiver) ────────
start_gnss() {
  local dev=""
  for _p in /dev/gnss /dev/ttyACM0 /dev/ttyACM1 /dev/ttyUSB1; do
    [[ -e "$_p" ]] && { dev="$_p"; break; }
  done
  mkdir -p "$SCRIPT_DIR/logs/gnss"
  if [[ -n "$dev" ]]; then
    [[ -r "$dev" && -w "$dev" ]] || sudo chmod a+rw "$dev" 2>/dev/null || true
    log "Starting GNSS reader on $dev"
    python3 -u "$SCRIPT_DIR/scripts/gnss_p9_read.py" "$dev" "$SCRIPT_DIR/logs/gnss" \
      2>>"$LOG_FILE" >/dev/null &
    sleep 1
  else
    log "GNSS receiver not found — GPS map will show 'No Data' (plug in P-9 Race to enable)"
  fi
}

# ── Per-chassis service startup ───────────────────────────────────────────────
if [[ "$CHASSIS" == "agrobot" ]]; then
  # The agrobot robot exposes its base over Modbus RTU; we run the full sensor stack.

  log "[agrobot] Starting RealSense D435i (color + aligned depth)"
  ros2 run realsense2_camera realsense2_camera_node --ros-args \
    -r __node:=camera -r __ns:=/camera \
    -p enable_color:=true -p enable_depth:=true -p align_depth.enable:=true \
    -p enable_infra1:=false -p enable_infra2:=false \
    -p enable_accel:=false -p enable_gyro:=false \
    -p initial_reset:=true -p rgb_camera.power_line_frequency:=0 &
  log "  waiting for camera to initialise..."
  sleep 5

  start_gnss

  log "[agrobot] Starting object_detector (YOLOv8s, persons only)"
  python3 -u "$SCRIPT_DIR/scripts/object_detector.py" 2>>"$LOG_FILE" &
  sleep 3

  log "[agrobot] Starting robot_base_node (Modbus RTU → /avatar_robot/speed_cmd)"
  if [[ -e /dev/ttyUSB0 && ( ! -r /dev/ttyUSB0 || ! -w /dev/ttyUSB0 ) ]]; then
    sudo chmod a+rw /dev/ttyUSB0 2>/dev/null || true
  fi
  pkill -f robot_base_node 2>/dev/null && sleep 1 || true
  CAR_TYPE="${CH_VARIANT:-T13}" ros2 run avatar_robot_base robot_base_node &
  sleep 2

  log "[agrobot] Starting rosbridge_websocket on ws://localhost:9090"
  ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090 \
    default_call_service_timeout:=5.0 \
    call_services_in_new_thread:=true \
    send_action_goals_in_new_thread:=true &
  sleep 2

elif [[ "$CHASSIS" == "jackal" ]]; then
  # The Jackal drives itself over ROS 2 (DDS) on a LAN cable; we just need the
  # network on the Jackal's subnet and the right ROS domain.

  export ROS_DOMAIN_ID="${CH_DOMAIN:-0}"
  unset FASTRTPS_DEFAULT_PROFILES_FILE 2>/dev/null || true
  log "[jackal] ROS_DOMAIN_ID=$ROS_DOMAIN_ID  robot=$CH_ROBOTIP"

  if [[ -n "${CH_IFACE:-}" && -n "${CH_HOSTIP:-}" ]]; then
    ip_only="${CH_HOSTIP%%/*}"
    subnet="${ip_only%.*}."
    if ! ip addr show "$CH_IFACE" 2>/dev/null | grep -q "$subnet"; then
      log "[jackal] Adding $CH_HOSTIP to $CH_IFACE (to reach Jackal at $CH_ROBOTIP)"
      sudo ip addr add "$CH_HOSTIP" dev "$CH_IFACE" 2>/dev/null || \
        log "  (could not add IP — may already exist or need sudo)"
    fi
  fi

  start_gnss

else
  log "ERROR: unknown chassis '$CHASSIS' (expected: agrobot | jackal)"
  log "Check config/active_chassis.yaml or pass --chassis agrobot|jackal"
  exit 1
fi

# ── Dashboard HTTP server ─────────────────────────────────────────────────────
URL="http://localhost:${PORT}"
NET_IP="$(hostname -I 2>/dev/null | awk '{print $1}')"
log "Starting dashboard server → $URL  (chassis=$CHASSIS${CLI_REAR:+, rear-camera=$CLI_REAR})"
if [[ -n "$CLI_REAR" ]]; then
  python3 -u "$SCRIPT_DIR/$SERVE_PY" --chassis "$CHASSIS" --port "$PORT" --rear-camera "$CLI_REAR" &
else
  python3 -u "$SCRIPT_DIR/$SERVE_PY" --chassis "$CHASSIS" --port "$PORT" &
fi
SERVER_PID=$!

# Wait for the server to accept connections (up to 30 s), then open a browser.
READY=0
for _ in $(seq 1 30); do
  if curl -s --max-time 1 "$URL" >/dev/null 2>&1; then READY=1; break; fi
  sleep 1
done

if [[ "$READY" == "1" ]]; then
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
else
  log "Server did not respond within 30 s — check the log above"
fi

log "Press Ctrl-C to stop all services."
wait "$SERVER_PID"
