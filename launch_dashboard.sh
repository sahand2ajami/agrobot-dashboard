#!/usr/bin/env bash
# Unified dual-robot teleoperation dashboard launcher.
#
# One dashboard, two chassis. The active chassis decides which services start and
# how the dashboard talks to the base:
#
#   agrobot   — RealSense D435i (color) + GNSS + robot_base_node (Modbus RTU over
#            serial) + rosbridge + dashboard HTTP server. Person detection runs
#            on-demand inside the dashboard (ZED front feed), not as a separate node.
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
#   ./launch_dashboard.sh --headless           # serve only; don't open a browser on
#                                              #   the Jetson — view it from your laptop
#                                              #   at the "Network" URL printed below.
#   ./launch_dashboard.sh --no-headless        # force-open a local browser (the default)
#
# Headless can also be defaulted without typing the flag:
#   export DASHBOARD_HEADLESS=1                 # e.g. in ~/.bashrc, to make headless default
# Precedence: --headless/--no-headless flag > $DASHBOARD_HEADLESS > default (open browser).

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Which dashboard server module to run (overridden by launch_dashboard_wide.sh).
SERVE_PY="${SERVE_PY:-dashboard/serve.py}"

# ── Parse --chassis / --port / --rear-camera (both "--x val" and "--x=val") and
#    the valueless --headless / --no-headless flags ─────────────────────────────
CLI_CHASSIS=""
CLI_REAR=""
CLI_HEADLESS=""          # "" = not set on the CLI; falls back to $DASHBOARD_HEADLESS
PORT=8766
while [[ $# -gt 0 ]]; do
  case "$1" in
    --chassis)        CLI_CHASSIS="${2:?--chassis needs a value}"; shift ;;
    --chassis=*)      CLI_CHASSIS="${1#--chassis=}" ;;
    --port)           PORT="${2:?--port needs a value}"; shift ;;
    --port=*)         PORT="${1#--port=}" ;;
    --rear-camera)    CLI_REAR="${2:?--rear-camera needs a value}"; shift ;;
    --rear-camera=*)  CLI_REAR="${1#--rear-camera=}" ;;
    --headless)       CLI_HEADLESS=1 ;;
    --no-headless)    CLI_HEADLESS=0 ;;
    --headless=*)     CLI_HEADLESS="${1#--headless=}" ;;
    *) echo "unknown option: $1 (expected --chassis/--port/--rear-camera/--headless)" >&2
       exit 2 ;;
  esac
  shift
done

# Resolve headless mode: CLI flag > $DASHBOARD_HEADLESS env > default (0 = open a
# local browser, the original behaviour). Accepts 1/true/yes/on (case-insensitive).
_truthy() { case "${1,,}" in 1|true|yes|on) printf 1 ;; *) printf 0 ;; esac; }
if [[ -n "$CLI_HEADLESS" ]]; then
  HEADLESS="$(_truthy "$CLI_HEADLESS")"
else
  HEADLESS="$(_truthy "${DASHBOARD_HEADLESS:-0}")"
fi

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
  # This list must match what this script actually launches — a previous
  # revision killed gnss_p9_read.py (never launched) and leaked the real GNSS
  # reader, leaving it holding the serial port after every exit.
  for proc in "dashboard/serve" "gnss_rtu608bt_read.py" \
              "robot_base_node" "rosbridge_websocket"; do
    pkill -TERM -f "$proc" 2>/dev/null || true
  done
  sleep 2
  for proc in "dashboard/serve" "gnss_rtu608bt_read.py" \
              "robot_base_node" "rosbridge_websocket"; do
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

# ── Logo (copy from Downloads if a newer version is present) ─────────────────
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
  # Use ONLY a dedicated GNSS device node — never a bare /dev/ttyUSB*/ttyACM*.
  # Those are where the chassis (robot base) lives; grabbing one here would put
  # two masters on the same serial line and knock the chassis offline (the
  # exact failure we hit before). A USB receiver must be pinned to /dev/gnss by
  # a udev rule (config/udev/99-agrobot-serial.rules); Bluetooth binds to
  # /dev/rfcomm0. If neither exists the GPS map just shows "No Data" — safe.
  for _p in /dev/gnss /dev/rfcomm0; do
    [[ -e "$_p" ]] && { dev="$_p"; break; }
  done
  mkdir -p "$SCRIPT_DIR/logs/gnss"
  if [[ -n "$dev" ]]; then
    [[ -r "$dev" && -w "$dev" ]] || sudo chmod a+rw "$dev" 2>/dev/null || true
    log "Starting GNSS reader on $dev"
    python3 -u "$SCRIPT_DIR/scripts/gnss_rtu608bt_read.py" "$dev" "$SCRIPT_DIR/logs/gnss" \
      2>>"$LOG_FILE" >/dev/null &
    sleep 1
  else
    log "GNSS receiver not found — GPS map will show 'No Data' (plug in GeoAstra RTU608BT via USB or Bluetooth to enable)"
  fi
}

# ── Shared: put the Jetson on the robot's wired subnet WITHOUT stealing WiFi ────
# The robot device (agrobot PLC at robot_ip on eno1; or the Jackal) lives on a wired
# subnet. Normally we add the host /24 so that whole subnet is on-link via eno1.
#
# The catch: field WiFi is often on the SAME 192.168.1.0/24 (e.g. the Agrobot26010
# router hands out 192.168.1.x). A competing /24 on eno1 then OUTRANKS the WiFi
# route, so every reply to a WiFi client — a phone/laptop viewing the dashboard —
# is routed into eno1, which is down whenever the robot is off. The page then
# loads on the Jetson itself but never from another device (the bug we hit).
#
# So: if some OTHER (WiFi) interface already owns this subnet, we do NOT claim the
# /24. eno1 gets the host address as a /32 and we reach the robot with a single
# /32 host route on eno1. A /32 is more specific than the WiFi /24, so the robot
# is reached over the wire whenever its link is up — while phones, laptops and the
# gateway stay on WiFi. No WiFi conflict → original /24 behaviour (Jackal/DDS
# unchanged). Every step is idempotent and safe to re-run.
setup_robot_subnet() {
  local iface="$1" host_cidr="$2" robot_ip="$3" label="$4"
  [[ -n "$iface" && -n "$host_cidr" ]] || return 0
  local host_ip="${host_cidr%%/*}"
  local subnet="${host_ip%.*}."                 # e.g. 192.168.1.

  # Is another (WiFi) interface already on this subnet?  (exclude eno1 itself)
  local owner
  owner="$(ip -o -4 addr show 2>/dev/null \
             | awk -v s="$subnet" -v i="$iface" '$2 != i && $4 ~ ("^" s) {print $2; exit}')"

  if [[ -z "$owner" ]]; then
    # No conflict — claim the whole /24 on the wired port (original behaviour).
    if ! ip -4 addr show "$iface" 2>/dev/null | grep -qw "$host_ip"; then
      log "[$label] Adding $host_cidr to $iface (to reach robot at ${robot_ip:-?})"
      sudo ip addr add "$host_cidr" dev "$iface" 2>/dev/null || \
        log "  (could not add IP — may already exist or need sudo)"
    fi
  else
    # Conflict — $owner (WiFi) already uses this subnet. Keep the WiFi subnet
    # intact; reach the robot with a host-scoped /32 address + /32 route on $iface.
    log "[$label] ${subnet}0/24 already on $owner (WiFi) — reaching robot via /32 host route on $iface so WiFi clients stay reachable"
    sudo ip addr del "$host_ip/24" dev "$iface" 2>/dev/null || true   # drop any hijacking /24
    sudo ip addr add "$host_ip/32" dev "$iface" 2>/dev/null || true   # ignore 'already exists'
    if [[ -n "$robot_ip" ]]; then
      sudo ip route replace "$robot_ip/32" dev "$iface" src "$host_ip" 2>/dev/null || \
        log "  (could not add host route to $robot_ip on $iface — need sudo?)"
    fi
  fi
}

# ── Per-chassis service startup ───────────────────────────────────────────────
if [[ "$CHASSIS" == "agrobot" ]]; then
  # The agrobot robot exposes its base over Modbus RTU; we run the full sensor stack.

  # The auger/planter/robot-arm PLC sits on the wired LAN (Modbus TCP at
  # robot_ip:502 on eno1). Put the Jetson on that subnet so it can reach the PLC —
  # but never at the cost of WiFi dashboard access when both share 192.168.1.0/24
  # (see setup_robot_subnet). The PLC being off is fine: the /32 host route just
  # sits idle on the down link until the cable/PLC comes up.
  setup_robot_subnet "${CH_IFACE:-}" "${CH_HOSTIP:-}" "${CH_ROBOTIP:-192.168.1.2}" agrobot

  # Cameras: ZED 2i front (pyzed SDK, color+depth) + ZED 2i rear (pyzed SDK, color).
  # Both are opened by serve.py directly — no ROS camera driver needed.
  log "[agrobot] Cameras: ZED 2i front (SDK, depth) + ZED 2i rear (SDK, color-only)"

  start_gnss

  # Person detection now runs ON DEMAND inside the dashboard (serve.py, ZED front
  # feed, yolov8n/FP16 on the GPU) — only while the detection view is open. The old
  # always-on RealSense object_detector.py (a second YOLO + depth) was dropped to cut
  # CPU; re-enable it here if you ever need RealSense distance-to-person again.

  # Always clear any stale robot_base_node a previous run left behind. In mobile
  # mode this is what frees the chassis Modbus speed bus for the wireless remote;
  # in normal mode it avoids two nodes fighting over the serial port.
  pkill -f robot_base_node 2>/dev/null && sleep 1 || true

  # DASHBOARD_NO_BASE=1 (set by launch_mobile.sh): do NOT start robot_base_node.
  # With nothing writing the chassis speed registers, the T10 wireless remote
  # keeps full control of the wheels. Trade-off: no chassis battery / wheel-odom
  # telemetry (both come from this node). The PLC over TCP is unaffected.
  if [[ "${DASHBOARD_NO_BASE:-0}" == "1" ]]; then
    log "[agrobot] MOBILE mode — robot_base_node NOT started; drive the wheels with the wireless remote."
  else
    log "[agrobot] Starting robot_base_node (Modbus RTU → /avatar_robot/speed_cmd)"
    # Resolve the chassis serial port robustly. Prefer the stable udev symlink
    # /dev/agrobot_base (pinned to the chassis FTDI by serial — see
    # config/udev/99-agrobot-serial.rules); fall back to /dev/ttyUSB0 on a box
    # without the rule installed. Wait up to 10 s for the device so a slow USB
    # enumeration never leaves the node opening a not-yet-present port. The
    # resolved path is exported so robot_base_node opens exactly this device.
    BASE_PORT=""
    for _try in $(seq 1 20); do
      for _cand in /dev/agrobot_base /dev/ttyUSB0; do
        [[ -e "$_cand" ]] && { BASE_PORT="$_cand"; break; }
      done
      [[ -n "$BASE_PORT" ]] && break
      [[ "$_try" == 1 ]] && log "[agrobot] Waiting for chassis serial port (/dev/agrobot_base or /dev/ttyUSB0)…"
      sleep 0.5
    done
    if [[ -n "$BASE_PORT" ]]; then
      log "[agrobot] Chassis serial port: $BASE_PORT"
      [[ -r "$BASE_PORT" && -w "$BASE_PORT" ]] || sudo chmod a+rw "$BASE_PORT" 2>/dev/null || true
    else
      log "[agrobot] WARNING: no chassis serial port after 10 s — check the USB cable. robot_base_node will keep retrying."
      BASE_PORT="/dev/agrobot_base"
    fi
    export AGROBOT_BASE_PORT="$BASE_PORT"

    ros2 run avatar_robot_base robot_base_node &
    sleep 2
  fi

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

  setup_robot_subnet "${CH_IFACE:-}" "${CH_HOSTIP:-}" "${CH_ROBOTIP:-}" jackal

  start_gnss

else
  log "ERROR: unknown chassis '$CHASSIS' (expected: agrobot | jackal)"
  log "Check config/active_chassis.yaml or pass --chassis agrobot|jackal"
  exit 1
fi

# ── Dashboard HTTP server ─────────────────────────────────────────────────────
URL="http://localhost:${PORT}"
log "Starting dashboard server → $URL  (chassis=$CHASSIS${CLI_REAR:+, rear-camera=$CLI_REAR})"
if [[ -n "$CLI_REAR" ]]; then
  # shellcheck disable=SC2086  # SERVE_EXTRA is intentionally word-split (e.g. "--wide")
  python3 -u "$SCRIPT_DIR/$SERVE_PY" --chassis "$CHASSIS" --port "$PORT" --rear-camera "$CLI_REAR" ${SERVE_EXTRA:-} &
else
  # shellcheck disable=SC2086
  python3 -u "$SCRIPT_DIR/$SERVE_PY" --chassis "$CHASSIS" --port "$PORT" ${SERVE_EXTRA:-} &
fi
SERVER_PID=$!

# Wait for the server to accept connections (up to 30 s), then open a browser.
READY=0
for _ in $(seq 1 30); do
  if curl -s --max-time 1 "$URL" >/dev/null 2>&1; then READY=1; break; fi
  sleep 1
done

if [[ "$READY" == "1" ]]; then
  if [[ "$HEADLESS" == "1" ]]; then
    log "Server ready — HEADLESS (no local browser opened)."
  else
    log "Server ready — opening local browser at $URL"
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
  log "Server did not respond within 30 s — check the log above"
fi

log "Press Ctrl-C to stop."
# Write the URL directly to /dev/tty so it reaches the terminal regardless
# of the log redirect and regardless of what background processes print after.
# Skips: loopback, link-local, docker/172.*, Tailscale (100.*), IPv6, and the
# wired robot-subnet alias ($CH_HOSTIP on eno1 — the PLC/Jackal side, not a
# dashboard access address). We intentionally do NOT blanket-skip 192.168.* any
# more: field WiFi (e.g. Agrobot26010) lives there, and that's the address a phone
# or laptop must use.  Whatever's left is the real LAN/WiFi address(es).
CH_HOST_ONLY="${CH_HOSTIP%%/*}"
{
  echo ""
  for _ip in $(hostname -I 2>/dev/null); do
    case "$_ip" in
      127.*|169.254.*|172.*|100.*|*:*) continue ;;
      "$CH_HOST_ONLY")                 continue ;;   # eno1 robot-subnet alias, not for browsers
    esac
    echo "  http://${_ip}:${PORT}"
  done
  echo ""
} > /dev/tty 2>/dev/null || true   # no controlling terminal (systemd/nohup/ssh -T): skip, don't die under set -e
wait "$SERVER_PID"
