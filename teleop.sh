#!/bin/bash
set -e

WS="$(cd "$(dirname "$0")" && pwd)"

# ── agrobot-chassis dev helper (Modbus driver + terminal WASD). Not for the Jackal.
_CH="$(python3 -c "import sys; sys.path.insert(0,'$WS/dashboard'); import chassis; print(chassis.resolve_name())" 2>/dev/null || echo agrobot)"
if [ "$_CH" != "agrobot" ]; then
    echo "teleop.sh is a agrobot-chassis dev helper, but the active chassis is '$_CH'."
    echo "  The Jackal drives itself over ROS — use: ./launch_dashboard.sh --chassis jackal"
    exit 1
fi

echo "Setting serial port permissions..."
sudo chmod 666 /dev/ttyUSB0

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

if pgrep -f robot_base_node > /dev/null; then
    echo "robot_base_node already running, killing it..."
    pkill -f robot_base_node
    sleep 1
fi

echo "Starting robot base node..."
ros2 run avatar_robot_base robot_base_node &
BASE_PID=$!
sleep 2

echo "Starting teleop (Ctrl+C to quit)..."
ros2 run avatar_robot_base teleop_keyboard

# Clean up on exit
kill $BASE_PID 2>/dev/null
wait $BASE_PID 2>/dev/null
echo "Stopped."
