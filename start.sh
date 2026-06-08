#!/bin/bash

# ── agrobot-chassis dev helper (chassis + RViz). Not used by the Jackal. ─────────
_SD="$(cd "$(dirname "$0")" && pwd)"
_CH="$(python3 -c "import sys; sys.path.insert(0,'$_SD/dashboard'); import chassis; print(chassis.resolve_name())" 2>/dev/null || echo agrobot)"
if [ "$_CH" != "agrobot" ]; then
    echo "start.sh is a agrobot-chassis dev helper, but the active chassis is '$_CH'."
    echo "  The Jackal drives itself over ROS — use: ./launch_dashboard.sh --chassis jackal"
    exit 1
fi

if [ "$1" != "T3" ] && [ "$1" != "T13" ] && [ "$1" != "T17E" ]; then
    echo -e "\033[31m请指定车型：\033[0m"
    echo "   ./start.sh T3"
    echo "   ./start.sh T13"
    echo "   ./start.sh T17E"
    exit 1
fi

CAR_TYPE=$1

# 1. 启动 micro-ros-agent（正确命令！）
gnome-terminal --tab --title="microROS" -- bash -c "
source /opt/ros/humble/setup.bash
source ~/microros_ws/install/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200
exec bash
"
sleep 2

# 2. 启动底盘
gnome-terminal --tab --title="底盘-$CAR_TYPE" -- bash -c "
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export CAR_TYPE=$CAR_TYPE
ros2 launch avatar_robot_base robot_launch_$CAR_TYPE.py
exec bash
"
sleep 3

# 3. 启动 RViz2
gnome-terminal --tab --title="RViz-$CAR_TYPE" -- bash -c "
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run rviz2 rviz2 -d ~/ros2_ws/install/avatar_robot_base/share/avatar_robot_base/rviz/robot_config.rviz
exec bash
"
