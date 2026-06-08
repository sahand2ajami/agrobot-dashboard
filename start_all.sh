#!/bin/bash

# 颜色
GREEN="\033[32m"
PLAIN="\033[0m"

# ── agrobot-chassis dev helper (full ROS stack + RViz). Not used by the Jackal. ──
_SD="$(cd "$(dirname "$0")" && pwd)"
_CH="$(python3 -c "import sys; sys.path.insert(0,'$_SD/dashboard'); import chassis; print(chassis.resolve_name())" 2>/dev/null || echo agrobot)"
if [ "$_CH" != "agrobot" ]; then
    echo "start_all.sh is a agrobot-chassis dev helper, but the active chassis is '$_CH'."
    echo "  The Jackal drives itself over ROS — use: ./launch_dashboard.sh --chassis jackal"
    exit 1
fi

echo -e "${GREEN}🚀 小车全系统 一键启动${PLAIN}"

# micro_ros_agent removed: robot firmware uses Modbus RTU at 38400 baud,
# not micro-ROS. The robot_base_node handles serial communication directly.

# 1. 底盘节点（串口Modbus驱动）
gnome-terminal --tab --title="底盘仪表盘" -- bash -c "
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch avatar_robot_base robot_launch.py
exec bash
"
sleep 2

# 2. RViz
gnome-terminal --tab --title="RViz" -- bash -c "
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run rviz2 rviz2 -d ~/ros2_ws/install/avatar_robot_base/share/avatar_robot_base/rviz/robot_config.rviz
exec bash
"

echo -e "${GREEN}✅ 启动完成！${PLAIN}"
