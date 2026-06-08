from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_path = get_package_share_directory('avatar_robot_base')
    urdf_path = os.path.join(pkg_path, 'urdf', 'robot_T3.urdf')

    return LaunchDescription([
        # 1. 机器人模型TF发布
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            arguments=[urdf_path],
            output='screen'
        ),
        # 2. 底盘通信节点
        Node(
            package='avatar_robot_base',
            executable='robot_base_node',
            output='screen'
        ),
        # 3. 里程计解算（正确可执行名，无.py）
        Node(
            package='avatar_robot_base',
            executable='odom_calculation',
            output='screen'
        ),
        # 4. 轨迹发布
        Node(
            package='avatar_robot_base',
            executable='path_publisher',
            output='screen'
        ),
    ])
