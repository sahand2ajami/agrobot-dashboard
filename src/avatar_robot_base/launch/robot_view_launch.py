from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_path = get_package_share_directory('avatar_robot_base')
    urdf_path = os.path.join(pkg_path, 'urdf', 'robot.urdf')

    return LaunchDescription([
        # 发布机器人模型TF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            arguments=[urdf_path],
            output='screen'
        ),

        # RViz2 自动打开
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=[],
            output='screen'
        ),
    ])
