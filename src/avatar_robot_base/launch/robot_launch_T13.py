from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_path = get_package_share_directory('avatar_robot_base')
    urdf_path = os.path.join(pkg_path, 'urdf', 'robot_T13.urdf')

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            arguments=[urdf_path],
            output='screen'
        ),
        Node(
            package='avatar_robot_base',
            executable='robot_base_node',
            output='screen'
        ),
        Node(
            package='avatar_robot_base',
            executable='odom_calculation',
            output='screen'
        ),
        Node(
            package='avatar_robot_base',
            executable='path_publisher',
            output='screen'
        ),
    ])
