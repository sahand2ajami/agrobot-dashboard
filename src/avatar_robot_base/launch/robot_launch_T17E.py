from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = get_package_share_directory('avatar_robot_base')
    urdf = os.path.join(pkg, 'urdf', 'robot_T17E.urdf')

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            arguments=[urdf],
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
