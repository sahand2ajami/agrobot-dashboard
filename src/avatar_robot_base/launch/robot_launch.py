from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取URDF路径
    pkg_path = get_package_share_directory('avatar_robot_base')
    urdf_path = os.path.join(pkg_path, 'urdf', 'robot.urdf')

    return LaunchDescription([
        # 1. 发布机器人模型TF（核心！解决RobotModel报错）
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            arguments=[urdf_path],
            output='screen'
        ),
        # 2. 底盘通信节点
        Node(package='avatar_robot_base', executable='robot_base_node', output='screen'),
        # 3. 里程计解算
        Node(package='avatar_robot_base', executable='odom_calculation', output='screen'),
        # 4. 轨迹发布
        Node(package='avatar_robot_base', executable='path_publisher', output='screen'),
        # 5. 里程清零服务
        Node(package='avatar_robot_base', executable='reset_service', output='screen'),
    ])
