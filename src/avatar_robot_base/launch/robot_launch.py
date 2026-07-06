"""Single parameterized launch for the Agrobot chassis stack.

Replaces the four near-identical robot_launch{,_T3,_T13,_T17E}.py files.

    ros2 launch avatar_robot_base robot_launch.py car_type:=T13

car_type selects both the URDF and the odometry geometry (see
odom_calculation.VARIANTS). Default: $CAR_TYPE, else T13 — the previous
implicit default of T3 gave permanently-zero odometry on the encoder-based
chassis actually in the field.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

URDF_BY_TYPE = {
    "T3":   "robot_T3.urdf",
    "T13":  "robot_T13.urdf",
    "T17E": "robot_T17E.urdf",
}


def _nodes(context):
    car_type = LaunchConfiguration("car_type").perform(context)
    if car_type not in URDF_BY_TYPE:
        raise ValueError(f"unknown car_type '{car_type}' — expected one of "
                         f"{sorted(URDF_BY_TYPE)}")
    pkg_path = get_package_share_directory("avatar_robot_base")
    urdf_path = os.path.join(pkg_path, "urdf", URDF_BY_TYPE[car_type])

    return [
        # Robot model TF (required by RViz's RobotModel display)
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             arguments=[urdf_path], output="screen"),
        # Chassis Modbus driver
        Node(package="avatar_robot_base", executable="robot_base_node",
             output="screen"),
        # Dead-reckoning odometry
        Node(package="avatar_robot_base", executable="odom_calculation",
             parameters=[{"car_type": car_type}], output="screen"),
        # Path trace for RViz
        Node(package="avatar_robot_base", executable="path_publisher",
             output="screen"),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "car_type",
            default_value=os.environ.get("CAR_TYPE", "T13"),
            description="Chassis variant: T3 | T13 | T17E"),
        OpaqueFunction(function=_nodes),
    ])
