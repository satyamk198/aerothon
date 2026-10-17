"""Run the full autonomous mission: takeoff -> move forward.

Start SITL first (sitl.launch.py), then:
    ros2 launch mission_management mission.launch.py
    ros2 launch mission_management mission.launch.py target_altitude:=5.0 distance:=10.0 speed:=1.0

The mission manager waits until PX4 and both task nodes are ready, then runs every task
in order with no further input.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def float_arg(name):
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("target_altitude", default_value="10.0", description="Takeoff height (m)"),
        DeclareLaunchArgument("distance", default_value="10.0", description="Forward move distance (m)"),
        DeclareLaunchArgument("speed", default_value="1.0", description="Forward move speed (m/s)"),

        Node(
            package="takeoff", executable="takeoff_node", output="screen",
            parameters=[{"target_altitude": float_arg("target_altitude")}],
        ),
        Node(
            package="qr_detector", executable="move_forward", output="screen",
            parameters=[{"distance": float_arg("distance"), "speed": float_arg("speed")}],
        ),
        Node(package="mission_management", executable="mission_management_node", output="screen"),
    ])
