"""Run the AeroTHON mission on the new stack: flight_controller + task nodes + py_trees mission tree.

Start SITL first (sitl.launch.py), then:
    ros2 launch mission_management autonomy.launch.py
    ros2 launch mission_management autonomy.launch.py reference_qr_path:=/abs/path/to/ref.png  # preload

mission.launch.py (takeoff_node + move_forward + mission_management_node) is the old stack and
keeps working until this one replaces it. Do not run both at the same time.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

TASK_NODES = [
    # perception (qr_detector is added separately: it needs a reference image)
    ("banner_detector", "banner_detector_node"),
    ("obstacle_detector", "obstacle_detector_node"),
    ("red_zone_detector", "red_zone_detector_node"),
    # behaviors
    ("banner_align", "banner_align_node"),
    ("corridor_navigator", "corridor_navigator_node"),
    ("target_search", "target_search_node"),
    ("payload", "payload_node"),
]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "reference_qr_path", default_value="",
                description="Optional reference QR image to preload (bench tests); "
                            "normally captured in flight over the start QR",
            ),
            DeclareLaunchArgument(
                "camera_hfov", default_value="1.74",
                description="Downward camera horizontal field of view, rad (default: the SITL camera)",
            ),
            Node(package="flight_controller", executable="flight_controller_node", output="screen"),
            Node(
                package="target_centering", executable="target_centering_node", output="screen",
                parameters=[{"camera_hfov": ParameterValue(LaunchConfiguration("camera_hfov"), value_type=float)}],
            ),
            Node(
                package="qr_detector", executable="qr_detector_node", output="screen",
                parameters=[{"reference_qr_path": LaunchConfiguration("reference_qr_path")}],
            ),
        ]
        + [Node(package=pkg, executable=exe, output="screen") for pkg, exe in TASK_NODES]
        + [Node(package="mission_management", executable="mission_tree", output="screen")]
    )
