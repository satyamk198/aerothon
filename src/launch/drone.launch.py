"""Run the mission on the real drone (Raspberry Pi companion computer, no simulator).

    ros2 launch mission_management drone.launch.py photo_dir:=<folder> \
        qr_venv:=$HOME/qr_venv

Starts:
  - MicroXRCEAgent on the flight controller's USB port (PX4 <-> ROS 2)
  - siyi_a8 photo_node: 4K photo loop from the A8 mini's SD card (HTTP media API, port 82) ->
    /camera/image_raw, each photo also saved + geotagged in <photo_dir>/<run>/<mission step>/
  - siyi_a8 gimbal_node: points the A8 mini gimbal straight down (pitch -90 deg), sets the zoom
    (start-up value `zoom`, change in flight with /siyi_gimbal/set_zoom)
  The A8 mini's RTSP stream is not used on the Pi: the ground station views it directly.
  - autonomy.launch.py: flight_controller, detectors, task nodes and the mission tree

One-time setup
  Flight controller (PX4 has no built-in uXRCE-DDS-over-USB option, so this is a workaround):
    - set SYS_USB_AUTO = 0 (stops PX4 starting MAVLink on USB; QGroundControl over USB then
      no longer works, use a telemetry radio)
    - create etc/extras.txt on the flight controller's SD card containing the line
          uxrce_dds_client start -t serial -d /dev/ttyACM0 -b 921600
    - reboot the flight controller
  SIYI A8 mini on Ethernet: give the Pi's Ethernet port a static address in 192.168.144.x
    (not .25, which is the camera's factory address).
  QR detector: qrdet in a venv with system packages visible (Ubuntu 24.04 blocks system pip):
      python3 -m venv --system-site-packages ~/qr_venv
      ~/qr_venv/bin/pip install qrdet pyzbar "numpy<2"
    then pass qr_venv:=$HOME/qr_venv.
"""
import glob
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def qr_venv_path(context):
    """Put the venv's site-packages on PYTHONPATH: ros2 runs nodes with /usr/bin/python3, not the venv."""
    venv = LaunchConfiguration("qr_venv").perform(context)
    if not venv:
        return []
    site = glob.glob(os.path.join(os.path.expanduser(venv), "lib", "python3*", "site-packages"))
    if not site:
        raise RuntimeError(f"qr_venv '{venv}' has no lib/python3*/site-packages")
    return [SetEnvironmentVariable(
        "PYTHONPATH", os.pathsep.join(filter(None, [site[0], os.environ.get("PYTHONPATH", "")])))]


def generate_launch_description():
    autonomy = os.path.join(get_package_share_directory("mission_management"), "launch", "autonomy.launch.py")
    return LaunchDescription([
        DeclareLaunchArgument("reference_qr_path", default_value="",
                              description="Optional reference QR image to preload (bench tests); "
                                          "normally captured in flight over the start QR"),
        DeclareLaunchArgument("qr_venv", default_value="",
                              description="venv with qrdet installed (empty: use system Python packages)"),
        DeclareLaunchArgument("fc_device", default_value="/dev/ttyACM0",
                              description="Flight controller USB serial device"),
        DeclareLaunchArgument("fc_baud", default_value="921600",
                              description="Must match the -b value in the flight controller's etc/extras.txt"),
        DeclareLaunchArgument("gimbal_ip", default_value="192.168.144.25",
                              description="A8 mini address (SIYI factory default)"),
        DeclareLaunchArgument("gimbal_port", default_value="37260", description="SIYI SDK UDP port"),
        DeclareLaunchArgument("zoom", default_value="1.0",
                              description="A8 mini digital zoom at start-up, 1.0 to 6.0 (1.0 = no zoom)"),
        DeclareLaunchArgument("photo_dir",
                              description="Root folder for the 4K photos: <photo_dir>/<run>/<mission step>/ "
                                          "(+ latest link); required"),
        DeclareLaunchArgument("camera_hfov", default_value="1.41",
                              description="A8 mini horizontal field of view (rad) for the photos used by "
                                          "target_centering; 81 deg per SIYI's spec"),
        DeclareLaunchArgument("gimbal_pitch_deg", default_value="-90.0",
                              description="Gimbal pitch to hold, -90 = straight down"),

        OpaqueFunction(function=qr_venv_path),

        ExecuteProcess(
            name="xrce_agent",
            cmd=["MicroXRCEAgent", "serial", "--dev", LaunchConfiguration("fc_device"),
                 "-b", LaunchConfiguration("fc_baud")],
            output="screen",
        ),
        Node(
            package="siyi_a8", executable="photo_node", output="screen",
            parameters=[{
                "camera_ip": LaunchConfiguration("gimbal_ip"),
                "camera_port": ParameterValue(LaunchConfiguration("gimbal_port"), value_type=int),
                "photo_dir": LaunchConfiguration("photo_dir"),
            }],
        ),
        Node(
            package="siyi_a8", executable="gimbal_node", output="screen",
            parameters=[{
                "gimbal_ip": LaunchConfiguration("gimbal_ip"),
                "gimbal_port": ParameterValue(LaunchConfiguration("gimbal_port"), value_type=int),
                "pitch_deg": ParameterValue(LaunchConfiguration("gimbal_pitch_deg"), value_type=float),
                "zoom": ParameterValue(LaunchConfiguration("zoom"), value_type=float),
            }],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(autonomy),
            launch_arguments={
                "reference_qr_path": LaunchConfiguration("reference_qr_path"),
                "camera_hfov": LaunchConfiguration("camera_hfov"),
            }.items(),
        ),
    ])
