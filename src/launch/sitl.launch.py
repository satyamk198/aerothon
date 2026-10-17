"""Bring up the SITL environment as described in the README of
https://github.com/Shiva007-RAndom/Qr-and-red-zone-detection ("Run it"):

  Terminal 1: PX4 SITL + Gazebo, aerothon_mission2 world, x500_mono_cam_down at the pad
  Terminal 2: the Gazebo -> ROS camera bridge to /camera/image_raw (from launch/skyscan_qr.launch.py)

plus the micro XRCE-DDS agent (PX4 <-> ROS 2 for our nodes) and QGroundControl.

    ros2 launch mission_management sitl.launch.py
    ros2 launch mission_management sitl.launch.py headless:=true qgc:=false

Needs the README's one-time setup (step 2: models in ~/.simulation-gazebo/models, the world and
the fixed x500_mono_cam_down/model.sdf copied into PX4-Autopilot) and, for the camera bridge,
ros_gz built for Gazebo Harmonic: sudo apt install ros-humble-ros-gzharmonic.
Differences from the README (it targets Jazzy): GZ_CONFIG_PATH is only /usr/share/gz, the
Jazzy-only /opt/ros/jazzy/... entries do not exist on Humble.

Ctrl+C stops everything.
"""
import os
import subprocess

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HOME = os.path.expanduser("~")


def simulation(context):
    px4_dir = LaunchConfiguration("px4_dir").perform(context)
    world = LaunchConfiguration("world").perform(context)
    model = LaunchConfiguration("model").perform(context)

    # README Terminal 1 environment
    gz_env = {
        "GZ_CONFIG_PATH": "/usr/share/gz",
        "GZ_SIM_RESOURCE_PATH": ":".join(filter(None, [
            os.path.join(px4_dir, "Tools", "simulation", "gz", "worlds"),
            os.path.join(px4_dir, "Tools", "simulation", "gz", "models"),
            os.path.join(HOME, ".simulation-gazebo", "models"),
            os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
        ])),
    }
    px4_env = dict(gz_env, PX4_GZ_WORLD=world, PX4_GZ_MODEL_POSE=LaunchConfiguration("spawn_pose").perform(context))
    if LaunchConfiguration("headless").perform(context).lower() in ("true", "1"):
        px4_env["HEADLESS"] = "1"  # PX4 then starts the Gazebo server without the GUI

    # README: "pkill -x px4  # clears an old PX4 if one is still running", then make.
    subprocess.run(["pkill", "-x", "px4"], check=False)
    px4 = ExecuteProcess(
        name="px4_sitl",
        cmd=["make", "px4_sitl", model],  # first build takes several minutes
        cwd=px4_dir,
        additional_env=px4_env,
        output="screen",
        emulate_tty=True,
    )

    # Camera bridge as in the README's launch/skyscan_qr.launch.py.
    # PX4 appends an instance suffix (_0) to the spawned model name.
    camera_topic = f"/world/{world}/model/{model.removeprefix('gz_')}_0/link/camera_link/sensor/camera/image"
    bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge", name="camera_bridge", output="screen",
        arguments=[f"{camera_topic}@sensor_msgs/msg/Image[gz.msgs.Image"],
        remappings=[(camera_topic, "/camera/image_raw")],
        condition=IfCondition(LaunchConfiguration("camera_bridge")),
    )

    def stop_px4_and_gazebo(*_):
        # `make` does not pass Ctrl+C on to PX4, and PX4 starts Gazebo in the background, so
        # stop them directly. Launch starts no new processes during shutdown, hence subprocess.
        # The patterns match only PX4 and the Gazebo server/GUI that PX4 starts. gz (a Ruby
        # script) pads its command line with blanks, hence "( |$)" instead of a plain "$".
        subprocess.run(["pkill", "-INT", "-x", "px4"], check=False)
        subprocess.run(["pkill", "-INT", "-f", f"^gz sim .* -s .*/{world}\\.sdf( |$)"], check=False)
        subprocess.run(["pkill", "-INT", "-f", "^gz sim -g( |$)"], check=False)

    return [
        px4,
        bridge,
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=stop_px4_and_gazebo)])),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("px4_dir", default_value=os.path.join(HOME, "PX4-Autopilot")),
        DeclareLaunchArgument("world", default_value="aerothon_mission2",
                              description="World in PX4-Autopilot/Tools/simulation/gz/worlds"),
        DeclareLaunchArgument("model", default_value="gz_x500_mono_cam_down", description="PX4 make target"),
        DeclareLaunchArgument("spawn_pose", default_value="0,20,0.1,0,0,0",
                              description="Drone spawn x,y,z,roll,pitch,yaw (README: launch pad at 0,20)"),
        DeclareLaunchArgument("headless", default_value="false", description="Gazebo without GUI"),
        DeclareLaunchArgument("camera_bridge", default_value="true",
                              description="Bridge the downward camera to /camera/image_raw"),
        DeclareLaunchArgument("qgc_path", default_value=os.path.join(HOME, "QGroundControl-x86_64.AppImage")),
        DeclareLaunchArgument("qgc", default_value="true", description="Start QGroundControl"),
        DeclareLaunchArgument("agent_port", default_value="8888"),

        ExecuteProcess(
            name="xrce_agent",
            cmd=["MicroXRCEAgent", "udp4", "-p", LaunchConfiguration("agent_port")],
            output="screen",
        ),
        ExecuteProcess(
            name="qgroundcontrol",
            cmd=[LaunchConfiguration("qgc_path")],
            output="log",  # QGC is very chatty; its output goes to ~/.ros/log
            condition=IfCondition(LaunchConfiguration("qgc")),
        ),
        OpaqueFunction(function=simulation),
    ])
