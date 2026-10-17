"""PX4 <-> ROS 2 helpers shared by nodes that talk to PX4 through the uXRCE-DDS agent."""
import math

from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from px4_msgs.msg import VehicleCommandAck

# QoS used by the official PX4 ROS 2 examples for /fmu/* topics.
PX4_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)


def px4_topic(name, msg_type):
    """PX4 v1.16+ appends _v<N> to topics of versioned messages; derive it from px4_msgs."""
    version = getattr(msg_type, "MESSAGE_VERSION", 0)
    return f"{name}_v{version}" if version else name


def wrap_pi(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


# DO_SET_MODE params, see px4_custom_mode.h
MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1.0
PX4_CUSTOM_MAIN_MODE_AUTO = 4.0
PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6.0
PX4_CUSTOM_SUB_MODE_AUTO_LOITER = 3.0

ACK_REJECTED = {
    VehicleCommandAck.VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED: "temporarily rejected",
    VehicleCommandAck.VEHICLE_CMD_RESULT_DENIED: "denied",
    VehicleCommandAck.VEHICLE_CMD_RESULT_UNSUPPORTED: "unsupported",
    VehicleCommandAck.VEHICLE_CMD_RESULT_FAILED: "failed",
    VehicleCommandAck.VEHICLE_CMD_RESULT_CANCELLED: "cancelled",
}
