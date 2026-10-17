import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String
from px4_msgs.msg import (
    VehicleCommand, VehicleCommandAck, VehicleStatus, VehicleGlobalPosition, VehicleLandDetected,
)

# Shared with mission_management_node so latched messages reach late joiners.
LATCHED_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)

# /task_command keeps only the latest command, so a node that (re)starts late is replayed
# the current task only, never tasks that already finished.
COMMAND_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)


def task_status(task, ok):
    """/task_status payload, e.g. 'takeoff:ok' or 'move_forward:failed'."""
    return String(data=f"{task}:{'ok' if ok else 'failed'}")


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


# DO_SET_MODE params for Hold (AUTO.LOITER), see px4_custom_mode.h
MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1.0
PX4_CUSTOM_MAIN_MODE_AUTO = 4.0
PX4_CUSTOM_SUB_MODE_AUTO_LOITER = 3.0

ALTITUDE_TOLERANCE = 0.5  # m
ACK_TIMEOUT = 1.0  # s; /fmu/in is best-effort, resend a command if it was not acknowledged
STEP_TIMEOUT = {"SET_MODE": 10.0, "ARM": 10.0, "TAKEOFF": 120.0}  # s

ACK_REJECTED = {
    VehicleCommandAck.VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED: "temporarily rejected",
    VehicleCommandAck.VEHICLE_CMD_RESULT_DENIED: "denied",
    VehicleCommandAck.VEHICLE_CMD_RESULT_UNSUPPORTED: "unsupported",
    VehicleCommandAck.VEHICLE_CMD_RESULT_FAILED: "failed",
    VehicleCommandAck.VEHICLE_CMD_RESULT_CANCELLED: "cancelled",
}


class SimpleTakeoffNode(Node):
    def __init__(self):
        super().__init__("simple_takeoff_node")

        self.target_altitude = self.declare_parameter("target_altitude", 10.0).value
        if self.target_altitude <= ALTITUDE_TOLERANCE:
            raise ValueError(f"target_altitude must be > {ALTITUDE_TOLERANCE} m")

        self.status = None  # latest VehicleStatus
        self.landed = None  # from VehicleLandDetected
        self.amsl = float("nan")  # VehicleGlobalPosition.alt, same value PX4 Navigator uses
        self.target_amsl = float("nan")

        self.step = "IDLE"  # IDLE -> SET_MODE -> ARM -> TAKEOFF -> DONE / FAILED
        self.step_started = None
        self.pending_cmd = None  # command id awaiting a VehicleCommandAck
        self.cmd_sent_at = None
        self.cmd_accepted = False
        self.command_sub = None

        # PX4 <-> ROS 2 via the uXRCE-DDS agent
        self.command_pub = self.create_publisher(
            VehicleCommand, px4_topic("/fmu/in/vehicle_command", VehicleCommand), PX4_QOS
        )
        self.create_subscription(
            VehicleStatus, px4_topic("/fmu/out/vehicle_status", VehicleStatus),
            self.status_cb, PX4_QOS,
        )
        self.create_subscription(
            VehicleGlobalPosition, px4_topic("/fmu/out/vehicle_global_position", VehicleGlobalPosition),
            self.global_pos_cb, PX4_QOS,
        )
        self.create_subscription(
            VehicleLandDetected, px4_topic("/fmu/out/vehicle_land_detected", VehicleLandDetected),
            self.land_cb, PX4_QOS,
        )
        self.create_subscription(
            VehicleCommandAck, px4_topic("/fmu/out/vehicle_command_ack", VehicleCommandAck),
            self.ack_cb, PX4_QOS,
        )

        self.status_pub = self.create_publisher(String, "/task_status", LATCHED_QOS)

        self.get_logger().info("Waiting for PX4 (/fmu/out/vehicle_status)...")
        self.create_timer(0.2, self.update)

    # ---------------- callbacks ----------------

    def status_cb(self, msg: VehicleStatus):
        self.status = msg
        if self.command_sub is None:
            # Subscribe to commands only once PX4 is reachable: the mission manager
            # treats "someone is subscribed to /task_command" as "takeoff node is ready".
            self.command_sub = self.create_subscription(
                String, "/task_command", self.command_callback, COMMAND_QOS
            )
            self.get_logger().info("PX4 connected. Waiting for 'takeoff' on /task_command.")

    def global_pos_cb(self, msg: VehicleGlobalPosition):
        self.amsl = msg.alt if msg.alt_valid else float("nan")

    def land_cb(self, msg: VehicleLandDetected):
        self.landed = msg.landed

    def ack_cb(self, msg: VehicleCommandAck):
        if self.pending_cmd is None or msg.command != self.pending_cmd:
            return
        if msg.result == VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED:
            self.cmd_accepted = True
            if self.pending_cmd == VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF:
                self.get_logger().info(f"Takeoff accepted. Climbing to {self.target_altitude} m...")
        elif msg.result in ACK_REJECTED:
            hint = "; check PX4 pre-arm messages" if self.step == "ARM" else ""
            self.fail(f"PX4 {ACK_REJECTED[msg.result]} command {msg.command}{hint}")
        # IN_PROGRESS: keep waiting

    def command_callback(self, msg: String):
        if msg.data.strip() != "takeoff":
            return
        if self.step not in ("IDLE", "FAILED"):
            self.get_logger().warn(f"Takeoff already in progress/done (step={self.step}).")
            return
        self.get_logger().info("Takeoff command received. Executing...")
        self.target_amsl = float("nan")
        self.go_to("SET_MODE")

    # ---------------- helpers ----------------

    def armed(self):
        return self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED

    def send_command(self, command, **params):
        """Publish a VehicleCommand, resending it if PX4 has not acknowledged it within ACK_TIMEOUT."""
        now = self.get_clock().now()
        if self.pending_cmd == command and (
            self.cmd_accepted or (now - self.cmd_sent_at).nanoseconds / 1e9 < ACK_TIMEOUT
        ):
            return
        msg = VehicleCommand()
        msg.command = command
        for name, value in params.items():
            setattr(msg, name, float(value))
        msg.target_system = self.status.system_id
        msg.target_component = self.status.component_id
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = now.nanoseconds // 1000
        self.command_pub.publish(msg)
        self.pending_cmd = command
        self.cmd_sent_at = now
        self.cmd_accepted = False

    # ---------------- state machine ----------------

    def go_to(self, step):
        self.step = step
        self.step_started = self.get_clock().now()
        self.pending_cmd = None
        self.get_logger().info(f"-> {step}")

    def elapsed(self):
        return (self.get_clock().now() - self.step_started).nanoseconds / 1e9

    def fail(self, reason):
        self.get_logger().error(f"Takeoff FAILED in {self.step}: {reason}")
        self.step = "FAILED"
        self.pending_cmd = None
        self.status_pub.publish(task_status("takeoff", False))

    def done(self, message):
        self.get_logger().info(message)
        self.step = "DONE"
        self.pending_cmd = None
        self.status_pub.publish(task_status("takeoff", True))

    def update(self):
        if self.step in ("IDLE", "DONE", "FAILED"):
            return

        if self.elapsed() > STEP_TIMEOUT[self.step]:
            self.fail(f"timed out after {STEP_TIMEOUT[self.step]:.0f}s")
            return

        if self.status is None or self.landed is None:
            return  # no PX4 telemetry yet (e.g. latched command right after a restart)

        if self.step == "SET_MODE":
            # The command is latched, so a restarted node receives "takeoff" again.
            # If we're already flying, report success instead of climbing further.
            if self.armed() and not self.landed:
                self.done("Already airborne; not taking off again.")
                return
            if self.status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER:
                self.go_to("ARM")
            else:
                self.send_command(
                    VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                    param1=MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    param2=PX4_CUSTOM_MAIN_MODE_AUTO,
                    param3=PX4_CUSTOM_SUB_MODE_AUTO_LOITER,
                )

        elif self.step == "ARM":
            if self.armed():
                self.get_logger().info("Vehicle ARMED.")
                self.go_to("TAKEOFF")
                self.update()  # send takeoff now: PX4 auto-disarms after COM_DISARM_PRFLT (10 s)
            elif math.isnan(self.amsl):
                return  # need altitude before arming, so takeoff can follow immediately
            else:
                self.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)

        elif self.step == "TAKEOFF":
            if not self.armed():
                self.fail("vehicle disarmed (PX4 auto-disarm COM_DISARM_PRFLT, or manual disarm)")
            elif self.pending_cmd != VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF or not self.cmd_accepted:
                if math.isnan(self.target_amsl):
                    if math.isnan(self.amsl):
                        return  # altitude briefly invalid; the disarm check above still runs
                    self.target_amsl = self.amsl + self.target_altitude
                    self.get_logger().info(
                        f"Sending takeoff to {self.target_amsl:.1f} m AMSL "
                        f"(+{self.target_altitude:.1f} m)"
                    )
                self.send_command(
                    VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF,
                    param4=float("nan"),  # yaw: ignored by PX4 for multicopter takeoff
                    param5=float("nan"),  # lat/lon NaN = current position
                    param6=float("nan"),
                    param7=self.target_amsl,  # altitude AMSL
                )
            elif self.amsl >= self.target_amsl - ALTITUDE_TOLERANCE:
                self.done(f"Reached {self.amsl:.1f} m AMSL. TAKEOFF COMPLETE.")


def main(args=None):
    rclpy.init(args=args)
    node = SimpleTakeoffNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
