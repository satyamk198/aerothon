import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from px4_msgs.msg import (
    OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleCommandAck,
    VehicleLocalPosition, VehicleStatus,
)

from takeoff.takeoff_node import (
    ACK_REJECTED, ACK_TIMEOUT, COMMAND_QOS, LATCHED_QOS, PX4_QOS, px4_topic, task_status,
    MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_LOITER,
)

# DO_SET_MODE param2 for Offboard, see px4_custom_mode.h
PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6.0

SETPOINT_RATE = 10.0  # Hz; PX4 leaves offboard if setpoints arrive slower than 2 Hz
WARMUP_SETPOINTS = 10  # PX4 only accepts OFFBOARD once setpoints are already streaming
POSITION_TOLERANCE = 0.3  # m
SPEED_TOLERANCE = 0.3  # m/s
STEP_TIMEOUT = {"WARMUP": 5.0, "SET_OFFBOARD": 10.0, "SET_HOLD": 10.0}  # s; MOVE depends on distance

NAN3 = [float("nan")] * 3


class MoveForwardNode(Node):
    def __init__(self):
        super().__init__("move_forward_node")

        self.distance = self.declare_parameter("distance", 10.0).value
        self.speed = self.declare_parameter("speed", 1.0).value  # m/s the setpoint advances
        if self.distance <= 0.0 or self.speed <= 0.0:
            raise ValueError("distance and speed must be > 0")

        self.status = None  # latest VehicleStatus
        self.local_pos = None  # latest VehicleLocalPosition (NED)

        self.start = None  # (x, y, z) when the move began
        self.yaw = float("nan")  # heading held during the move; "forward" is along it
        self.target = None  # (x, y) final position
        self.setpoint = None  # [x, y, z] currently commanded
        self.progress = 0.0  # m the commanded setpoint has advanced along the path
        self.warmup_count = 0

        self.step = "IDLE"  # IDLE -> WARMUP -> SET_OFFBOARD -> MOVE -> SET_HOLD -> DONE / FAILED
        self.step_started = None
        self.pending_cmd = None  # command id awaiting a VehicleCommandAck
        self.cmd_sent_at = None
        self.cmd_accepted = False
        self.command_sub = None

        # PX4 <-> ROS 2 via the uXRCE-DDS agent
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode, px4_topic("/fmu/in/offboard_control_mode", OffboardControlMode), PX4_QOS
        )
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, px4_topic("/fmu/in/trajectory_setpoint", TrajectorySetpoint), PX4_QOS
        )
        self.command_pub = self.create_publisher(
            VehicleCommand, px4_topic("/fmu/in/vehicle_command", VehicleCommand), PX4_QOS
        )
        self.create_subscription(
            VehicleStatus, px4_topic("/fmu/out/vehicle_status", VehicleStatus),
            self.status_cb, PX4_QOS,
        )
        self.create_subscription(
            VehicleLocalPosition, px4_topic("/fmu/out/vehicle_local_position", VehicleLocalPosition),
            self.local_pos_cb, PX4_QOS,
        )
        self.create_subscription(
            VehicleCommandAck, px4_topic("/fmu/out/vehicle_command_ack", VehicleCommandAck),
            self.ack_cb, PX4_QOS,
        )

        self.status_pub = self.create_publisher(String, "/task_status", LATCHED_QOS)

        self.get_logger().info("Waiting for PX4 (vehicle_status, vehicle_local_position)...")
        self.create_timer(1.0 / SETPOINT_RATE, self.update)

    # ---------------- callbacks ----------------

    def status_cb(self, msg: VehicleStatus):
        self.status = msg
        self.subscribe_commands_when_ready()

    def local_pos_cb(self, msg: VehicleLocalPosition):
        self.local_pos = msg
        self.subscribe_commands_when_ready()

    def subscribe_commands_when_ready(self):
        # The mission manager treats "subscribed to /task_command" as "move_forward node is ready".
        if self.command_sub is None and self.status is not None and self.local_pos is not None:
            self.command_sub = self.create_subscription(
                String, "/task_command", self.command_callback, COMMAND_QOS
            )
            self.get_logger().info("PX4 connected. Waiting for 'move_forward' on /task_command.")

    def ack_cb(self, msg: VehicleCommandAck):
        if self.pending_cmd is None or msg.command != self.pending_cmd:
            return
        if msg.result == VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED:
            self.cmd_accepted = True
        elif msg.result in ACK_REJECTED:
            self.fail(f"PX4 {ACK_REJECTED[msg.result]} command {msg.command}")
        # IN_PROGRESS: keep waiting

    def command_callback(self, msg: String):
        if msg.data.strip() != "move_forward":
            return
        if self.step not in ("IDLE", "FAILED"):
            self.get_logger().warn(f"Move already in progress/done (step={self.step}).")
            return
        self.get_logger().info("Move forward command received. Executing...")

        pos = self.local_pos
        if not self.armed():
            self.fail("vehicle is not armed")
            return
        if not (pos.xy_valid and pos.z_valid and math.isfinite(pos.heading)):
            self.fail("local position/heading not valid")
            return

        self.start = (pos.x, pos.y, pos.z)
        self.yaw = pos.heading
        self.target = (
            pos.x + self.distance * math.cos(self.yaw),
            pos.y + self.distance * math.sin(self.yaw),
        )
        self.setpoint = [pos.x, pos.y, pos.z]  # hold position until offboard is engaged
        self.progress = 0.0
        self.warmup_count = 0
        self.get_logger().info(
            f"Start N={pos.x:.1f} E={pos.y:.1f} D={pos.z:.1f}, heading {math.degrees(self.yaw):.0f} deg -> "
            f"target N={self.target[0]:.1f} E={self.target[1]:.1f} ({self.distance:.1f} m forward)"
        )
        self.go_to("WARMUP")

    # ---------------- helpers ----------------

    def armed(self):
        return self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED

    def nav_state(self):
        return self.status.nav_state

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

    def request_hold(self):
        self.send_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            param2=PX4_CUSTOM_MAIN_MODE_AUTO,
            param3=PX4_CUSTOM_SUB_MODE_AUTO_LOITER,
        )

    def publish_setpoint(self):
        timestamp = self.get_clock().now().nanoseconds // 1000

        mode = OffboardControlMode()
        mode.timestamp = timestamp
        mode.position = True
        self.offboard_mode_pub.publish(mode)

        sp = TrajectorySetpoint()
        sp.timestamp = timestamp
        sp.position = [float(v) for v in self.setpoint]
        sp.velocity = NAN3
        sp.acceleration = NAN3
        sp.jerk = NAN3
        sp.yaw = float(self.yaw)
        sp.yawspeed = float("nan")
        self.setpoint_pub.publish(sp)

    def reached_target(self):
        pos = self.local_pos
        if not pos.xy_valid:
            return False
        dist = math.hypot(self.target[0] - pos.x, self.target[1] - pos.y)
        return dist < POSITION_TOLERANCE and math.hypot(pos.vx, pos.vy) < SPEED_TOLERANCE

    # ---------------- state machine ----------------

    def go_to(self, step):
        self.step = step
        self.step_started = self.get_clock().now()
        self.pending_cmd = None
        self.get_logger().info(f"-> {step}")

    def elapsed(self):
        return (self.get_clock().now() - self.step_started).nanoseconds / 1e9

    def step_timeout(self):
        if self.step == "MOVE":
            return self.distance / self.speed + 30.0
        return STEP_TIMEOUT[self.step]

    def fail(self, reason):
        self.get_logger().error(f"Move forward FAILED in {self.step}: {reason}")
        # Setpoint streaming stops now; leave offboard ourselves rather than relying on PX4's failsafe.
        if self.status is not None and self.nav_state() == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.pending_cmd = None
            self.request_hold()
        self.step = "FAILED"
        self.pending_cmd = None
        self.status_pub.publish(task_status("move_forward", False))

    def done(self, message):
        self.get_logger().info(message)
        self.step = "DONE"
        self.pending_cmd = None
        self.status_pub.publish(task_status("move_forward", True))

    def update(self):
        if self.step in ("IDLE", "DONE", "FAILED"):
            return

        if self.elapsed() > self.step_timeout():
            self.fail(f"timed out after {self.step_timeout():.0f}s")
            return

        if not self.armed():
            self.fail("vehicle disarmed")
            return

        if self.step == "WARMUP":
            self.warmup_count += 1
            if self.warmup_count >= WARMUP_SETPOINTS:
                self.go_to("SET_OFFBOARD")

        elif self.step == "SET_OFFBOARD":
            if self.nav_state() == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self.get_logger().info(f"OFFBOARD engaged. Moving forward at {self.speed:.1f} m/s...")
                self.go_to("MOVE")
            else:
                self.send_command(
                    VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                    param1=MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    param2=PX4_CUSTOM_MAIN_MODE_OFFBOARD,
                )

        elif self.step == "MOVE":
            if self.nav_state() != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self.fail("vehicle left OFFBOARD (pilot override or failsafe)")
                return
            # Advance the setpoint gradually instead of jumping 10 m, which PX4 would fly at full speed.
            self.progress = min(self.distance, self.progress + self.speed / SETPOINT_RATE)
            self.setpoint[0] = self.start[0] + self.progress * math.cos(self.yaw)
            self.setpoint[1] = self.start[1] + self.progress * math.sin(self.yaw)
            if self.progress >= self.distance and self.reached_target():
                pos = self.local_pos
                self.get_logger().info(f"Reached N={pos.x:.1f} E={pos.y:.1f}. Switching to Hold.")
                self.go_to("SET_HOLD")

        elif self.step == "SET_HOLD":
            if self.nav_state() == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER:
                self.done(f"Moved {self.distance:.1f} m forward. MOVE FORWARD COMPLETE. Vehicle holding.")
                return
            self.request_hold()

        # Keep streaming until PX4 is in Hold, so offboard never loses its setpoints.
        self.publish_setpoint()


def main(args=None):
    rclpy.init(args=args)
    node = MoveForwardNode()
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
