"""The only node that commands PX4 in OFFBOARD; every task flies the drone through it.

Actions (one goal at a time across all of them):
    ~/takeoff   mission_interfaces/Takeoff  arm, PX4 AUTO takeoff, then engage OFFBOARD holding position
    ~/goto      mission_interfaces/GoTo     fly to a local-NED position or a body-frame offset
    ~/land      mission_interfaces/Land     PX4 AUTO land at the current position, wait for disarm

Topic:
    ~/cmd_vel_body  geometry_msgs/Twist     body-frame velocity, REP-103 FLU: linear.x forward,
                    linear.y left, linear.z up (m/s), angular.z yaw rate counter-clockwise (rad/s).
                    Used only while OFFBOARD is engaged and no goal is active. If commands stop for
                    CMD_VEL_TIMEOUT the vehicle holds its position.

If anything else takes the vehicle out of OFFBOARD (pilot, failsafe), the controller stops
streaming and rejects GoTo goals: control is never taken back without a new Takeoff.
"""
import math
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from px4_msgs.msg import (
    OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleCommandAck,
    VehicleGlobalPosition, VehicleLandDetected, VehicleLocalPosition, VehicleStatus,
)
from mission_interfaces.action import GoTo, Land, Takeoff

from flight_controller.px4_utils import (
    ACK_REJECTED, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, PX4_CUSTOM_MAIN_MODE_AUTO,
    PX4_CUSTOM_MAIN_MODE_OFFBOARD, PX4_CUSTOM_SUB_MODE_AUTO_LOITER, PX4_QOS, px4_topic, wrap_pi,
)

SETPOINT_RATE = 20.0  # Hz; PX4 leaves OFFBOARD if setpoints arrive slower than 2 Hz
OFFBOARD_WARMUP = 1.0  # s of setpoints streamed before requesting OFFBOARD (PX4 requires it)
CMD_VEL_TIMEOUT = 0.5  # s without /cmd_vel_body -> hold position
ACK_TIMEOUT = 1.0  # s; /fmu/in is best-effort, resend a command if it was not acknowledged
COMMAND_TIMEOUT = 5.0  # s to get a command accepted
MODE_TIMEOUT = 5.0  # s for PX4 to report a requested mode / arming state
TAKEOFF_TIMEOUT = 120.0  # s
LAND_TIMEOUT = 120.0  # s
GOTO_EXTRA_TIME = 30.0  # s allowed beyond distance / speed
ALTITUDE_TOLERANCE = 0.5  # m, takeoff
SETTLED_SPEED = 0.3  # m/s, GoTo counts as arrived only below this speed

NAN3 = [float("nan")] * 3


class FlightController(Node):
    def __init__(self):
        super().__init__("flight_controller")

        self.default_speed = self.declare_parameter("default_speed", 1.0).value  # m/s
        self.default_radius = self.declare_parameter("default_acceptance_radius", 0.3).value  # m
        self.max_speed = self.declare_parameter("max_speed", 3.0).value  # m/s, clamps GoTo and cmd_vel

        self.lock = threading.Lock()

        # PX4 telemetry
        self.status = None
        self.local_pos = None
        self.amsl = float("nan")  # VehicleGlobalPosition.alt, same value PX4 Navigator uses
        self.landed = None

        # Setpoint streamed to PX4: None (not streaming), "position" or "velocity"
        self.sp_mode = None
        self.sp_position = list(NAN3)  # local NED
        self.sp_yaw = float("nan")
        self.sp_velocity = list(NAN3)  # local NED
        self.sp_yawspeed = float("nan")
        self.cmd_vel_stamp = 0.0
        self.offboard_owned = False  # we engaged OFFBOARD and still have it
        self.active_goal = None  # name of the running action, if any

        # VehicleCommand acknowledgement
        self.pending_cmd = None
        self.ack_result = None
        self.ack_event = threading.Event()

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
            VehicleStatus, px4_topic("/fmu/out/vehicle_status", VehicleStatus), self.status_cb, PX4_QOS
        )
        self.create_subscription(
            VehicleLocalPosition, px4_topic("/fmu/out/vehicle_local_position", VehicleLocalPosition),
            self.local_pos_cb, PX4_QOS,
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
        self.create_subscription(Twist, "~/cmd_vel_body", self.cmd_vel_cb, 10)

        # Action executes block in their own threads; telemetry callbacks keep running meanwhile.
        actions = ReentrantCallbackGroup()
        for action_type, name, execute, cancel in (
            (Takeoff, "~/takeoff", self.execute_takeoff, self.accept_cancel),
            (GoTo, "~/goto", self.execute_goto, self.accept_cancel),
            (Land, "~/land", self.execute_land, self.reject_cancel),  # aborting a landing is unsafe
        ):
            ActionServer(
                self, action_type, name, execute_callback=execute,
                goal_callback=lambda goal, n=name: self.on_goal(n),
                cancel_callback=cancel, callback_group=actions,
            )

        self.create_timer(1.0 / SETPOINT_RATE, self.stream_setpoint)
        self.get_logger().info("Flight controller ready. Waiting for PX4 telemetry...")

    # ---------------- PX4 callbacks ----------------

    def status_cb(self, msg: VehicleStatus):
        self.status = msg

    def local_pos_cb(self, msg: VehicleLocalPosition):
        self.local_pos = msg

    def global_pos_cb(self, msg: VehicleGlobalPosition):
        self.amsl = msg.alt if msg.alt_valid else float("nan")

    def land_cb(self, msg: VehicleLandDetected):
        self.landed = msg.landed

    def ack_cb(self, msg: VehicleCommandAck):
        if msg.command == self.pending_cmd:
            self.ack_result = msg.result
            self.ack_event.set()

    # ---------------- velocity commands ----------------

    def cmd_vel_cb(self, msg: Twist):
        with self.lock:
            if not self.offboard_owned or self.active_goal is not None:
                self.get_logger().warn(
                    "Ignoring cmd_vel_body: needs OFFBOARD engaged and no active goal.",
                    throttle_duration_sec=2.0,
                )
                return
            heading = self.local_pos.heading
            forward, left = self.clamp_horizontal(msg.linear.x, msg.linear.y)
            self.sp_mode = "velocity"
            self.sp_velocity = [
                forward * math.cos(heading) + left * math.sin(heading),
                forward * math.sin(heading) - left * math.cos(heading),
                -msg.linear.z,
            ]
            self.sp_yawspeed = -msg.angular.z  # FLU counter-clockwise -> NED clockwise
            self.cmd_vel_stamp = time.monotonic()

    def clamp_horizontal(self, a, b):
        norm = math.hypot(a, b)
        if norm <= self.max_speed:
            return a, b
        return a * self.max_speed / norm, b * self.max_speed / norm

    # ---------------- setpoint streaming ----------------

    def stream_setpoint(self):
        with self.lock:
            if self.sp_mode is None or self.status is None:
                return
            if self.offboard_owned and self.status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self.get_logger().warn("Vehicle left OFFBOARD (pilot or failsafe). Releasing control.")
                self.offboard_owned = False
                self.sp_mode = None
                return
            if self.sp_mode == "velocity" and time.monotonic() - self.cmd_vel_stamp > CMD_VEL_TIMEOUT:
                self.get_logger().info("cmd_vel_body stopped. Holding position.")
                self.hold_here_locked()

            timestamp = self.get_clock().now().nanoseconds // 1000
            mode = OffboardControlMode()
            mode.timestamp = timestamp
            sp = TrajectorySetpoint()
            sp.timestamp = timestamp
            sp.acceleration = NAN3
            sp.jerk = NAN3
            if self.sp_mode == "position":
                mode.position = True
                sp.position = [float(v) for v in self.sp_position]
                sp.velocity = NAN3
                sp.yaw = float(self.sp_yaw)
                sp.yawspeed = float("nan")
            else:
                mode.velocity = True
                sp.position = NAN3
                sp.velocity = [float(v) for v in self.sp_velocity]
                sp.yaw = float("nan")
                sp.yawspeed = float(self.sp_yawspeed)
            self.offboard_mode_pub.publish(mode)
            self.setpoint_pub.publish(sp)

    def hold_here_locked(self):
        """Hold the current position and heading. Caller holds self.lock."""
        pos = self.local_pos
        self.sp_mode = "position"
        self.sp_position = [pos.x, pos.y, pos.z]
        self.sp_yaw = pos.heading

    def hold_here(self):
        with self.lock:
            self.hold_here_locked()

    # ---------------- PX4 helpers (called from action threads) ----------------

    def armed(self):
        return self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED

    def publish_command(self, command, **params):
        msg = VehicleCommand()
        msg.command = command
        for name, value in params.items():
            setattr(msg, name, float(value))
        msg.target_system = self.status.system_id
        msg.target_component = self.status.component_id
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.command_pub.publish(msg)

    def command(self, command, **params):
        """Send a VehicleCommand until PX4 accepts it. Returns an error string, or None on success."""
        self.pending_cmd = command
        deadline = time.monotonic() + COMMAND_TIMEOUT
        while time.monotonic() < deadline:
            self.ack_event.clear()
            self.publish_command(command, **params)
            if not self.ack_event.wait(ACK_TIMEOUT):
                continue
            if self.ack_result == VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED:
                return None
            if self.ack_result in ACK_REJECTED:
                return f"PX4 {ACK_REJECTED[self.ack_result]} command {command}"
            # IN_PROGRESS: keep waiting
        return f"no acknowledgement for command {command}"

    def set_mode(self, main_mode, sub_mode=0.0):
        return self.command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, param2=main_mode, param3=sub_mode,
        )

    def wait_until(self, condition, timeout, goal_handle, on_tick=None):
        """Poll until condition() is true. Returns None, or 'timeout' / 'canceled' / 'offboard lost'."""
        deadline = time.monotonic() + timeout
        must_own_offboard = self.offboard_owned
        while not condition():
            if goal_handle.is_cancel_requested:
                return "canceled"
            if must_own_offboard and not self.offboard_owned:
                return "offboard lost"
            if time.monotonic() > deadline:
                return "timeout"
            if on_tick:
                on_tick()
            time.sleep(1.0 / SETPOINT_RATE)
        return None

    # ---------------- action plumbing ----------------

    def on_goal(self, name):
        with self.lock:
            if self.status is None or self.local_pos is None:
                self.get_logger().warn(f"Rejecting {name}: no PX4 telemetry yet.")
                return GoalResponse.REJECT
            if self.active_goal is not None:
                self.get_logger().warn(f"Rejecting {name}: {self.active_goal} is running.")
                return GoalResponse.REJECT
            self.active_goal = name
        return GoalResponse.ACCEPT

    def accept_cancel(self, goal_handle):
        return CancelResponse.ACCEPT

    def reject_cancel(self, goal_handle):
        return CancelResponse.REJECT

    def finish(self, goal_handle, result, error=None):
        """Complete the goal (error None = success) and free the controller for the next goal."""
        result.success = error is None
        result.message = error or "ok"
        if error is None:
            goal_handle.succeed()
        elif goal_handle.is_cancel_requested:
            goal_handle.canceled()
        else:
            goal_handle.abort()
            self.get_logger().error(f"{self.active_goal} failed: {error}")
        with self.lock:
            self.active_goal = None
        return result

    # ---------------- Takeoff ----------------

    def execute_takeoff(self, goal_handle):
        result = Takeoff.Result()
        altitude = goal_handle.request.altitude
        self.get_logger().info(f"Takeoff: climbing {altitude:.1f} m")

        if altitude <= ALTITUDE_TOLERANCE:
            return self.finish(goal_handle, result, f"altitude must be > {ALTITUDE_TOLERANCE} m")
        if self.armed() and not self.landed:
            return self.finish(goal_handle, result, "already airborne")
        if math.isnan(self.amsl):
            return self.finish(goal_handle, result, "no valid altitude from PX4")

        # Hold mode first: PX4 accepts arming and the takeoff command from there.
        error = self.set_mode(PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_LOITER) or self.wait_until(
            lambda: self.status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER,
            MODE_TIMEOUT, goal_handle,
        )
        if error is None:
            error = self.command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        if error is None:
            error = self.wait_until(self.armed, MODE_TIMEOUT, goal_handle)
        if error is not None:
            return self.finish(goal_handle, result, f"arming: {error}")

        # Send takeoff right away: PX4 auto-disarms after COM_DISARM_PRFLT (10 s) on the ground.
        target_amsl = self.amsl + altitude
        error = self.command(
            VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF,
            param4=float("nan"),  # yaw: ignored by PX4 for multicopter takeoff
            param5=float("nan"),  # lat/lon NaN = current position
            param6=float("nan"),
            param7=target_amsl,  # altitude AMSL
        )
        if error is not None:
            return self.finish(goal_handle, result, f"takeoff command: {error}")

        feedback = Takeoff.Feedback()

        def climbing():
            if not self.armed():
                raise RuntimeError("vehicle disarmed during takeoff")
            feedback.altitude_remaining = float(max(0.0, target_amsl - self.amsl))
            goal_handle.publish_feedback(feedback)

        try:
            error = self.wait_until(
                lambda: self.amsl >= target_amsl - ALTITUDE_TOLERANCE, TAKEOFF_TIMEOUT, goal_handle, climbing
            )
        except RuntimeError as e:
            error = str(e)
        if error is not None:
            if self.armed():
                self.set_mode(PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_LOITER)
            return self.finish(goal_handle, result, f"climb: {error}")

        error = self.engage_offboard(goal_handle)
        if error is not None:
            return self.finish(goal_handle, result, f"engaging OFFBOARD: {error}")
        self.get_logger().info(f"Takeoff complete at {self.amsl:.1f} m AMSL. OFFBOARD engaged, holding.")
        return self.finish(goal_handle, result)

    def engage_offboard(self, goal_handle):
        self.hold_here()  # starts streaming the current position
        time.sleep(OFFBOARD_WARMUP)
        error = self.set_mode(PX4_CUSTOM_MAIN_MODE_OFFBOARD) or self.wait_until(
            lambda: self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD,
            MODE_TIMEOUT, goal_handle,
        )
        with self.lock:
            if error is None:
                self.offboard_owned = True
            else:
                self.sp_mode = None
        if error is not None:
            self.set_mode(PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_LOITER)
        return error

    # ---------------- GoTo ----------------

    def execute_goto(self, goal_handle):
        req = goal_handle.request
        result = GoTo.Result()
        if not self.offboard_owned:
            return self.finish(goal_handle, result, "OFFBOARD not engaged (run Takeoff first)")

        pos = self.local_pos
        start = (pos.x, pos.y, pos.z)
        heading = pos.heading
        if req.frame == GoTo.Goal.FRAME_LOCAL_NED:
            # NaN components keep the current value, e.g. pure altitude changes.
            target = tuple(t if math.isfinite(t) else c for t, c in zip((req.x, req.y, req.z), start))
            yaw = req.yaw if math.isfinite(req.yaw) else heading
        elif req.frame == GoTo.Goal.FRAME_BODY_FLU:
            target = (
                pos.x + req.x * math.cos(heading) + req.y * math.sin(heading),
                pos.y + req.x * math.sin(heading) - req.y * math.cos(heading),
                pos.z - req.z,
            )
            yaw = wrap_pi(heading - req.yaw) if math.isfinite(req.yaw) else heading
        else:
            return self.finish(goal_handle, result, f"unknown frame {req.frame}")

        speed = min(req.speed if req.speed > 0.0 else self.default_speed, self.max_speed)
        radius = req.acceptance_radius if req.acceptance_radius > 0.0 else self.default_radius
        delta = [t - s for t, s in zip(target, start)]
        total = math.sqrt(sum(d * d for d in delta))
        self.get_logger().info(
            f"GoTo N={target[0]:.1f} E={target[1]:.1f} D={target[2]:.1f} "
            f"yaw={math.degrees(yaw):.0f} deg ({total:.1f} m at {speed:.1f} m/s)"
        )

        with self.lock:
            self.sp_mode = "position"
            self.sp_yaw = yaw
        started = time.monotonic()
        feedback = GoTo.Feedback()

        def advance():
            # Move the setpoint along the line at `speed` instead of jumping to the target,
            # which PX4 would fly at full speed.
            progress = min(total, speed * (time.monotonic() - started))
            fraction = progress / total if total > 0.0 else 1.0
            with self.lock:
                self.sp_position = [s + d * fraction for s, d in zip(start, delta)]
            feedback.distance_remaining = float(self.distance_to(target))
            goal_handle.publish_feedback(feedback)

        def arrived():
            p = self.local_pos
            return (
                time.monotonic() - started >= total / speed
                and self.distance_to(target) < radius
                and math.sqrt(p.vx ** 2 + p.vy ** 2 + p.vz ** 2) < SETTLED_SPEED
            )

        error = self.wait_until(arrived, total / speed + GOTO_EXTRA_TIME, goal_handle, advance)
        if error is not None:
            if self.offboard_owned:
                self.hold_here()
            return self.finish(goal_handle, result, error)

        p = self.local_pos
        result.final_position = Point(x=float(p.x), y=float(p.y), z=float(p.z))
        self.get_logger().info("GoTo reached.")
        return self.finish(goal_handle, result)

    def distance_to(self, target):
        p = self.local_pos
        return math.sqrt((target[0] - p.x) ** 2 + (target[1] - p.y) ** 2 + (target[2] - p.z) ** 2)

    # ---------------- Land ----------------

    def execute_land(self, goal_handle):
        result = Land.Result()
        if not self.armed():
            return self.finish(goal_handle, result)  # already on the ground and disarmed

        self.get_logger().info("Landing...")
        error = self.command(
            VehicleCommand.VEHICLE_CMD_NAV_LAND,
            param4=float("nan"),  # yaw: keep current
            param5=float("nan"),  # lat/lon NaN = current position
            param6=float("nan"),
        )
        if error is not None:
            return self.finish(goal_handle, result, f"land command: {error}")
        with self.lock:  # PX4 is in AUTO land now; stop OFFBOARD streaming
            self.offboard_owned = False
            self.sp_mode = None

        feedback = Land.Feedback()

        def descending():
            feedback.altitude = float(-self.local_pos.z)
            goal_handle.publish_feedback(feedback)

        # PX4 disarms on its own shortly after touchdown (COM_DISARM_LAND).
        error = self.wait_until(lambda: not self.armed(), LAND_TIMEOUT, goal_handle, descending)
        if error is not None:
            return self.finish(goal_handle, result, error)
        self.get_logger().info("Landed and disarmed.")
        return self.finish(goal_handle, result)


def main(args=None):
    rclpy.init(args=args)
    node = FlightController()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
