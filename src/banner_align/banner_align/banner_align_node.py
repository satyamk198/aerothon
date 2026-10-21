"""Line the drone up with the corridor entrance using the banner detection.
Used on both the forward and the return lap.

Subscribes to /perception/banner (BannerDetection) for live pixel offsets,
runs a PID-style visual servoing loop, and publishes body-frame velocity
commands on /flight_controller/cmd_vel_body (geometry_msgs/Twist, body FLU).

Action: /align_with_corridor (mission_interfaces/AlignWithCorridor)
Input:  /perception/banner (mission_interfaces/BannerDetection)
Output: /flight_controller/cmd_vel_body (geometry_msgs/Twist, body FLU)

Based on the visual servoing alignment logic from:
https://github.com/leeIITM/aerothon_banner_det
"""
import math
import time
import threading

import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import Twist
from mission_interfaces.action import AlignWithCorridor
from mission_interfaces.msg import BannerDetection


class BannerAlign(Node):
    def __init__(self):
        super().__init__("banner_align")

        # PID gains for visual servoing (tunable via ROS parameters)
        self.kp = float(self.declare_parameter("kp", 0.5).value)
        self.ki = float(self.declare_parameter("ki", 0.0).value)
        self.kd = float(self.declare_parameter("kd", 0.1).value)
        self.max_speed = float(self.declare_parameter("max_speed", 0.5).value)  # m/s

        # Alignment convergence criteria
        self.centered_threshold = float(self.declare_parameter("centered_threshold", 0.05).value)
        self.centered_count = int(self.declare_parameter("centered_count", 15).value)

        # Timeouts
        self.timeout = float(self.declare_parameter("timeout", 60.0).value)  # s
        self.lost_timeout = float(self.declare_parameter("lost_timeout", 5.0).value)  # s

        # Control loop rate
        self.loop_rate = float(self.declare_parameter("loop_rate", 10.0).value)  # Hz

        # PID state
        self.prev_err_x = 0.0
        self.prev_err_y = 0.0
        self.integral_x = 0.0
        self.integral_y = 0.0
        self.last_pid_time = 0.0

        # Latest banner detection
        self.lock = threading.Lock()
        self.banner_detection = None
        self.last_detection_time = 0.0

        group = ReentrantCallbackGroup()
        self.create_subscription(
            BannerDetection, "/perception/banner", self.banner_detection_cb, 10
        )
        self.cmd_vel_pub = self.create_publisher(
            Twist, "/flight_controller/cmd_vel_body", 10
        )
        ActionServer(
            self, AlignWithCorridor, "/align_with_corridor", self.execute,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=group,
        )
        self.get_logger().info(
            f"banner_align ready (kp={self.kp}, ki={self.ki}, kd={self.kd}, "
            f"max_speed={self.max_speed}, centered_threshold={self.centered_threshold}, "
            f"centered_count={self.centered_count})."
        )

    def banner_detection_cb(self, msg: BannerDetection):
        with self.lock:
            self.banner_detection = msg
            if msg.detected:
                self.last_detection_time = time.monotonic()

    def reset_pid(self):
        """Reset PID integrator and derivative state."""
        self.prev_err_x = 0.0
        self.prev_err_y = 0.0
        self.integral_x = 0.0
        self.integral_y = 0.0
        self.last_pid_time = time.monotonic()

    def compute_pid(self, err_x: float, err_y: float):
        """Compute PID velocity corrections from normalized pixel errors.

        err_x: normalized [-1, 1], positive = banner right of image center
        err_y: normalized [-1, 1], positive = banner below image center

        Returns (vy, vz) in body FLU:
            vy: negative = move left (banner is right, move right by negating FLU left convention)
            vz: positive = move up (banner is below center, move up is positive in FLU)
        """
        now = time.monotonic()
        dt = now - self.last_pid_time
        if dt <= 0.0 or dt > 1.0:
            dt = 1.0 / self.loop_rate
        self.last_pid_time = now

        # X-axis (lateral) PID
        self.integral_x += err_x * dt
        derivative_x = (err_x - self.prev_err_x) / dt
        self.prev_err_x = err_x

        # Y-axis (vertical) PID
        self.integral_y += err_y * dt
        derivative_y = (err_y - self.prev_err_y) / dt
        self.prev_err_y = err_y

        # PID outputs
        output_x = self.kp * err_x + self.ki * self.integral_x + self.kd * derivative_x
        output_y = self.kp * err_y + self.ki * self.integral_y + self.kd * derivative_y

        # Body FLU convention:
        #   linear.y = positive is LEFT; banner right of center (err_x > 0) -> move right -> negative linear.y
        #   linear.z = positive is UP; banner below center (err_y > 0) -> move up -> positive linear.z
        vy = -self.clamp(output_x, self.max_speed)
        vz = -self.clamp(output_y, self.max_speed)

        return vy, vz

    @staticmethod
    def clamp(value, limit):
        return max(-limit, min(limit, value))

    def stop(self):
        """Publish zero velocity to stop the drone."""
        self.cmd_vel_pub.publish(Twist())

    def execute(self, goal_handle):
        """Execute the AlignWithCorridor action."""
        self.get_logger().info("AlignWithCorridor: starting alignment...")
        self.reset_pid()

        result = AlignWithCorridor.Result()
        feedback = AlignWithCorridor.Feedback()

        start_time = time.monotonic()
        stable_count = 0
        dt = 1.0 / self.loop_rate

        while rclpy.ok():
            elapsed = time.monotonic() - start_time

            # Check cancel
            if goal_handle.is_cancel_requested:
                self.stop()
                goal_handle.canceled()
                result.success = False
                result.message = "canceled"
                self.get_logger().info("AlignWithCorridor: canceled.")
                return result

            # Check overall timeout
            if elapsed > self.timeout:
                self.stop()
                result.success = False
                result.message = f"timeout after {self.timeout:.0f}s"
                goal_handle.abort()
                self.get_logger().error(f"AlignWithCorridor: {result.message}")
                return result

            # Get latest detection
            with self.lock:
                det = self.banner_detection
                last_det_time = self.last_detection_time

            # Check if banner is detected
            if det is None or not det.detected:
                # Check lost timeout
                time_since_last = time.monotonic() - last_det_time if last_det_time > 0 else elapsed
                if time_since_last > self.lost_timeout:
                    self.stop()
                    result.success = False
                    result.message = f"banner lost for {self.lost_timeout:.1f}s"
                    goal_handle.abort()
                    self.get_logger().error(f"AlignWithCorridor: {result.message}")
                    return result
                # Banner temporarily lost; hold position (send zero velocity)
                self.stop()
                stable_count = 0
                time.sleep(dt)
                continue

            # Compute normalized errors
            if det.image_width == 0 or det.image_height == 0:
                time.sleep(dt)
                continue

            err_x = float(det.center_x - (det.image_width / 2.0)) / float(det.image_width / 2.0)
            err_y = float(det.center_y - (det.image_height / 2.0)) / float(det.image_height / 2.0)

            # Publish feedback
            feedback.error_x = float(err_x)
            feedback.error_y = float(err_y)
            feedback.centered = abs(err_x) < self.centered_threshold and abs(err_y) < self.centered_threshold
            goal_handle.publish_feedback(feedback)

            # Check if centered
            if feedback.centered:
                stable_count += 1
            else:
                stable_count = max(0, stable_count - 1)

            if stable_count >= self.centered_count:
                self.stop()
                result.success = True
                result.message = f"aligned (err_x={err_x:+.4f}, err_y={err_y:+.4f})"
                goal_handle.succeed()
                self.get_logger().info(f"AlignWithCorridor: SUCCESS - {result.message}")
                return result

            # Compute PID velocity corrections and publish
            vy, vz = self.compute_pid(err_x, err_y)

            cmd = Twist()
            # linear.x = 0 (no forward/backward during alignment)
            cmd.linear.y = vy  # left/right in body FLU
            cmd.linear.z = vz  # up/down in body FLU
            self.cmd_vel_pub.publish(cmd)

            self.get_logger().info(
                f"  err_x={err_x:+.3f} err_y={err_y:+.3f} -> vy={vy:+.3f} vz={vz:+.3f} "
                f"stable={stable_count}/{self.centered_count}",
                throttle_duration_sec=0.5,
            )

            time.sleep(dt)

        # rclpy shutdown
        self.stop()
        result.success = False
        result.message = "node shutting down"
        goal_handle.abort()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = BannerAlign()
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
