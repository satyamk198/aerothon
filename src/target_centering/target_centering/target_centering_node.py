"""Hover directly over a QR code using its position in the downward camera image.

Ported from the centring loop of launch/lawnmower_mission.py in
https://github.com/Shiva007-RAndom/Qr-and-red-zone-detection, with the same logic and defaults:
  - offset of the QR centre from the image centre, converted to metres with
    metres_per_px = 2 * height * tan(hfov / 2) / image_width (height = altitude above the
    local origin, at least 0.5 m); image up = drone forward, image right = drone right
  - move by center_gain (0.4) of the offset per step, at most max_step (0.5 m); only north/east
    change, the altitude stays at the value it had when centring started
  - after each move: wait until within 0.15 m of the new point, dwell `settle` (1.5 s), then skip
    the 2 detection results still in flight before trusting a new one
  - centred = 3 readings in a row within center_tol (20 px); give up after `timeout` (60 s)
Moves go through flight_controller's GoTo (body frame), instead of raw MAVLink setpoints.

Action: /center_on_target (mission_interfaces/CenterOnTarget)
    TARGET_SINGLE_QR:  the only QR in view (start QR, before a reference exists)
    TARGET_MATCHED_QR: the best-scoring QR that matches the reference
Input:  /perception/qr (mission_interfaces/QrDetection), PX4 vehicle_local_position (height)
Output: /flight_controller/goto (mission_interfaces/GoTo action)
"""
import math
import threading
import time

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from px4_msgs.msg import VehicleLocalPosition
from mission_interfaces.action import CenterOnTarget, GoTo
from mission_interfaces.msg import QrDetection

from flight_controller.px4_utils import PX4_QOS, px4_topic

CENTRED_READINGS = 3  # consecutive readings within tolerance
SKIP_IN_FLIGHT = 2  # detection results still being processed when the drone stopped
GOTO_TIMEOUT = 30.0  # s for one correction move


class TargetCentering(Node):
    def __init__(self):
        super().__init__("target_centering")
        # Defaults from lawnmower_mission.py; camera_hfov is the sim camera's (x500_mono_cam_down).
        self.hfov = float(self.declare_parameter("camera_hfov", 1.74).value)  # rad, horizontal FOV
        self.center_tol = float(self.declare_parameter("center_tol", 20.0).value)  # px
        self.center_gain = float(self.declare_parameter("center_gain", 0.4).value)
        self.max_step = float(self.declare_parameter("max_step", 0.5).value)  # m
        self.settle = float(self.declare_parameter("settle", 1.5).value)  # s
        self.timeout = float(self.declare_parameter("timeout", 60.0).value)  # s
        self.arrive_tol = float(self.declare_parameter("arrive_tol", 0.15).value)  # m

        self.lock = threading.Lock()
        self.last = None  # latest QrDetection
        self.count = 0  # number of QrDetection results received
        self.height = None  # m above the local origin

        group = ReentrantCallbackGroup()
        # qr_detector publishes its results latched
        self.create_subscription(
            QrDetection, "/perception/qr", self.qr_cb,
            QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                       durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(
            VehicleLocalPosition, px4_topic("/fmu/out/vehicle_local_position", VehicleLocalPosition),
            self.local_pos_cb, PX4_QOS)
        self.goto_client = ActionClient(self, GoTo, "/flight_controller/goto", callback_group=group)
        ActionServer(self, CenterOnTarget, "/center_on_target", self.execute,
                     cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=group)
        self.get_logger().info(f"target_centering ready (hfov {self.hfov:.2f} rad).")

    def qr_cb(self, msg: QrDetection):
        with self.lock:
            self.last = msg
            self.count += 1

    def local_pos_cb(self, msg: VehicleLocalPosition):
        self.height = -msg.z if msg.z_valid else None

    # ---------------- helpers (action thread) ----------------

    def goto_body(self, forward, right, hold_height):
        """Blocking GoTo by (forward, right) metres at `hold_height`; True once the drone arrived."""
        up = hold_height - self.height  # undo any altitude drift since centring started
        goal = GoTo.Goal(frame=GoTo.Goal.FRAME_BODY_FLU, x=float(forward), y=float(-right), z=float(up),
                         yaw=math.nan, acceptance_radius=self.arrive_tol)
        if not self.goto_client.wait_for_server(timeout_sec=5.0):
            return False
        done = threading.Event()
        outcome = {}

        def on_result(future):
            outcome["result"] = future.result().result
            done.set()

        def on_goal(future):
            handle = future.result()
            if not handle.accepted:
                done.set()
                return
            handle.get_result_async().add_done_callback(on_result)

        self.goto_client.send_goal_async(goal).add_done_callback(on_goal)
        done.wait(GOTO_TIMEOUT)
        return bool(outcome.get("result") and outcome["result"].success)

    def wait_fresh(self, gate, deadline, goal_handle):
        """Wait until at least `gate` results were received. False on timeout or cancel."""
        while True:
            with self.lock:
                if self.count >= gate:
                    return True
            if goal_handle.is_cancel_requested or time.monotonic() > deadline:
                return False
            time.sleep(0.05)

    def settle_then_gate(self):
        """Dwell, then skip the results still in flight; returns the result count to wait for."""
        time.sleep(self.settle)
        with self.lock:
            return self.count + SKIP_IN_FLIGHT

    @staticmethod
    def pick(result, target):
        if result is None:
            return None
        if target == CenterOnTarget.Goal.TARGET_MATCHED_QR:
            matched = [d for d in result.detections if d.matched]  # detections are sorted by score
            return matched[0] if matched else None
        return result.detections[0] if len(result.detections) == 1 else None

    # ---------------- action ----------------

    def execute(self, goal_handle):
        target = goal_handle.request.target
        result = CenterOnTarget.Result()
        feedback = CenterOnTarget.Feedback()
        self.get_logger().info("Centring over the QR...")
        start = time.monotonic()
        deadline = start + self.timeout
        centred = 0
        gate = self.settle_then_gate()
        hold_height = self.height  # as lawnmower_mission.py: the hold altitude never changes
        if hold_height is None:
            result.message = "no altitude from PX4"
            goal_handle.abort()
            return result

        while centred < CENTRED_READINGS and time.monotonic() < deadline:
            if not self.wait_fresh(gate, deadline, goal_handle):
                break
            with self.lock:
                res, count = self.last, self.count
            det = self.pick(res, target)
            if det is None or self.height is None or res.image_width == 0:
                gate = count + 1  # QR lost this frame: try the next one
                continue

            x1, y1, x2, y2 = det.bbox_xyxy
            u, v = (x1 + x2) / 2.0, (y1 + y2) / 2.0
            cx, cy = res.image_width / 2.0, res.image_height / 2.0
            metres_per_px = 2.0 * max(self.height, 0.5) * math.tan(self.hfov / 2.0) / res.image_width
            fwd, right = -(v - cy) * metres_per_px, (u - cx) * metres_per_px
            err_px = math.hypot(u - cx, v - cy)
            feedback.offset_px = float(err_px)
            goal_handle.publish_feedback(feedback)

            if err_px <= self.center_tol:
                centred += 1
                gate = self.settle_then_gate()  # keep sampling while stationary
                continue

            centred = 0
            step_f, step_r = self.center_gain * fwd, self.center_gain * right
            norm = math.hypot(step_f, step_r)
            if norm > self.max_step:  # cap each correction
                step_f, step_r = step_f * self.max_step / norm, step_r * self.max_step / norm
            self.get_logger().info(
                f"  offset {err_px:.0f} px (u={u:.0f}, v={v:.0f}) -> move fwd {step_f:+.2f} m, right {step_r:+.2f} m")
            if not self.goto_body(step_f, step_r, hold_height):
                result.message = "flight_controller GoTo failed"
                goal_handle.abort()
                self.get_logger().error(f"Centring failed: {result.message}")
                return result
            gate = self.settle_then_gate()

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result.message = "canceled"
            return result
        if centred >= CENTRED_READINGS:
            self.get_logger().info("Centred over the QR.")
            result.success = True
            result.message = "centred"
            goal_handle.succeed()
        else:
            result.message = "could not fully centre (QR lost or still drifting); holding here"
            self.get_logger().warn(result.message)
            goal_handle.abort()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = TargetCentering()
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
