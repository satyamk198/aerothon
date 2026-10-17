"""Line the drone up with the corridor entrance using the banner detection.
Used on both the forward and the return lap.

Action: /align_with_corridor (mission_interfaces/AlignWithCorridor)
Input:  /perception/banner (mission_interfaces/BannerDetection)
Output: /flight_controller/cmd_vel_body (geometry_msgs/Twist, body FLU)
"""
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from geometry_msgs.msg import Twist
from mission_interfaces.action import AlignWithCorridor
from mission_interfaces.msg import BannerDetection


class BannerAlign(Node):
    def __init__(self):
        super().__init__("banner_align")
        self.banner_detection = None  # latest BannerDetection
        self.create_subscription(BannerDetection, "/perception/banner", self.banner_detection_cb, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, "/flight_controller/cmd_vel_body", 10)
        ActionServer(self, AlignWithCorridor, "/align_with_corridor", self.execute)
        self.get_logger().info("banner_align ready (not implemented yet).")

    def banner_detection_cb(self, msg: BannerDetection):
        self.banner_detection = msg

    def execute(self, goal_handle):
        # TODO: implement the task; call goal_handle.succeed() and return a successful result
        self.get_logger().error("AlignWithCorridor is not implemented yet.")
        goal_handle.abort()
        return AlignWithCorridor.Result(success=False, message="not implemented")


def main(args=None):
    rclpy.init(args=args)
    node = BannerAlign()
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
