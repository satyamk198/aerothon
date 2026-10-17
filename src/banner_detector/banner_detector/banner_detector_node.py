"""Detect the AeroTHON 2026 banner (green background) at the corridor entrance
and report where it is relative to the drone.

Input:  TODO (camera / sensor topic)
Output: /perception/banner (mission_interfaces/BannerDetection)
"""
import rclpy
from rclpy.node import Node
from mission_interfaces.msg import BannerDetection


class BannerDetector(Node):
    def __init__(self):
        super().__init__("banner_detector")
        self.detection_pub = self.create_publisher(BannerDetection, "/perception/banner", 10)
        # TODO: subscribe to the input, run detection, publish BannerDetection on /perception/banner
        self.get_logger().info("banner_detector started (not implemented yet).")


def main(args=None):
    rclpy.init(args=args)
    node = BannerDetector()
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
