"""Search the delivery zone from 10 m for the QR code that matches the delivery info
read from the start QR, routing around restricted zones.

Action: /search_target (mission_interfaces/SearchTarget)
Input:  /perception/qr (mission_interfaces/QrDetection)
Input:  /perception/restricted_zones (mission_interfaces/RestrictedZones)
Output: /flight_controller/goto (mission_interfaces/GoTo action)
"""
import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.node import Node
from mission_interfaces.action import GoTo, SearchTarget
from mission_interfaces.msg import QrDetection, RestrictedZones


class TargetSearch(Node):
    def __init__(self):
        super().__init__("target_search")
        self.qr_detection = None  # latest QrDetection
        self.restricted_zones = None  # latest RestrictedZones
        self.create_subscription(QrDetection, "/perception/qr", self.qr_detection_cb, 10)
        self.create_subscription(RestrictedZones, "/perception/restricted_zones", self.restricted_zones_cb, 10)
        self.goto_client = ActionClient(self, GoTo, "/flight_controller/goto")
        ActionServer(self, SearchTarget, "/search_target", self.execute)
        self.get_logger().info("target_search ready (not implemented yet).")

    def qr_detection_cb(self, msg: QrDetection):
        self.qr_detection = msg

    def restricted_zones_cb(self, msg: RestrictedZones):
        self.restricted_zones = msg

    def execute(self, goal_handle):
        # TODO: implement the task; call goal_handle.succeed() and return a successful result
        self.get_logger().error("SearchTarget is not implemented yet.")
        goal_handle.abort()
        return SearchTarget.Result(success=False, message="not implemented")


def main(args=None):
    rclpy.init(args=args)
    node = TargetSearch()
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
