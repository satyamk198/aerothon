"""Lower and release the payload.
Mechanism and hardware interface not decided yet.

Action: /deliver_payload (mission_interfaces/DeliverPayload)
"""
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from mission_interfaces.action import DeliverPayload


class Payload(Node):
    def __init__(self):
        super().__init__("payload")
        ActionServer(self, DeliverPayload, "/deliver_payload", self.execute)
        self.get_logger().info("payload ready (not implemented yet).")

    def execute(self, goal_handle):
        # TODO: implement the task; call goal_handle.succeed() and return a successful result
        self.get_logger().error("DeliverPayload is not implemented yet.")
        goal_handle.abort()
        return DeliverPayload.Result(success=False, message="not implemented")


def main(args=None):
    rclpy.init(args=args)
    node = Payload()
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
