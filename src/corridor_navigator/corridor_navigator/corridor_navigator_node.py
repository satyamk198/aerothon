"""Fly through the 3.5 m wide corridor, staying centred and avoiding obstacles.
Used on both the forward and the return lap.

Action: /navigate_corridor (mission_interfaces/NavigateCorridor)
Input:  /perception/obstacles (mission_interfaces/Obstacles)
Output: /flight_controller/cmd_vel_body (geometry_msgs/Twist, body FLU)
"""
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from geometry_msgs.msg import Twist
from mission_interfaces.action import NavigateCorridor
from mission_interfaces.msg import Obstacles


class CorridorNavigator(Node):
    def __init__(self):
        super().__init__("corridor_navigator")
        self.obstacles = None  # latest Obstacles
        self.create_subscription(Obstacles, "/perception/obstacles", self.obstacles_cb, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, "/flight_controller/cmd_vel_body", 10)
        ActionServer(self, NavigateCorridor, "/navigate_corridor", self.execute)
        self.get_logger().info("corridor_navigator ready (not implemented yet).")

    def obstacles_cb(self, msg: Obstacles):
        self.obstacles = msg

    def execute(self, goal_handle):
        # TODO: implement the task; call goal_handle.succeed() and return a successful result
        self.get_logger().error("NavigateCorridor is not implemented yet.")
        goal_handle.abort()
        return NavigateCorridor.Result(success=False, message="not implemented")


def main(args=None):
    rclpy.init(args=args)
    node = CorridorNavigator()
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
