"""Detect obstacles and corridor walls around the drone inside the 3.5 m corridor.
Sensor not decided yet (mono camera, depth camera, lidar or ToF).

Input:  TODO (camera / sensor topic)
Output: /perception/obstacles (mission_interfaces/Obstacles)
"""
import rclpy
from rclpy.node import Node
from mission_interfaces.msg import Obstacles


class ObstacleDetector(Node):
    def __init__(self):
        super().__init__("obstacle_detector")
        self.detection_pub = self.create_publisher(Obstacles, "/perception/obstacles", 10)
        # TODO: subscribe to the input, run detection, publish Obstacles on /perception/obstacles
        self.get_logger().info("obstacle_detector started (not implemented yet).")


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
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
