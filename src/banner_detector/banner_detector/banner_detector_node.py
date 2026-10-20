#!/usr/bin/env python3
"""ROS 2 Node for Aerothon Banner Detection.

Subscribes to front-facing camera topic, detects Aerothon banner using EasyOCR + HSV,
and publishes detection results on /perception/banner (mission_interfaces/BannerDetection)
and an annotated debug image on /banner/debug_image.

Based on: https://github.com/leeIITM/aerothon_banner_det
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from mission_interfaces.msg import BannerDetection

from banner_detector.banner_detector import BannerDetector


def image_msg_to_numpy(msg: Image) -> np.ndarray:
    """Robust conversion of ROS sensor_msgs/Image to OpenCV BGR numpy array.

    Avoids ABI incompatibilities with native cv_bridge.
    """
    if msg.encoding in ("rgb8", "bgr8"):
        channels = 3
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, channels))
        if msg.encoding == "rgb8":
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        return img
    elif msg.encoding == "mono8":
        return np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width))
    elif msg.encoding == "rgba8":
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 4))
        return cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
    elif msg.encoding == "bgra8":
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 4))
        return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
    else:
        # Fallback
        return np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, -1))


def numpy_to_image_msg(img: np.ndarray, encoding: str = "bgr8") -> Image:
    """Converts OpenCV numpy array to ROS sensor_msgs/Image."""
    msg = Image()
    msg.height, msg.width = img.shape[:2]
    msg.encoding = encoding
    msg.is_bigendian = 0
    channels = img.shape[2] if len(img.shape) > 2 else 1
    msg.step = msg.width * channels
    msg.data = img.tobytes()
    return msg


class BannerDetectorNode(Node):
    def __init__(self):
        super().__init__("banner_detector")

        # Declare parameters
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("process_every_n", 2)
        self.declare_parameter("use_gpu", False)

        image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        self.process_every_n = self.get_parameter("process_every_n").get_parameter_value().integer_value
        use_gpu = self.get_parameter("use_gpu").get_parameter_value().bool_value

        self.get_logger().info(f"Initializing BannerDetectorNode on topic: {image_topic}")

        self.detector = BannerDetector(use_gpu=use_gpu)
        self.frame_count = 0

        # Publishers
        self.detection_pub = self.create_publisher(BannerDetection, "/perception/banner", 10)
        self.pub_debug_img = self.create_publisher(Image, "/banner/debug_image", 10)

        # Subscriber
        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.sub_image = self.create_subscription(Image, image_topic, self.image_callback, qos)

        self.get_logger().info("BannerDetectorNode successfully started.")

    def image_callback(self, msg: Image):
        self.frame_count += 1
        if self.frame_count % self.process_every_n != 0:
            return

        try:
            cv_img = image_msg_to_numpy(msg)
        except Exception as e:
            self.get_logger().warn(f"Failed to convert ROS Image to numpy: {e}")
            return

        h, w = cv_img.shape[:2]
        res = self.detector.process_frame(cv_img)

        # Publish BannerDetection
        det_msg = BannerDetection()
        det_msg.header = msg.header
        det_msg.detected = bool(res["detected"])
        det_msg.image_width = w
        det_msg.image_height = h

        if res["detected"] and res["center"] is not None and res["bbox"] is not None:
            bx1, by1, bx2, by2 = res["bbox"]
            cx, cy = res["center"]
            det_msg.bbox_xyxy = [float(bx1), float(by1), float(bx2), float(by2)]
            det_msg.center_x = float(cx)
            det_msg.center_y = float(cy)
            det_msg.confidence = float(res.get("green_ratio", 0.0))
            det_msg.text = str(res.get("text", ""))
            self.get_logger().info(
                f"BANNER DETECTED -> text='{det_msg.text}' center=({cx}, {cy}) "
                f"green={res['green_ratio']*100:.1f}%",
                throttle_duration_sec=1.0,
            )
        else:
            det_msg.bbox_xyxy = [0.0, 0.0, 0.0, 0.0]
            det_msg.center_x = 0.0
            det_msg.center_y = 0.0
            det_msg.confidence = 0.0
            det_msg.text = ""

        self.detection_pub.publish(det_msg)

        # Publish annotated debug image
        try:
            debug_msg = numpy_to_image_msg(res["annotated_frame"], encoding="bgr8")
            debug_msg.header = msg.header
            self.pub_debug_img.publish(debug_msg)
        except Exception as e:
            self.get_logger().warn(f"Failed to publish debug image: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = BannerDetectorNode()
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
