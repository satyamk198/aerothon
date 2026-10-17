"""Publish the SIYI A8 mini RTSP video stream as sensor_msgs/Image (debugging only: not started by
drone.launch.py, which uses photo_node; the ground station views the stream directly).

Reads <rtsp_url> with OpenCV's FFmpeg backend (as mzahana/siyi_sdk's stream.py does) in a
background thread that always keeps only the newest frame, so a slow consumer never builds
up latency. Reconnects if the stream drops.

Publishes
    <image_topic>   sensor_msgs/Image (bgr8), default /camera/image_raw (what the detectors read)
"""
import threading
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

RECONNECT_DELAY = 2.0  # s


class RtspCamera(Node):
    def __init__(self):
        super().__init__("siyi_camera")
        # Default: SIYI factory stream address (mzahana/siyi_sdk stream.py).
        self.url = self.declare_parameter("rtsp_url", "rtsp://192.168.144.25:8554/main.264").value
        self.frame_id = self.declare_parameter("frame_id", "camera").value
        topic = self.declare_parameter("image_topic", "/camera/image_raw").value

        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, topic, qos_profile_sensor_data)
        self.lock = threading.Lock()
        self.frame = None
        self.frame_stamp = None
        self.running = True
        threading.Thread(target=self.read_loop, daemon=True).start()
        self.create_timer(1.0 / 60.0, self.publish_latest)
        self.get_logger().info(f"Opening {self.url} -> {topic}")

    def read_loop(self):
        while self.running:
            cap = cv2.VideoCapture(self.url, cv2.CAP_FFMPEG)
            if not cap.isOpened():
                self.get_logger().warn(f"Cannot open {self.url}; retrying.", throttle_duration_sec=10.0)
                time.sleep(RECONNECT_DELAY)
                continue
            self.get_logger().info(
                f"Stream open: {int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}"
                f" @ {cap.get(cv2.CAP_PROP_FPS):.0f} fps")
            while self.running:
                ok, frame = cap.read()
                if not ok:
                    self.get_logger().warn("Stream dropped; reconnecting.")
                    break
                with self.lock:
                    self.frame = frame
                    self.frame_stamp = self.get_clock().now().to_msg()
            cap.release()
            time.sleep(RECONNECT_DELAY)

    def publish_latest(self):
        with self.lock:
            frame, stamp = self.frame, self.frame_stamp
            self.frame = None
        if frame is None:
            return
        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RtspCamera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
