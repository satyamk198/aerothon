"""Detect red / restricted zones in the camera image using HSV thresholds.

Ported from the final cell of "red_zone_detection (1).ipynb" in
https://github.com/Shiva007-RAndom/Qr-and-red-zone-detection: same HSV ranges,
morphological cleanup, contour area filter, centroids and annotation. The notebook
reads one image file; this node runs the same detection on every camera frame.

Subscribes
    <image_topic>              sensor_msgs/Image

Publishes
    /perception/restricted_zones  mission_interfaces/RestrictedZones  patches in pixels
    ~/debug_image                 sensor_msgs/Image  annotated frame (only while subscribed)
"""
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from mission_interfaces.msg import RedPatch, RestrictedZones

# HSV ranges for red: red wraps around 0/180 in OpenCV's hue, so two ranges are needed.
RED_LOWER1 = np.array([0, 80, 40])  # range 1: pure red side
RED_UPPER1 = np.array([15, 255, 255])
RED_LOWER2 = np.array([165, 80, 40])  # range 2: wraparound red side
RED_UPPER2 = np.array([180, 255, 255])

MORPH_KERNEL = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))


def detect_red_patches(bgr, min_area):
    """Detect red patches in a BGR image using filled contours.

    Returns (annotated, mask, patches); patches is a list of dicts with keys
    area and centroid (cx, cy), in pixels.
    """
    start = time.perf_counter()

    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

    # Masks for both red hue ranges, combined
    mask1 = cv2.inRange(hsv, RED_LOWER1, RED_UPPER1)
    mask2 = cv2.inRange(hsv, RED_LOWER2, RED_UPPER2)
    mask = cv2.bitwise_or(mask1, mask2)

    # Morphological cleanup
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, MORPH_KERNEL, iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, MORPH_KERNEL, iterations=2)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    patches = []
    annotated = bgr.copy()

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < min_area:
            continue

        # Centroid via image moments
        m = cv2.moments(cnt)
        cx = int(m["m10"] / m["m00"]) if m["m00"] != 0 else 0
        cy = int(m["m01"] / m["m00"]) if m["m00"] != 0 else 0

        patches.append({"area": area, "centroid": (cx, cy)})

        # Filled contour with semi-transparency, outline, centroid dot, label
        overlay = annotated.copy()
        cv2.drawContours(overlay, [cnt], -1, (0, 255, 0), thickness=cv2.FILLED)
        cv2.addWeighted(overlay, 0.35, annotated, 0.65, 0, annotated)
        cv2.drawContours(annotated, [cnt], -1, (0, 255, 0), 2)
        cv2.circle(annotated, (cx, cy), 5, (255, 0, 0), -1)
        cv2.putText(annotated, f"Red ({int(area)}px²)", (cx + 8, cy),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)

    elapsed = time.perf_counter() - start

    cv2.putText(annotated, f"Red patches: {len(patches)}  |  {elapsed * 1000:.1f} ms",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)

    return annotated, mask, patches, elapsed


class RedZoneDetector(Node):
    def __init__(self):
        super().__init__("red_zone_detector")

        image_topic = self.declare_parameter("image_topic", "/camera/image_raw").value
        self.min_area = float(self.declare_parameter("min_area", 100.0).value)  # px^2

        self.bridge = CvBridge()
        # Cameras usually publish best-effort.
        image_qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                               history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(Image, image_topic, self.on_image, image_qos)
        self.zones_pub = self.create_publisher(RestrictedZones, "/perception/restricted_zones", 10)
        self.debug_pub = self.create_publisher(Image, "~/debug_image", 1)

        self.get_logger().info(
            f"Ready. Listening on '{image_topic}', min_area {self.min_area:.0f} px^2.")

    def on_image(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as exc:
            self.get_logger().error(f"cv_bridge failed: {exc}")
            return
        if frame is None or frame.size == 0:
            return

        annotated, _, patches, elapsed = detect_red_patches(frame, self.min_area)

        result = RestrictedZones()
        result.header = msg.header
        result.processing_time_ms = float(elapsed * 1000.0)
        for p in patches:
            patch = RedPatch()
            patch.area_px = float(p["area"])
            patch.centroid_x, patch.centroid_y = p["centroid"]
            result.patches.append(patch)
        self.zones_pub.publish(result)

        if self.debug_pub.get_subscription_count() > 0:
            debug = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            debug.header = msg.header
            self.debug_pub.publish(debug)


def main(args=None):
    rclpy.init(args=args)
    node = RedZoneDetector()
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
