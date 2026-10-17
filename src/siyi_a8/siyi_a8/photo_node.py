"""4K photo loop for the SIYI A8 mini: while enabled, take a photo, download it from the
camera's SD card, publish it, repeat. All detectors read the published images.

The loop starts disabled. The mission tree enables it only while a step that processes images
runs (start-QR capture, banner alignment, corridor, target search, centering + delivery), so
no photos are taken during e.g. takeoff or altitude changes.

Taking the photo uses the SIYI SDK photo command (siyi_protocol.take_picture, UDP).
Downloading it from the SD card uses an unofficial API that has not been chosen yet:
download_latest_photo() is a placeholder. Until it is implemented the loop stops after the
first photo after each enable, so it does not fill the SD card with photos nobody reads.

Publishes
    <image_topic>   sensor_msgs/Image (bgr8), default /camera/image_raw. header.stamp is the time
                    the photo command was sent (for matching the photo to the drone's pose).
Services
    ~/enable        std_srvs/SetBool  true: run the loop, false: stop after the current photo
"""
import socket
import threading

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool

from siyi_a8 import siyi_protocol as siyi


def download_latest_photo(camera_ip):
    """Return the photo just taken, as a BGR numpy array, or None if this attempt failed.

    TODO: implement with the SD-card API once it is chosen. It must wait until the new photo
    exists on the SD card and return that photo (not an older one).
    """
    raise NotImplementedError("SD-card download API not chosen yet")


class PhotoNode(Node):
    def __init__(self):
        super().__init__("siyi_photo")
        # Defaults are SIYI's factory network settings (as used by mzahana/siyi_sdk).
        self.camera_ip = self.declare_parameter("camera_ip", "192.168.144.25").value
        self.address = (self.camera_ip, int(self.declare_parameter("camera_port", 37260).value))
        self.frame_id = self.declare_parameter("frame_id", "camera").value
        topic = self.declare_parameter("image_topic", "/camera/image_raw").value

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.seq = 0
        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, topic, 1)
        self.running = True
        self.enabled = threading.Event()  # loop runs only while set
        self.create_service(SetBool, "~/enable", self.enable_cb)
        self.thread = threading.Thread(target=self.loop, daemon=True)
        self.thread.start()
        self.get_logger().info(
            f"Photo loop ready (off until ~/enable): camera {self.address[0]}:{self.address[1]} -> {topic}")

    def enable_cb(self, request, response):
        if request.data != self.enabled.is_set():
            self.get_logger().info("Photo loop " + ("ON" if request.data else "OFF"))
        if request.data:
            self.enabled.set()
        else:
            self.enabled.clear()
        response.success = True
        response.message = "on" if request.data else "off"
        return response

    def loop(self):
        while self.running and rclpy.ok():
            if not self.enabled.wait(timeout=0.5):
                continue
            stamp = self.get_clock().now().to_msg()
            try:
                self.sock.sendto(siyi.take_picture(self.seq), self.address)
            except OSError as exc:
                self.get_logger().warn(f"Cannot reach camera: {exc}", throttle_duration_sec=5.0)
            self.seq = (self.seq + 1) & 0xFFFF

            try:
                image = download_latest_photo(self.camera_ip)
            except NotImplementedError as exc:
                self.get_logger().error(f"{exc}: photo loop OFF after one photo.")
                self.enabled.clear()
                continue
            if image is None:
                self.get_logger().warn("Photo download failed; retrying.", throttle_duration_sec=5.0)
                continue

            msg = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
            msg.header.stamp = stamp
            msg.header.frame_id = self.frame_id
            self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PhotoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.sock.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
