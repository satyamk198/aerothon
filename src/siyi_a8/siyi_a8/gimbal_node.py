"""Point the SIYI A8 mini gimbal (on Ethernet) at a fixed attitude (default straight down) and set its zoom.

Talks the SIYI SDK UDP protocol (see siyi_protocol.py) to <gimbal_ip>:<gimbal_port>.
At start-up, and on ~/point_down, it sends the target attitude once per second until the
gimbal's own attitude reading is within `tolerance_deg` of it.

Publishes
    ~/attitude       geometry_msgs/Vector3Stamped  gimbal yaw (x), pitch (y), roll (z) in degrees,
                                                    as reported by the gimbal (pitch -90 = straight down)
Services
    ~/point_down     std_srvs/Trigger              send the target attitude again
    ~/set_zoom       mission_interfaces/SetZoom    digital zoom multiple, 1.0 to 6.0 (A8 mini).
                                                    The camera does not confirm the zoom, so
                                                    success means the command was sent.
Parameters
    zoom             zoom multiple sent at start-up (default 1.0 = no zoom)
"""
import socket

import rclpy
from geometry_msgs.msg import Vector3Stamped
from rclpy.node import Node
from std_srvs.srv import Trigger
from mission_interfaces.srv import SetZoom

from siyi_a8 import siyi_protocol as siyi


class GimbalNode(Node):
    def __init__(self):
        super().__init__("siyi_gimbal")
        # Defaults are SIYI's factory network settings (as used by mzahana/siyi_sdk).
        self.address = (
            self.declare_parameter("gimbal_ip", "192.168.144.25").value,
            int(self.declare_parameter("gimbal_port", 37260).value),
        )
        self.target_yaw = float(self.declare_parameter("yaw_deg", 0.0).value)
        self.target_pitch = float(self.declare_parameter("pitch_deg", -90.0).value)
        self.tolerance = float(self.declare_parameter("tolerance_deg", 2.0).value)
        attitude_rate = float(self.declare_parameter("attitude_rate_hz", 5.0).value)
        start_zoom = float(self.declare_parameter("zoom", 1.0).value)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)
        self.seq = 0
        self.pointing = True  # still sending the target attitude
        self.last_attitude = None

        self.attitude_pub = self.create_publisher(Vector3Stamped, "~/attitude", 10)
        self.create_service(Trigger, "~/point_down", self.point_down_cb)
        self.create_service(SetZoom, "~/set_zoom", self.set_zoom_cb)
        self.create_timer(1.0, self.send_target)
        self.create_timer(1.0 / attitude_rate, self.request_attitude)
        self.create_timer(0.02, self.receive)

        self.get_logger().info(
            f"Pointing gimbal at {self.address[0]}:{self.address[1]} to yaw {self.target_yaw:.0f} deg, "
            f"pitch {self.target_pitch:.0f} deg...")
        self.set_zoom(start_zoom)

    def send(self, packet):
        try:
            self.sock.sendto(packet, self.address)
        except OSError as exc:
            self.get_logger().warn(f"Cannot reach gimbal: {exc}", throttle_duration_sec=5.0)
        self.seq = (self.seq + 1) & 0xFFFF

    def send_target(self):
        if self.pointing:
            self.send(siyi.set_attitude(self.target_yaw, self.target_pitch, self.seq))

    def request_attitude(self):
        self.send(siyi.request_attitude(self.seq))

    def receive(self):
        while True:
            try:
                packet = self.sock.recv(256)
            except BlockingIOError:
                return
            except OSError as exc:
                self.get_logger().warn(f"Gimbal receive failed: {exc}", throttle_duration_sec=5.0)
                return
            decoded = siyi.decode(packet)
            if decoded is None or decoded[0] != siyi.CMD_ACQUIRE_GIMBAL_ATTITUDE:
                continue
            attitude = siyi.parse_attitude(decoded[1])
            if attitude is not None:
                self.on_attitude(*attitude)

    def on_attitude(self, yaw, pitch, roll):
        self.last_attitude = (yaw, pitch, roll)
        msg = Vector3Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "gimbal"
        msg.vector.x, msg.vector.y, msg.vector.z = yaw, pitch, roll
        self.attitude_pub.publish(msg)

        if self.pointing and abs(pitch - self.target_pitch) <= self.tolerance \
                and abs(yaw - self.target_yaw) <= self.tolerance:
            self.pointing = False
            self.get_logger().info(f"Gimbal at yaw {yaw:.1f} deg, pitch {pitch:.1f} deg.")

    def set_zoom(self, multiple):
        low, high = siyi.A8_MINI_ZOOM_LIMITS
        if not low <= multiple <= high:
            return f"zoom {multiple:.1f} outside {low:.0f}..{high:.0f}"
        self.send(siyi.absolute_zoom(multiple, self.seq))
        self.get_logger().info(f"Zoom set to {multiple:.1f}x (sent; not confirmed by the camera).")
        return None

    def set_zoom_cb(self, request, response):
        error = self.set_zoom(request.zoom)
        response.success = error is None
        response.message = error or f"zoom {request.zoom:.1f}x sent"
        return response

    def point_down_cb(self, request, response):
        self.pointing = True
        self.send_target()
        response.success = True
        response.message = f"sending yaw {self.target_yaw:.0f} deg, pitch {self.target_pitch:.0f} deg"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = GimbalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.sock.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
