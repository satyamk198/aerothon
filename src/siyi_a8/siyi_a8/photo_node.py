"""4K photo loop for the SIYI A8 mini: while a mission step needs images, take a photo, download
it from the camera's SD card, save + geotag it, publish it, repeat. All detectors read the
published images.

The loop starts stopped. The mission tree starts it (~/set_task with the step's folder name) only
while a step that processes images runs (start-QR capture, banner alignment, corridor, target
search, centering + delivery), so no photos are taken during e.g. takeoff or altitude changes.

Taking the photo uses the SIYI SDK photo command (siyi_protocol.take_picture, UDP 37260).
Finding and downloading it is ported from capture_and_save_image() in "airdrop survey node.py":
the camera's HTTP media API on port 82 (getdirectories -> last directory, getmedialist -> last
file), 1.0 s wait after the photo command, file URL with the camera's default IP replaced.
Changes from that file:
  - newness check: the latest file is noted before the photo command, and only a different file
    is accepted (polled every 0.5 s for up to new_photo_timeout s after the 1.0 s wait)
  - geotag written right after the download (no 10 s delay), pose from PX4 instead of MAVROS:
    lat/lon/AMSL altitude from vehicle_global_position, roll/pitch/yaw (NED, yaw = heading
    from north) from vehicle_attitude, all snapshotted when the photo command is sent
  - downloads time out after DOWNLOAD_TIMEOUT s

Saved per run and per mission step: <photo_dir>/<run>/<task>/<sequence:06d>.jpg, with the poses
appended to <photo_dir>/<run>/<task>/sequence.jsonl (as in the original). <run> is the date-time
this node started; <photo_dir>/latest points to the current run. Nothing is moved afterwards,
so every run stays as its own backup even after a crash or power cut. A photo goes to the task
that was active when it was triggered.

Publishes
    <image_topic>   sensor_msgs/Image (bgr8), default /camera/image_raw. header.stamp is the time
                    the photo command was sent.
Services
    ~/set_task      mission_interfaces/SetPhotoTask  task name: run the loop saving into that task's
                    folder; "": stop after the current photo
"""
import json
import math
import os
import re
import socket
import threading
import time
import urllib.parse
import urllib.request

import cv2
import numpy as np
import piexif
import rclpy
from cv_bridge import CvBridge
from px4_msgs.msg import VehicleAttitude, VehicleGlobalPosition
from rclpy.node import Node
from sensor_msgs.msg import Image
from mission_interfaces.srv import SetPhotoTask

from flight_controller.px4_utils import PX4_QOS, px4_topic
from siyi_a8 import siyi_protocol as siyi

CAMERA_HTTP_PORT = 82
CAMERA_IP_DEFAULT = "192.168.144.25"  # camera reports this in its file URLs regardless of actual IP
SD_WRITE_WAIT = 1.0  # s, "let camera finish writing the file to SD card"
POLL_INTERVAL = 0.5  # s between SD-card listings while waiting for the new photo
LIST_TIMEOUT = 3.0  # s per HTTP listing request
DOWNLOAD_TIMEOUT = 30.0  # s for downloading one photo


# ---------------- SIYI HTTP media API (ported from "airdrop survey node.py") ----------------

def camera_dirlist_url(camera_ip, http_port, media_type=0):
    params = {"media_type": media_type}
    return (f"http://{camera_ip}:{http_port}/cgi-bin/media.cgi/api/v1/getdirectories?"
            + urllib.parse.urlencode(params))


def camera_filelist_url(camera_ip, http_port, media_type, dir_path):
    params = {"media_type": str(media_type), "path": dir_path, "start": 0, "count": 9999}
    return (f"http://{camera_ip}:{http_port}/cgi-bin/media.cgi/api/v1/getmedialist?"
            + urllib.parse.urlencode(params))


def latest_sd_image_url(camera_ip, http_port=CAMERA_HTTP_PORT):
    """URL of the most recently listed image on the SD card, or None (no images / API error).
    Raises on network errors."""
    with urllib.request.urlopen(camera_dirlist_url(camera_ip, http_port, media_type=0), timeout=LIST_TIMEOUT) as resp:
        dir_resp = json.load(resp)
    if not dir_resp.get("success"):
        return None
    directories = dir_resp["data"]["directories"]
    if not directories:
        return None
    dir_path = directories[-1]["path"]  # assumes last-listed dir is most recent

    with urllib.request.urlopen(camera_filelist_url(camera_ip, http_port, 0, dir_path), timeout=LIST_TIMEOUT) as resp:
        file_resp = json.load(resp)
    if not file_resp.get("success"):
        return None
    files = file_resp["data"]["list"]
    if not files:
        return None
    return files[-1]["url"].replace(CAMERA_IP_DEFAULT, camera_ip)  # assumes last-listed file is most recent


# ---------------- geotag (ported from add_geotag_metadata) ----------------

def decimal_to_dms(decimal):
    dd = abs(decimal)
    degrees = int(dd)
    minutes_full = (dd - degrees) * 60
    minutes = int(minutes_full)
    seconds = round((minutes_full - minutes) * 60 * 100)
    return [(degrees, 1), (minutes, 1), (int(seconds), 100)]


def add_geotag_metadata(image_path, pose, width, height):
    lat, lon, alt = pose["lat"], pose["lon"], pose["alt"]
    pitch, roll, yaw = pose["pitch"], pose["roll"], pose["yaw"]
    comment_payload = {
        "Pitch": round(pitch, 2) if pitch is not None else 0.0,
        "Roll": round(roll, 2) if roll is not None else 0.0,
        "Yaw": round(yaw, 2) if yaw is not None else 0.0,
    }
    user_comment_bytes = b"ASCII\x00\x00\x00" + json.dumps(comment_payload).encode("utf-8")
    exif_dict = {
        "0th": {
            piexif.ImageIFD.ImageLength: int(height),
            piexif.ImageIFD.ImageWidth: int(width),
        },
        "Exif": {
            piexif.ExifIFD.UserComment: user_comment_bytes,
        },
        "GPS": {
            piexif.GPSIFD.GPSLatitudeRef: "N" if lat >= 0 else "S",
            piexif.GPSIFD.GPSLatitude: decimal_to_dms(lat),
            piexif.GPSIFD.GPSLongitudeRef: "E" if lon >= 0 else "W",
            piexif.GPSIFD.GPSLongitude: decimal_to_dms(lon),
            piexif.GPSIFD.GPSAltitudeRef: 0,
            piexif.GPSIFD.GPSAltitude: (int(abs(alt * 100)), 100),
            piexif.GPSIFD.GPSImgDirectionRef: "T",
            piexif.GPSIFD.GPSImgDirection: (int(round(yaw * 100)), 100) if yaw is not None else (0, 100),
        },
    }
    piexif.insert(piexif.dump(exif_dict), image_path)


def quaternion_to_euler_deg(q):
    """PX4 q = [w, x, y, z] (FRD body -> NED). Returns roll, pitch, yaw in degrees, each 0..360
    like the original (yaw = heading from north, clockwise)."""
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return tuple((math.degrees(a) + 360) % 360 for a in (roll, pitch, yaw))


def next_sequence_index(save_dir):
    """Continue numbering after the highest <NNNNNN>.jpg already in save_dir."""
    numbers = [int(m.group(1)) for f in os.listdir(save_dir) if (m := re.fullmatch(r"(\d{6})\.jpg", f))]
    return max(numbers) + 1 if numbers else 0


class PhotoNode(Node):
    def __init__(self):
        super().__init__("siyi_photo")
        # Defaults are SIYI's factory network settings (as used by mzahana/siyi_sdk).
        self.camera_ip = self.declare_parameter("camera_ip", CAMERA_IP_DEFAULT).value
        self.address = (self.camera_ip, int(self.declare_parameter("camera_port", 37260).value))
        self.http_port = int(self.declare_parameter("camera_http_port", CAMERA_HTTP_PORT).value)
        self.frame_id = self.declare_parameter("frame_id", "camera").value
        topic = self.declare_parameter("image_topic", "/camera/image_raw").value
        self.new_photo_timeout = float(self.declare_parameter("new_photo_timeout", 3.0).value)
        photo_dir = self.declare_parameter("photo_dir", "").value
        if not photo_dir:
            raise RuntimeError("Parameter 'photo_dir' (folder for the downloaded photos) is required")
        self.run_dir = os.path.join(os.path.expanduser(photo_dir), time.strftime("%Y%m%d-%H%M%S"))
        os.makedirs(self.run_dir, exist_ok=True)
        self.point_latest_to_run(photo_dir)
        self.sequence = {}  # task -> next sequence number in that task's folder

        self.global_pos = None  # latest VehicleGlobalPosition
        self.attitude = None  # latest VehicleAttitude
        self.create_subscription(
            VehicleGlobalPosition, px4_topic("/fmu/out/vehicle_global_position", VehicleGlobalPosition),
            self.global_pos_cb, PX4_QOS)
        self.create_subscription(
            VehicleAttitude, px4_topic("/fmu/out/vehicle_attitude", VehicleAttitude),
            self.attitude_cb, PX4_QOS)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.seq = 0
        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, topic, 1)
        self.running = True
        self.task = ""  # folder name of the mission step that needs photos; "" = stopped
        self.enabled = threading.Event()  # set while self.task is not ""
        self.create_service(SetPhotoTask, "~/set_task", self.set_task_cb)
        self.thread = threading.Thread(target=self.loop, daemon=True)
        self.thread.start()
        self.get_logger().info(
            f"Photo loop ready (off until ~/set_task): camera {self.camera_ip}, saving to {self.run_dir} "
            f"-> {topic}")

    def point_latest_to_run(self, photo_dir):
        """Make <photo_dir>/latest point at this run's folder (replaced atomically)."""
        latest = os.path.join(os.path.expanduser(photo_dir), "latest")
        tmp = latest + ".tmp"
        try:
            if os.path.lexists(tmp):
                os.remove(tmp)
            os.symlink(os.path.basename(self.run_dir), tmp)
            os.replace(tmp, latest)
        except OSError as exc:
            self.get_logger().warn(f"Could not update {latest}: {exc}")

    def global_pos_cb(self, msg):
        self.global_pos = msg

    def attitude_cb(self, msg):
        self.attitude = msg

    def set_task_cb(self, request, response):
        task = request.task.strip()
        if task and not re.fullmatch(r"[A-Za-z0-9_-]+", task):
            response.success = False
            response.message = f"invalid task name {task!r} (letters, digits, _ or - only)"
            return response
        if task != self.task:
            self.get_logger().info(f"Photo loop ON for '{task}'" if task else "Photo loop OFF")
        self.task = task
        if task:
            self.enabled.set()
        else:
            self.enabled.clear()
        response.success = True
        response.message = f"photos -> {os.path.join(self.run_dir, task)}" if task else "stopped"
        return response

    def pose_snapshot(self):
        """Pose at the instant the photo is triggered (lat/lon/alt None without a valid fix)."""
        g, a = self.global_pos, self.attitude
        pose = {"lat": None, "lon": None, "alt": None, "pitch": None, "roll": None, "yaw": None}
        if g is not None and g.lat_lon_valid and g.alt_valid:
            pose.update(lat=g.lat, lon=g.lon, alt=float(g.alt))
        if a is not None:
            pose["roll"], pose["pitch"], pose["yaw"] = quaternion_to_euler_deg(a.q)
        return pose

    def take_new_photo(self):
        """Trigger a photo and return (jpeg bytes, pose, stamp) of the new file, or None."""
        try:
            before = latest_sd_image_url(self.camera_ip, self.http_port)
        except Exception as exc:  # noqa: BLE001 - network errors: retry next cycle
            self.get_logger().warn(f"SD card listing failed: {exc}", throttle_duration_sec=5.0)
            return None

        pose = self.pose_snapshot()
        stamp = self.get_clock().now().to_msg()
        try:
            self.sock.sendto(siyi.take_picture(self.seq), self.address)
        except OSError as exc:
            self.get_logger().warn(f"Cannot reach camera: {exc}", throttle_duration_sec=5.0)
            return None
        self.seq = (self.seq + 1) & 0xFFFF

        time.sleep(SD_WRITE_WAIT)
        deadline = time.monotonic() + self.new_photo_timeout
        while True:
            try:
                url = latest_sd_image_url(self.camera_ip, self.http_port)
            except Exception as exc:  # noqa: BLE001
                url = None
                self.get_logger().warn(f"SD card listing failed: {exc}", throttle_duration_sec=5.0)
            if url is not None and url != before:
                break
            if time.monotonic() >= deadline:
                self.get_logger().warn(
                    f"No new photo on the SD card {SD_WRITE_WAIT + self.new_photo_timeout:.0f} s after the "
                    "photo command.")
                return None
            time.sleep(POLL_INTERVAL)

        try:
            with urllib.request.urlopen(url, timeout=DOWNLOAD_TIMEOUT) as resp:
                return resp.read(), pose, stamp
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Photo download failed ({url}): {exc}")
            return None

    def save_and_geotag(self, task, jpeg, image, pose):
        task_dir = os.path.join(self.run_dir, task)
        os.makedirs(task_dir, exist_ok=True)
        if task not in self.sequence:
            self.sequence[task] = next_sequence_index(task_dir)
        index = self.sequence[task]
        self.sequence[task] += 1
        path = os.path.join(task_dir, f"{index:06d}.jpg")
        with open(path, "wb") as f:
            f.write(jpeg)
        if all(pose[k] is not None for k in ("lat", "lon", "alt")):
            height, width = image.shape[:2]
            try:
                add_geotag_metadata(path, pose, width, height)
            except Exception as exc:  # noqa: BLE001 - keep the photo even if EXIF fails
                self.get_logger().error(f"Geotag failed for {index:06d}: {exc}")
        else:
            self.get_logger().warn(f"No GPS fix at capture time for {index:06d}; skipping geotag.")
        entry = dict(sequence=index, **{k: pose[k] for k in ("lat", "lon", "alt", "pitch", "roll", "yaw")})
        with open(os.path.join(task_dir, "sequence.jsonl"), "a") as f:
            f.write(json.dumps(entry) + "\n")
        self.get_logger().info(f"Photo {task}/{index:06d} saved -> {path}")

    def loop(self):
        while self.running and rclpy.ok():
            if not self.enabled.wait(timeout=0.5):
                continue
            task = self.task  # the photo belongs to the step active when it is triggered
            if not task:
                continue
            photo = self.take_new_photo()
            if photo is None:
                continue
            jpeg, pose, stamp = photo
            image = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
            if image is None:
                self.get_logger().error("Downloaded file is not a readable image.")
                continue
            try:
                self.save_and_geotag(task, jpeg, image, pose)
            except OSError as exc:
                self.get_logger().error(f"Saving the photo failed: {exc}")

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
