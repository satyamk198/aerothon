#!/usr/bin/env python3
"""ROS 2 node: detect QR codes in an image stream and match them to a reference.

Ported from qr_match_node.py in https://github.com/Shiva007-RAndom/QR_Node
(commit 117f34b, Apache-2.0). Changes: node name, result topic and message
types (qr_match_interfaces -> mission_interfaces); the reference can be captured
from the camera in flight (~/capture_reference) and reference_qr_path is optional.
The detection and matching pipeline is unchanged.

Subscribes
    <image_topic>   sensor_msgs/Image (or CompressedImage)

Publishes
    ~/detected   std_msgs/Bool                  any QR present in the frame
    ~/matched    std_msgs/Bool                  a QR matched the reference
    /perception/qr  mission_interfaces/QrDetection  full per-QR detail
    ~/reference_ready  std_msgs/Bool              a reference is loaded (latched)

Action
    ~/capture_reference  mission_interfaces/CaptureReference
        Store a clear QR in view as the reference: exactly one QR, detector
        confidence >= conf_threshold, box fully inside the image. The first such
        frame that also decodes is used; if the first one does not decode, it is
        kept and decodable frames are tried for capture_decode_wait s more (on top
        of the action timeout) before it is used. The padded crop is saved to
        <reference_save_dir>/reference_<time>.png.

The reference can also be preloaded from ``reference_qr_path`` (e.g. for bench
tests); a capture replaces it. Until a reference exists nothing matches. A QR
counts as a match when its similarity score is >= ``match_threshold`` (0.85).

Pipeline (ported from the QR_detection notebook):
    image -> qrdet YOLO detection -> perspective-warp each QR -> threshold
          -> sample into an NxN module grid -> compare vs reference over the
          4 rotations -> score in [0, 1].
If both the reference and a candidate decode cleanly, their payloads are
compared directly instead (an exact test beats a fuzzy one).
"""

from __future__ import annotations

import os
import threading
import time
from typing import List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Bool

from mission_interfaces.action import CaptureReference
from mission_interfaces.msg import QrCode, QrDetection

WARP_SIZE = 300
VALID_MODULE_COUNTS = [21 + 4 * (v - 1) for v in range(1, 15)]  # QR v1..v14


# ======================================================================
# QR pipeline (plain functions -- no ROS, no shared state)
# ======================================================================
def order_corners(pts) -> np.ndarray:
    """Return 4 corners ordered top-left, top-right, bottom-right, bottom-left.

    getPerspectiveTransform needs a known corner order; qrdet returns points
    unsorted, so sort them by angle about the centroid (clockwise in image
    coordinates) and start at the top-left corner.
    """
    pts = np.asarray(pts, dtype=np.float32).reshape(-1, 2)
    centre = pts.mean(axis=0)
    angles = np.arctan2(pts[:, 1] - centre[1], pts[:, 0] - centre[0])
    pts = pts[np.argsort(angles)]
    start = int(np.argmin(pts.sum(axis=1)))
    return np.roll(pts, -start, axis=0).astype(np.float32)


def warp_qr(image_bgr, det, padding: int = 10) -> Tuple[np.ndarray, Optional[np.ndarray]]:
    """Perspective-correct one detection into a square grayscale image.

    Returns (gray_warp, quad); quad is None when no polygon was available and
    a padded bounding-box crop was used instead.
    """
    height, width = image_bgr.shape[:2]

    quad = None
    for key in ("quad_xy", "padded_quad_xy", "polygon_xy"):
        raw = det.get(key)
        if raw is None:
            continue
        p = np.asarray(raw, dtype=np.float32).reshape(-1, 2)
        if p.shape[0] == 4:
            quad = order_corners(p)
        elif p.shape[0] > 4:  # many-vertex polygon -> its min-area box
            quad = order_corners(cv2.boxPoints(cv2.minAreaRect(p)))
        break

    if quad is not None:
        dst = np.array([[0, 0], [WARP_SIZE - 1, 0],
                        [WARP_SIZE - 1, WARP_SIZE - 1], [0, WARP_SIZE - 1]],
                       dtype=np.float32)
        matrix = cv2.getPerspectiveTransform(quad, dst)
        warped = cv2.warpPerspective(image_bgr, matrix, (WARP_SIZE, WARP_SIZE))
        return cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY), quad

    x1, y1, x2, y2 = (int(v) for v in det["bbox_xyxy"])
    cy1, cy2 = max(0, y1 - padding), min(height, y2 + padding)
    cx1, cx2 = max(0, x1 - padding), min(width, x2 + padding)
    crop = image_bgr[cy1:cy2, cx1:cx2]
    if crop.size == 0:
        return np.zeros((WARP_SIZE, WARP_SIZE), np.uint8), None
    gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
    return cv2.resize(gray, (WARP_SIZE, WARP_SIZE), interpolation=cv2.INTER_AREA), None


def normalize_qr(gray) -> np.ndarray:
    """Binarise a warped QR so module colours survive uneven lighting."""
    binary = cv2.adaptiveThreshold(
        gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 51, 10)
    return cv2.morphologyEx(binary, cv2.MORPH_CLOSE, np.ones((2, 2), np.uint8))


def estimate_module_count(binary) -> int:
    """Estimate the module count from run lengths along a few scan lines.

    The most common same-colour run is one module wide, so the median run
    length estimates module size; snap the result to a legal QR size.
    """
    size = binary.shape[0]
    runs: List[int] = []
    for frac in (0.2, 0.35, 0.5, 0.65, 0.8):
        idx = min(int(size * frac), size - 1)
        for line in (binary[idx, :], binary[:, idx]):
            changes = np.flatnonzero(np.diff((line < 128).astype(np.uint8)))
            if changes.size >= 2:
                runs.extend(np.diff(changes).tolist())
    if not runs or np.median(runs) <= 0:
        return 25
    modules = size / float(np.median(runs))
    return min(VALID_MODULE_COUNTS, key=lambda v: abs(v - modules))


def extract_module_grid(binary, grid_size: Optional[int] = None) -> np.ndarray:
    """Sample a binarised QR into an NxN uint8 grid (1 = dark module).

    Each module is read as the median of a small patch, which is far less
    sensitive to sensor noise than a single-pixel sample.
    """
    size = binary.shape[0]
    if grid_size is None:
        grid_size = estimate_module_count(binary)
    cell = size / grid_size
    half = max(1, int(cell * 0.25))
    grid = np.zeros((grid_size, grid_size), np.uint8)
    for r in range(grid_size):
        cy = min(int(r * cell + cell / 2), size - 1)
        y0, y1 = max(0, cy - half), min(size, cy + half + 1)
        for c in range(grid_size):
            cx = min(int(c * cell + cell / 2), size - 1)
            x0, x1 = max(0, cx - half), min(size, cx + half + 1)
            grid[r, c] = 1 if np.median(binary[y0:y1, x0:x1]) < 128 else 0
    return grid


def compute_match_score(grid_a, grid_b) -> float:
    """Fraction of modules that agree, maximised over the 4 rotations."""
    if grid_a.shape != grid_b.shape:
        size = max(grid_a.shape[0], grid_b.shape[0])
        grid_a = cv2.resize(grid_a, (size, size), interpolation=cv2.INTER_NEAREST)
        grid_b = cv2.resize(grid_b, (size, size), interpolation=cv2.INTER_NEAREST)
    best, cand = 0.0, grid_b.copy()
    for _ in range(4):
        best = max(best, float(np.sum(grid_a == cand) / grid_a.size))
        cand = np.rot90(cand)
    return best


def build_decoder():
    """Return (backend_name, impl) for the best available QR text decoder."""
    try:
        return "cv2", cv2.QRCodeDetector()
    except Exception:  # noqa: BLE001
        pass
    try:
        from pyzbar import pyzbar  # noqa: F401
        return "pyzbar", None
    except ImportError:
        return "none", None


def decode_qr(crop_bgr, backend, impl) -> Optional[str]:
    """Decode a QR crop to text, or return None. A failure is never fatal."""
    if crop_bgr is None or crop_bgr.size == 0:
        return None
    try:
        if backend == "cv2":
            data, _, _ = impl.detectAndDecode(crop_bgr)
            return data or None
        if backend == "pyzbar":
            from pyzbar import pyzbar
            found = pyzbar.decode(crop_bgr)
            return found[0].data.decode("utf-8") if found else None
    except Exception:  # noqa: BLE001
        return None
    return None


# ======================================================================
# ROS 2 node
# ======================================================================
class QRMatchNode(Node):
    def __init__(self) -> None:
        super().__init__("qr_detector")

        # ---- parameters ----
        self.declare_parameter("reference_qr_path", "")
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("use_compressed", False)
        self.declare_parameter("match_threshold", 0.85)
        self.declare_parameter("model_size", "s")
        self.declare_parameter("conf_threshold", 0.5)
        self.declare_parameter("trust_decoded_text", True)
        # s to keep looking for a decodable QR after the first complete but undecodable one
        self.declare_parameter("capture_decode_wait", 5.0)
        self.declare_parameter("reference_save_dir",
                               os.path.join(os.path.expanduser("~"), ".ros", "qr_reference"))
        self.declare_parameter("process_every_n", 1)
        self.declare_parameter("best_effort_qos", True)
        self.declare_parameter("log_every_frame", False)

        reference_path = self.get_parameter("reference_qr_path").value
        image_topic = self.get_parameter("image_topic").value
        self.use_compressed = bool(self.get_parameter("use_compressed").value)
        self.threshold = float(self.get_parameter("match_threshold").value)
        self.trust_text = bool(self.get_parameter("trust_decoded_text").value)
        self.every_n = max(1, int(self.get_parameter("process_every_n").value))
        self.log_every = bool(self.get_parameter("log_every_frame").value)
        self.conf_threshold = float(self.get_parameter("conf_threshold").value)
        self.save_dir = self.get_parameter("reference_save_dir").value
        self.decode_wait = float(self.get_parameter("capture_decode_wait").value)

        # ---- detector + decoder ----
        # Load weights here rather than in the callback so the first frame
        # doesn't pay the cost.
        self.get_logger().info("Loading QR detector and reference image...")
        from qrdet import QRDetector
        try:
            self.detector = QRDetector(
                model_size=self.get_parameter("model_size").value,
                conf_th=float(self.get_parameter("conf_threshold").value))
        except TypeError:  # older qrdet has no conf_th
            self.detector = QRDetector(
                model_size=self.get_parameter("model_size").value)

        self.decoder_backend, self.decoder_impl = build_decoder()
        self.get_logger().info(f"QR text decoder: {self.decoder_backend}")

        self.ref_grid, self.ref_text, self.ref_size = None, None, None
        if reference_path:
            self.ref_grid, self.ref_text = self._load_reference(reference_path)
            self.ref_size = int(self.ref_grid.shape[0])
            self.get_logger().info(
                f"Reference loaded from {reference_path} "
                f"({self.ref_size}x{self.ref_size} modules, "
                f"text={self.ref_text or 'n/a'})")
        else:
            self.get_logger().info("No reference yet: waiting for ~/capture_reference.")

        # In-flight capture, shared between the image callback and the action thread
        self.lock = threading.Lock()
        self.capture_requested = False
        self.capture_done = threading.Event()
        self.capture_path = ""
        self.capture_fallback = None  # first complete but undecodable QR of this capture
        self.capture_fallback_time = 0.0

        self.bridge = CvBridge()
        self.frame_count = 0

        # ---- QoS ----
        # Cameras usually publish best-effort; results are latched so a
        # late subscriber immediately sees the latest verdict.
        rel = (QoSReliabilityPolicy.BEST_EFFORT
               if bool(self.get_parameter("best_effort_qos").value)
               else QoSReliabilityPolicy.RELIABLE)
        image_qos = QoSProfile(reliability=rel,
                               history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        result_qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                                history=QoSHistoryPolicy.KEEP_LAST, depth=1)

        # ---- pub/sub ----
        msg_type = CompressedImage if self.use_compressed else Image
        self.create_subscription(msg_type, image_topic, self.on_image, image_qos)
        self.pub_detected = self.create_publisher(Bool, "~/detected", result_qos)
        self.pub_matched = self.create_publisher(Bool, "~/matched", result_qos)
        self.pub_result = self.create_publisher(QrDetection, "/perception/qr", result_qos)
        self.pub_ref_ready = self.create_publisher(Bool, "~/reference_ready", result_qos)
        self.pub_ref_ready.publish(Bool(data=self.ref_grid is not None))
        # The action waits in its own thread while on_image keeps running.
        ActionServer(self, CaptureReference, "~/capture_reference", self.execute_capture,
                     cancel_callback=lambda _: CancelResponse.ACCEPT,
                     callback_group=ReentrantCallbackGroup())

        self.get_logger().info(
            f"Ready. Listening on '{image_topic}', threshold {self.threshold:.2f}, "
            f"1 in {self.every_n} frame(s).")

    # ---- reference ----
    def _load_reference(self, path: str):
        image = cv2.imread(path)
        if image is None:
            raise FileNotFoundError(f"Could not read reference QR: {path}")
        dets = self._detect(image, grid_size=None)
        if dets:
            grid, _, _, text = max(dets, key=lambda d: d[2])  # highest confidence
            return grid, text
        # A cleanly cropped PNG can fail YOLO; treat the whole image as the QR.
        self.get_logger().warn(
            "No QR detected in reference; using the whole image as the QR.")
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        dark = (gray < 128).astype(np.uint8)
        rows, cols = np.flatnonzero(dark.any(1)), np.flatnonzero(dark.any(0))
        if rows.size and cols.size:
            gray = gray[rows[0]:rows[-1] + 1, cols[0]:cols[-1] + 1]
        gray = cv2.resize(gray, (WARP_SIZE, WARP_SIZE), interpolation=cv2.INTER_AREA)
        grid = extract_module_grid(normalize_qr(gray))
        return grid, decode_qr(image, self.decoder_backend, self.decoder_impl)

    # ---- detection: returns list of (grid, bbox, confidence, decoded_text) ----
    def _detect(self, image_bgr, grid_size):
        rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
        height, width = image_bgr.shape[:2]
        out = []
        for det in self.detector.detect(image=rgb, is_bgr=False):
            if "bbox_xyxy" not in det:
                continue
            warped, _ = warp_qr(image_bgr, det)
            grid = extract_module_grid(normalize_qr(warped), grid_size)
            x1, y1, x2, y2 = (int(v) for v in det["bbox_xyxy"])
            x1, y1, x2, y2 = max(0, x1), max(0, y1), min(width, x2), min(height, y2)
            crop = image_bgr[max(0, y1 - 10):min(height, y2 + 10),
                             max(0, x1 - 10):min(width, x2 + 10)]
            text = decode_qr(crop, self.decoder_backend, self.decoder_impl)
            out.append((grid, (x1, y1, x2, y2),
                        float(det.get("confidence", 0.0)), text))
        return out

    # ---- in-flight reference capture ----
    def _try_capture(self, frame) -> None:
        """Check one frame for a clear start QR (exactly one QR, confidence >= conf_threshold,
        box fully inside the image). A decodable one becomes the reference at once; the first
        undecodable one is kept as fallback while decodable frames are tried for decode_wait s."""
        try:
            dets = self._detect(frame, grid_size=None)
        except Exception as exc:  # noqa: BLE001 - one bad frame must not kill the node
            self.get_logger().error(f"Capture failed on this frame: {exc}")
            return
        height, width = frame.shape[:2]
        if len(dets) != 1:
            return
        grid, (x1, y1, x2, y2), conf, text = dets[0]
        # _detect clamps boxes to the image, so a box touching an edge was cut off.
        if conf < self.conf_threshold or x1 <= 0 or y1 <= 0 or x2 >= width or y2 >= height:
            return

        crop = frame[max(0, y1 - 10):min(height, y2 + 10), max(0, x1 - 10):min(width, x2 + 10)]
        candidate = {"grid": grid, "text": text, "conf": conf, "box": (x2 - x1, y2 - y1), "crop": crop}
        if text:
            self._commit_capture(candidate)
            return
        with self.lock:
            if self.capture_fallback is None and self.capture_requested:
                self.capture_fallback = candidate
                self.capture_fallback_time = time.monotonic()
                self.get_logger().info(
                    f"Complete QR found but not decodable; trying {self.decode_wait:.0f} s more "
                    "for a decodable frame.")

    def _commit_capture(self, candidate) -> None:
        """Store `candidate` as the reference (once per capture) and save its crop."""
        with self.lock:
            if not self.capture_requested:
                return  # already committed, or the capture ended
            self.capture_requested = False

        path = os.path.join(self.save_dir, time.strftime("reference_%Y%m%d-%H%M%S.png"))
        try:
            os.makedirs(self.save_dir, exist_ok=True)
            saved = cv2.imwrite(path, candidate["crop"])
        except (OSError, cv2.error):
            saved = False
        if not saved:
            self.get_logger().error(f"Could not save the reference image to {path}")

        grid, text = candidate["grid"], candidate["text"]
        with self.lock:
            self.ref_grid, self.ref_text = grid, text  # as _load_reference: grid + decoded text
            self.ref_size = int(grid.shape[0])
            self.capture_path = path if saved else ""
        self.capture_done.set()
        self.pub_ref_ready.publish(Bool(data=True))
        self.get_logger().info(
            f"Reference captured: {self.ref_size}x{self.ref_size} modules, text={text or 'n/a'}, "
            f"confidence {candidate['conf']:.2f}, box {candidate['box'][0]}x{candidate['box'][1]} px"
            + (f", saved to {path}" if saved else ""))

    def execute_capture(self, goal_handle):
        timeout = float(goal_handle.request.timeout)
        result = CaptureReference.Result()
        if timeout <= 0.0:
            goal_handle.abort()
            result.message = "timeout must be > 0"
            return result

        self.get_logger().info(f"Capturing reference QR (timeout {timeout:.0f} s)...")
        with self.lock:
            self.capture_done.clear()
            self.capture_path = ""
            self.capture_fallback = None
            self.capture_requested = True
        start = time.monotonic()
        feedback = CaptureReference.Feedback()
        while not self.capture_done.wait(0.5):
            now = time.monotonic()
            with self.lock:
                fallback, fallback_time = self.capture_fallback, self.capture_fallback_time
            # The decode_wait after a fallback runs on top of the timeout.
            if fallback is not None and now - fallback_time >= self.decode_wait:
                self.get_logger().info("No decodable frame; using the first complete QR frame.")
                self._commit_capture(fallback)
                continue
            if goal_handle.is_cancel_requested or (fallback is None and now - start > timeout):
                with self.lock:
                    self.capture_requested = False
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.message = "canceled"
                else:
                    goal_handle.abort()
                    result.message = f"no clear QR seen within {timeout:.0f} s"
                    self.get_logger().error(f"Reference capture failed: {result.message}")
                return result
            feedback.elapsed = float(now - start)
            goal_handle.publish_feedback(feedback)

        goal_handle.succeed()
        result.success = True
        result.message = "ok"
        result.image_path = self.capture_path
        return result

    # ---- image callback ----
    def on_image(self, msg) -> None:
        self.frame_count += 1
        if self.frame_count % self.every_n != 0:
            return
        try:
            if self.use_compressed:
                frame = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
            else:
                frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as exc:
            self.get_logger().error(f"cv_bridge failed: {exc}")
            return
        if frame is None or frame.size == 0:
            return

        with self.lock:
            capturing = self.capture_requested
        if capturing:
            self._try_capture(frame)
            return

        start = time.perf_counter()
        try:
            dets = self._detect(frame, grid_size=self.ref_size)
        except Exception as exc:  # noqa: BLE001 - one bad frame must not kill the node
            self.get_logger().error(f"Processing failed on this frame: {exc}")
            return
        elapsed_ms = (time.perf_counter() - start) * 1000.0

        # score each detection
        scored = []
        for grid, bbox, conf, text in dets:
            if self.ref_grid is None:  # no reference yet: nothing can match
                score, from_text = 0.0, False
            elif self.trust_text and self.ref_text and text:
                score = 1.0 if text == self.ref_text else 0.0
                from_text = True
            else:
                score = compute_match_score(self.ref_grid, grid)
                from_text = False
            scored.append((score, bbox, conf, text, from_text))
        scored.sort(key=lambda s: s[0], reverse=True)

        detected = len(scored) > 0
        best = scored[0][0] if scored else 0.0
        matched = best >= self.threshold

        # publish
        result = QrDetection()
        result.header = msg.header
        result.image_height, result.image_width = frame.shape[:2]
        result.qr_detected = detected
        result.num_detections = len(scored)
        result.match_found = matched
        result.best_score = float(best)
        result.match_threshold = float(self.threshold)
        result.processing_time_ms = float(elapsed_ms)
        for score, bbox, conf, text, from_text in scored:
            d = QrCode()
            d.bbox_xyxy = [int(v) for v in bbox]
            d.detector_confidence = float(conf)
            d.score = float(score)
            d.matched = bool(score >= self.threshold)
            d.decoded_text = text or ""
            d.score_from_decoded_text = bool(from_text)
            result.detections.append(d)
        self.pub_result.publish(result)
        self.pub_detected.publish(Bool(data=detected))
        self.pub_matched.publish(Bool(data=matched))

        # log
        if matched:
            self.get_logger().info(
                f"MATCH: best={best:.3f} over {len(scored)} QR(s) "
                f"({elapsed_ms:.0f} ms)")
        elif self.log_every:
            state = f"{len(scored)} QR(s), best={best:.3f}" if detected else "no QR"
            self.get_logger().info(f"{state} ({elapsed_ms:.0f} ms)")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = QRMatchNode()
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception as exc:  # noqa: BLE001
        print(f"qr_detector terminated: {exc}")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
