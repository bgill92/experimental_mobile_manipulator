"""
Find the red pick-and-place block in the wrist camera's colour and depth images.

    ros2 run emma_perception block_detector --ros-args -p use_sim_time:=true

Classical pipeline: HSV threshold → largest blob → backproject every blob pixel with its
depth → transform the points to `target_frame` (z up) → the top face's points give the
centre and yaw → publish `/block_pose`. The pure functions at the top hold the maths and
are unit tested without ROS.
"""

from collections.abc import Sequence
import math
import sys

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, TransformStamped
import message_filters
import numpy as np
from numpy.typing import ArrayLike
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener

# Depth readings outside this range (metres) are not the block: the Gemini 305 sees from
# 4 cm, and the block sits about 20 cm away at the look pose.
MIN_DEPTH = 0.04
MAX_DEPTH = 0.6

# Intrinsics as (fx, fy, cx, cy).
Intrinsics = tuple[float, float, float, float]


def segment(bgr: np.ndarray, lower: Sequence[int], upper: Sequence[int]) -> np.ndarray:
    """
    Mask (uint8, 0 or 255) of pixels whose OpenCV HSV lies in [lower, upper].

    Red hue wraps around 0, so a range starting at hue 0 also takes hues 170..180.
    """
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    lo = np.array(lower, dtype=np.uint8)
    hi = np.array(upper, dtype=np.uint8)
    mask = cv2.inRange(hsv, lo, hi)
    if lower[0] == 0:
        wrap_lo = np.array([170, lower[1], lower[2]], dtype=np.uint8)
        wrap_hi = np.array([180, upper[1], upper[2]], dtype=np.uint8)
        mask = cv2.bitwise_or(mask, cv2.inRange(hsv, wrap_lo, wrap_hi))
    return mask


def largest_blob(mask: np.ndarray, min_area: float = 30.0) -> np.ndarray | None:
    """Return the largest blob of `mask`, filled, as its own mask; None if under `min_area` px."""
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    blob = max(contours, key=cv2.contourArea)
    if cv2.contourArea(blob) < min_area:
        return None
    filled = np.zeros_like(mask)
    cv2.drawContours(filled, [blob], -1, 255, thickness=cv2.FILLED)
    return filled


def locate(mask: np.ndarray, depth: np.ndarray, min_area: float = 30.0
           ) -> tuple[float, float, float, float] | None:
    """
    Centre (u, v), depth d and in-image angle of the largest blob in `mask`, or None.

    The angle (radians) is the direction of the blob's minimum-area rectangle's first side.
    Depth is the median over the blob, so stray pixels at the edges do not matter. Returns
    None if the blob is smaller than `min_area` pixels or its depth is out of range.
    """
    blob = largest_blob(mask, min_area)
    if blob is None:
        return None
    contours, _ = cv2.findContours(blob, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    (u, v), _, angle_deg = cv2.minAreaRect(contours[0])
    values = depth[(blob > 0) & np.isfinite(depth)]
    if values.size == 0:
        return None
    d = float(np.median(values))
    if not MIN_DEPTH <= d <= MAX_DEPTH:
        return None
    return float(u), float(v), d, math.radians(angle_deg)


def backproject(u: ArrayLike, v: ArrayLike, d: ArrayLike, k: Intrinsics) -> np.ndarray:
    """Point(s) in the optical frame (z forward) for pixel(s) (u, v) at depth d, as [..., 3]."""
    fx, fy, cx, cy = k
    d = np.asarray(d, dtype=float)
    return np.stack([(np.asarray(u) - cx) * d / fx, (np.asarray(v) - cy) * d / fy, d], axis=-1)


def blob_points(blob: np.ndarray, depth: np.ndarray, k: Intrinsics) -> np.ndarray:
    """N x 3 optical-frame points of the pixels in `blob` whose depth is in range."""
    v, u = np.nonzero(blob)
    d = depth[v, u]
    keep = np.isfinite(d) & (d >= MIN_DEPTH) & (d <= MAX_DEPTH)
    return backproject(u[keep], v[keep], d[keep], k)


def top_face(points: np.ndarray, cube_size: float, band: float = 0.002
             ) -> tuple[np.ndarray, float] | None:
    """
    Cube centre and yaw from its visible surface points in a z-up frame.

    The camera looks down at an angle, so it sees the top face and one or two sides. The
    points within `band` of the top are the top face: the rectangle around them gives the
    centre in x, y and the yaw (wrapped to a quarter turn), and the centre lies half a cube
    below the top. The 95th percentile stands in for the top so a few stray points cannot
    lift it. None with fewer than 3 top points.
    """
    top_z = float(np.percentile(points[:, 2], 95))
    top = points[points[:, 2] > top_z - band]
    if len(top) < 3:
        return None
    (x, y), _, angle_deg = cv2.minAreaRect(top[:, :2].astype(np.float32))
    centre = np.array([x, y, top_z - cube_size / 2])
    return centre, wrap_quarter_turn(math.radians(angle_deg))


def transform_matrix(transform: TransformStamped) -> np.ndarray:
    """4x4 homogeneous matrix of a TransformStamped."""
    t, q = transform.transform.translation, transform.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    matrix = np.eye(4)
    matrix[:3, :3] = [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]
    matrix[:3, 3] = (t.x, t.y, t.z)
    return matrix


def wrap_quarter_turn(angle: float) -> float:
    """Wrap to [-pi/4, pi/4): a cube looks the same every 90 degrees."""
    quarter = math.pi / 2
    return (angle + quarter / 2) % quarter - quarter / 2


class BlockDetector(Node):
    """Publish the block's pose on `/block_pose` for every synchronised image pair."""

    def __init__(self) -> None:
        super().__init__('block_detector')
        self.hsv_lower = list(self.declare_parameter('hsv_lower', [0, 120, 70]).value)
        self.hsv_upper = list(self.declare_parameter('hsv_upper', [10, 255, 255]).value)
        self.min_area = float(self.declare_parameter('min_area_px', 30).value)
        self.target_frame = str(self.declare_parameter('target_frame', 'base_link').value)
        self.cube_size = float(self.declare_parameter('cube_size', 0.01).value)
        self.debug = bool(self.declare_parameter('debug', False).value)
        color_topic = self.declare_parameter(
            'color_topic', '/wrist_camera/color/image_raw').value
        depth_topic = self.declare_parameter(
            'depth_topic', '/wrist_camera/depth/image_raw').value
        info_topic = self.declare_parameter(
            'info_topic', '/wrist_camera/color/camera_info').value

        self.bridge = CvBridge()
        self.k: Intrinsics | None = None
        self.tf_buffer = Buffer()
        # The listener's callbacks are reentrant; with main()'s multi-threaded executor they
        # keep running while an image callback blocks in lookup_transform waiting for them.
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pose_pub = self.create_publisher(PoseStamped, '/block_pose', 10)
        self.debug_pub = (self.create_publisher(Image, '~/debug_image', 1)
                          if self.debug else None)
        self.create_subscription(CameraInfo, info_topic, self.on_info, qos_profile_sensor_data)
        color = message_filters.Subscriber(
            self, Image, color_topic, qos_profile=qos_profile_sensor_data)
        depth = message_filters.Subscriber(
            self, Image, depth_topic, qos_profile=qos_profile_sensor_data)
        self.sync = message_filters.ApproximateTimeSynchronizer([color, depth], 5, 0.05)
        self.sync.registerCallback(self.on_images)
        self._last_warning = ''

    def on_info(self, msg: CameraInfo) -> None:
        self.k = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])

    def warn(self, message: str) -> None:
        # Say each distinct problem once rather than at the camera rate.
        if message != self._last_warning:
            self.get_logger().warning(message)
            self._last_warning = message

    def on_images(self, color_msg: Image, depth_msg: Image) -> None:
        if self.k is None:
            self.warn('no camera_info yet')
            return
        bgr = self.bridge.imgmsg_to_cv2(color_msg, 'bgr8')
        depth = self.bridge.imgmsg_to_cv2(depth_msg).astype(np.float32)
        if depth_msg.encoding == '16UC1':
            depth = depth / 1000.0  # Real drivers publish millimetres.
        mask = segment(bgr, self.hsv_lower, self.hsv_upper)
        found = locate(mask, depth, self.min_area)
        if self.debug_pub is not None:
            self.publish_debug(bgr, mask, found, color_msg)
        if found is None:
            self.warn('no block in view')
            return
        blob = largest_blob(mask, self.min_area)
        assert blob is not None  # locate() found it.
        points = blob_points(blob, depth, self.k)
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame, color_msg.header.frame_id,
                Time.from_msg(color_msg.header.stamp), timeout=Duration(seconds=0.2))
        except TransformException as error:
            self.warn(f'no transform to {self.target_frame}: {error}')
            return
        matrix = transform_matrix(transform)
        result = top_face(points @ matrix[:3, :3].T + matrix[:3, 3], self.cube_size)
        if result is None:
            self.warn('too few points on the block top')
            return
        centre, yaw = result
        pose = PoseStamped()
        pose.header.stamp = color_msg.header.stamp
        pose.header.frame_id = self.target_frame
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = (
            float(c) for c in centre)
        pose.pose.orientation.z = math.sin(yaw / 2)
        pose.pose.orientation.w = math.cos(yaw / 2)
        self.pose_pub.publish(pose)
        self._last_warning = ''

    def publish_debug(self, bgr: np.ndarray, mask: np.ndarray,
                      found: tuple[float, float, float, float] | None,
                      color_msg: Image) -> None:
        assert self.debug_pub is not None
        image = bgr.copy()
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(image, contours, -1, (0, 255, 0), 1)
        if found is not None:
            u, v, d, _ = found
            cv2.drawMarker(image, (int(round(u)), int(round(v))), (255, 0, 0),
                           cv2.MARKER_CROSS, 12, 1)
            cv2.putText(image, f'{d:.3f} m', (int(u) + 8, int(v) - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 1)
        msg = self.bridge.cv2_to_imgmsg(image, 'bgr8')
        msg.header = color_msg.header
        self.debug_pub.publish(msg)


def main(argv: list[str] | None = None) -> int:
    rclpy.init(args=argv)
    node = BlockDetector()
    try:
        rclpy.spin(node, executor=MultiThreadedExecutor())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
