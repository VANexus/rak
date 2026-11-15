"""Run the road-segmentation TensorRT engine and publish ROS perception outputs.

Inference is TensorRT only (JetPack ships it; torch is not a deployable dependency
on this board). The engine is built once by scripts/build_roadseg_engine.py from an
ONNX export of either TwinLiteNet+ (default) or LFD-RoadSeg.

The camera may arrive as a ROS Image or through the existing Astra shared-memory
feeder. The output mask is always available for RViz. Depth cloud outputs are
opt-in and require an explicit image/depth alignment mode plus a calibrated TF
from the depth camera to base_footprint.
"""

from __future__ import annotations

import copy
import math
import struct
import time
from pathlib import Path

import cv2
import numpy as np

import rclpy
from cv_bridge import CvBridge
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener

from .roadseg_trt import TrtSegmenter


_MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
_STD = np.array([0.229, 0.224, 0.225], dtype=np.float32)


class RoadSegNode(Node):
    """TorchScript inference node with a conservative optional Nav2 cloud path."""

    def __init__(self) -> None:
        super().__init__("roadseg_node")
        self._declare_parameters()
        self._warned_at: dict[str, float] = {}
        self._bridge = CvBridge()
        self._last_probability: np.ndarray | None = None
        self._last_mask_wall_time = 0.0
        self._last_sequence: int | None = None
        self._frames = 0
        self._fps_started = time.monotonic()

        self._input_mode = str(self.get_parameter("input_mode").value)
        if self._input_mode not in {"ros_image", "shared_memory"}:
            raise ValueError("input_mode must be 'ros_image' or 'shared_memory'")

        self._threshold = float(self.get_parameter("road_threshold").value)
        self._height = int(self.get_parameter("model_height").value)
        self._width = int(self.get_parameter("model_width").value)
        if not (0.0 < self._threshold < 1.0 and self._height > 0 and self._width > 0):
            raise ValueError("road_threshold must be in (0, 1), model dimensions must be positive")

        engine_path = Path(str(self.get_parameter("engine_path").value)).expanduser()
        if not engine_path.is_file():
            raise FileNotFoundError(
                f"TensorRT engine does not exist: {engine_path}. "
                "Build it once with scripts/build_roadseg_engine.py."
            )
        self._segmenter = TrtSegmenter(
            str(engine_path), (self._width, self._height),
            output_name=str(self.get_parameter("engine_output_name").value) or None,
        )
        self._normalize = str(self.get_parameter("normalize").value)
        if self._normalize not in {"unit", "imagenet"}:
            raise ValueError("normalize must be 'unit' (TwinLiteNet+) or 'imagenet' (LFD-RoadSeg)")
        self.get_logger().info(
            f"RoadSeg ready: {engine_path}, input={self._width}x{self._height}, "
            f"normalize={self._normalize}, threshold={self._threshold:.2f}"
        )

        self._mask_pub = self.create_publisher(
            Image, "/road_perception/road_mask", qos_profile_sensor_data
        )
        self._overlay_pub = self.create_publisher(
            Image, "/road_perception/overlay", qos_profile_sensor_data
        )

        self._image_sub = None
        self._shm_timer = None
        if self._input_mode == "ros_image":
            image_topic = str(self.get_parameter("image_topic").value)
            self._image_sub = self.create_subscription(
                Image, image_topic, self._image_callback, qos_profile_sensor_data
            )
            self.get_logger().info(f"RoadSeg image input: {image_topic}")
        else:
            poll_ms = max(5.0, float(self.get_parameter("shared_memory_poll_ms").value))
            self._shm_timer = self.create_timer(poll_ms / 1000.0, self._poll_shared_memory)
            self.get_logger().info(
                "RoadSeg shared-memory input: "
                f"{self.get_parameter('shared_memory_frame').value}"
            )

        self._publish_nav_clouds = bool(self.get_parameter("publish_nav_clouds").value)
        self._depth_alignment = str(self.get_parameter("depth_alignment").value)
        self._obstacle_pub = None
        self._clear_pub = None
        self._pointcloud_sub = None
        self._tf_buffer = None
        self._tf_listener = None
        if self._publish_nav_clouds:
            if self._depth_alignment not in {"registered", "astra_flip_shift"}:
                self._warn(
                    "alignment",
                    "Nav2 cloud output is enabled but depth_alignment is 'none'; "
                    "no semantic clouds will be published.",
                    interval_s=60.0,
                )
            self._obstacle_pub = self.create_publisher(
                PointCloud2, "/road_perception/nonroad_obstacles", qos_profile_sensor_data
            )
            self._clear_pub = self.create_publisher(
                PointCloud2, "/road_perception/road_clear", qos_profile_sensor_data
            )
            self._pointcloud_sub = self.create_subscription(
                PointCloud2,
                str(self.get_parameter("pointcloud_topic").value),
                self._pointcloud_callback,
                qos_profile_sensor_data,
            )
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self)
            self.get_logger().warning(
                "Nav2 cloud output is enabled. It will publish only when image/depth "
                "alignment is configured and TF to base_footprint is available."
            )

    def _declare_parameters(self) -> None:
        defaults = {
            "engine_path": "/home/xrak/rak/data/models/roadseg/twinlite_plus_small_fp16_384x672.plan",
            "engine_output_name": "",
            "normalize": "unit",
            "input_mode": "shared_memory",
            "image_topic": "/camera/color/image_raw",
            "image_frame_id": "camera_color_frame",
            "shared_memory_frame": "/dev/shm/cam_frame.bin",
            "shared_memory_sequence": "/dev/shm/cam_seq",
            "shared_memory_poll_ms": 10.0,
            "model_width": 672,
            "model_height": 384,
            "road_threshold": 0.40,
            "rotate_180": False,
            "adaptive_gamma": True,
            "low_light_threshold": 110.0,
            "low_light_gamma": 0.60,
            "publish_nav_clouds": False,
            "pointcloud_topic": "/camera/depth/points",
            "depth_alignment": "none",
            "depth_to_color_shift_px": -8,
            "base_frame": "base_footprint",
            "max_mask_age_s": 0.25,
            "point_stride": 4,
            "max_cloud_points": 3000,
            "max_range_m": 2.5,
            "max_lateral_m": 1.5,
            "obstacle_probability_max": 0.20,
            "road_clear_probability_min": 0.65,
            "obstacle_height_min_m": 0.08,
            "obstacle_height_max_m": 1.80,
            "road_height_min_m": -0.10,
            "road_height_max_m": 0.25,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _warn(self, key: str, message: str, interval_s: float = 5.0) -> None:
        now = time.monotonic()
        if now - self._warned_at.get(key, 0.0) >= interval_s:
            self.get_logger().warning(message)
            self._warned_at[key] = now

    def _poll_shared_memory(self) -> None:
        frame_path = Path(str(self.get_parameter("shared_memory_frame").value))
        seq_path = Path(str(self.get_parameter("shared_memory_sequence").value))
        try:
            sequence = int(seq_path.read_text(encoding="ascii").strip())
            if sequence == self._last_sequence:
                return
            raw = frame_path.read_bytes()
            if len(raw) < 16:
                return
            magic, width, height, size = struct.unpack_from("<4sIII", raw, 0)
            if magic != b"CAM1" or size != width * height * 3 or len(raw) < 16 + size:
                self._warn("shm_header", "Shared-memory camera frame header is invalid")
                return
            rgb = np.frombuffer(raw, dtype=np.uint8, count=size, offset=16).reshape(
                height, width, 3
            )
            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = str(self.get_parameter("image_frame_id").value)
            self._last_sequence = sequence
            self._infer_and_publish(bgr, header)
        except (OSError, ValueError, struct.error) as exc:
            self._warn("shm_read", f"Shared-memory frame unavailable: {exc}")

    def _image_callback(self, message: Image) -> None:
        try:
            bgr = self._bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            header = copy.copy(message.header)
            if not header.frame_id:
                header.frame_id = str(self.get_parameter("image_frame_id").value)
            self._infer_and_publish(bgr, header)
        except Exception as exc:
            self._warn("image_decode", f"Could not decode input image: {exc}")

    def _infer_and_publish(self, bgr: np.ndarray, header: Header) -> None:
        if bgr is None or bgr.ndim != 3 or bgr.shape[2] != 3:
            self._warn("bad_image", "RoadSeg received an invalid color image")
            return
        if bool(self.get_parameter("adaptive_gamma").value):
            luminance = float(cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY).mean())
            gamma = float(self.get_parameter("low_light_gamma").value)
            if gamma <= 0.0:
                self._warn("bad_gamma", "low_light_gamma must be positive; skipping frame")
                return
            if luminance < float(self.get_parameter("low_light_threshold").value):
                lut = (np.arange(256, dtype=np.float32) / 255.0) ** gamma * 255.0
                bgr = cv2.LUT(bgr, lut.astype(np.uint8))
        if bool(self.get_parameter("rotate_180").value):
            # The camera is mounted upside down and there is no V4L2 flip control; only
            # this consumer needs an upright image ("road at the bottom"), so the turn
            # happens here instead of in a separate flip node that would drop camera_info.
            bgr = cv2.rotate(bgr, cv2.ROTATE_180)
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        rgb = cv2.resize(rgb, (self._width, self._height), interpolation=cv2.INTER_LINEAR)
        normalized = rgb.astype(np.float32) / 255.0
        if self._normalize == "imagenet":
            normalized = (normalized - _MEAN) / _STD
        chw = np.ascontiguousarray(normalized.transpose(2, 0, 1)[None, ...])
        try:
            probability = self._segmenter.class_probability(chw)
        except Exception as exc:
            self._warn("inference", f"RoadSeg inference failed: {exc}")
            return

        image_h, image_w = bgr.shape[:2]
        probability = cv2.resize(
            probability, (image_w, image_h), interpolation=cv2.INTER_LINEAR
        ).astype(np.float32, copy=False)
        mask = (probability > self._threshold).astype(np.uint8) * 255
        self._last_probability = probability
        self._last_mask_wall_time = time.monotonic()

        mask_message = self._bridge.cv2_to_imgmsg(mask, encoding="mono8")
        mask_message.header = copy.copy(header)
        self._mask_pub.publish(mask_message)

        if self._overlay_pub.get_subscription_count() > 0:
            overlay = bgr.copy()
            selected = mask != 0
            tint = np.array([0, 0, 255], dtype=np.float32)
            overlay[selected] = (
                overlay[selected].astype(np.float32) * 0.55 + tint * 0.45
            ).astype(np.uint8)
            overlay_message = self._bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
            overlay_message.header = copy.copy(header)
            self._overlay_pub.publish(overlay_message)

        self._frames += 1
        elapsed = time.monotonic() - self._fps_started
        if elapsed >= 5.0:
            coverage = float(mask.mean()) / 255.0 * 100.0
            self.get_logger().info(
                f"RoadSeg {self._frames / elapsed:.1f} FPS, road={coverage:.1f}%"
            )
            self._frames = 0
            self._fps_started = time.monotonic()

    @staticmethod
    def _organized_xyz(message: PointCloud2) -> np.ndarray | None:
        """Read organized float32 XYZ fields while honoring row padding."""
        if message.width <= 0 or message.height <= 1 or message.point_step <= 0:
            return None
        fields = {field.name: field for field in message.fields}
        if any(name not in fields for name in ("x", "y", "z")):
            return None
        xyz_fields = [fields[name] for name in ("x", "y", "z")]
        if any(field.datatype != PointField.FLOAT32 for field in xyz_fields):
            return None
        endian = ">" if message.is_bigendian else "<"
        dtype = np.dtype({
            "names": ["x", "y", "z"],
            "formats": [endian + "f4"] * 3,
            "offsets": [field.offset for field in xyz_fields],
            "itemsize": message.point_step,
        })
        try:
            view = np.ndarray(
                shape=(message.height, message.width), dtype=dtype, buffer=message.data,
                strides=(message.row_step, message.point_step),
            )
        except (TypeError, ValueError):
            return None
        return np.stack((view["x"], view["y"], view["z"]), axis=-1).astype(
            np.float32, copy=False
        )

    @staticmethod
    def _rotation_matrix(rotation) -> np.ndarray:
        x, y, z, w = rotation.x, rotation.y, rotation.z, rotation.w
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if norm < 1e-9:
            raise ValueError("invalid zero-norm TF quaternion")
        x, y, z, w = x / norm, y / norm, z / norm, w / norm
        return np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ], dtype=np.float32)

    def _pointcloud_callback(self, message: PointCloud2) -> None:
        if self._depth_alignment not in {"registered", "astra_flip_shift"}:
            return
        if self._last_probability is None:
            return
        max_age = float(self.get_parameter("max_mask_age_s").value)
        if time.monotonic() - self._last_mask_wall_time > max_age:
            self._warn("stale_mask", "Depth cloud has no fresh RoadSeg mask; skipping Nav2 points")
            return
        if self._tf_buffer is None or not message.header.frame_id:
            return

        xyz = self._organized_xyz(message)
        if xyz is None:
            self._warn(
                "unorganized_cloud",
                "Depth cloud must be organized and contain float32 x/y/z fields",
            )
            return

        image_h, image_w = self._last_probability.shape
        cloud_h, cloud_w = xyz.shape[:2]
        stride = max(1, int(self.get_parameter("point_stride").value))
        vv, uu = np.mgrid[0:cloud_h:stride, 0:cloud_w:stride]
        points_camera = xyz[::stride, ::stride].reshape(-1, 3)
        if self._depth_alignment == "astra_flip_shift":
            color_u = (uu.astype(np.float32) + 0.5) * image_w / cloud_w - 0.5
            color_v = (cloud_h - 1 - vv.astype(np.float32) + 0.5) * image_h / cloud_h - 0.5
            color_u += float(self.get_parameter("depth_to_color_shift_px").value)
        else:
            color_u = (uu.astype(np.float32) + 0.5) * image_w / cloud_w - 0.5
            color_v = (vv.astype(np.float32) + 0.5) * image_h / cloud_h - 0.5

        inside = (
            (color_u >= 0) & (color_u < image_w) &
            (color_v >= 0) & (color_v < image_h)
        ).reshape(-1)
        if not np.any(inside):
            return
        color_u = np.clip(np.rint(color_u).astype(np.int32), 0, image_w - 1).reshape(-1)
        color_v = np.clip(np.rint(color_v).astype(np.int32), 0, image_h - 1).reshape(-1)
        probability = self._last_probability[color_v, color_u]
        valid_points = np.isfinite(points_camera).all(axis=1) & (points_camera[:, 2] > 0.05)
        valid_points &= inside

        try:
            stamp = Time.from_msg(message.header.stamp)
            transform = self._tf_buffer.lookup_transform(
                str(self.get_parameter("base_frame").value),
                message.header.frame_id,
                stamp,
                timeout=Duration(seconds=0.0),
            )
            rotation = self._rotation_matrix(transform.transform.rotation)
            translation = np.array([
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            ], dtype=np.float32)
        except (TransformException, ValueError) as exc:
            self._warn(
                "camera_tf",
                f"No calibrated TF for the depth cloud; Nav2 output stays off: {exc}",
            )
            return

        points_base = points_camera @ rotation.T + translation
        max_range = float(self.get_parameter("max_range_m").value)
        max_lateral = float(self.get_parameter("max_lateral_m").value)
        xy_range = np.hypot(points_base[:, 0], points_base[:, 1])
        in_local_roi = (
            (points_base[:, 0] > 0.05) & (xy_range <= max_range) &
            (np.abs(points_base[:, 1]) <= max_lateral)
        )

        obstacle_keep = valid_points & in_local_roi
        obstacle_keep &= probability <= float(
            self.get_parameter("obstacle_probability_max").value
        )
        obstacle_keep &= points_base[:, 2] >= float(
            self.get_parameter("obstacle_height_min_m").value
        )
        obstacle_keep &= points_base[:, 2] <= float(
            self.get_parameter("obstacle_height_max_m").value
        )

        clear_keep = valid_points & in_local_roi
        clear_keep &= probability >= float(
            self.get_parameter("road_clear_probability_min").value
        )
        clear_keep &= points_base[:, 2] >= float(self.get_parameter("road_height_min_m").value)
        clear_keep &= points_base[:, 2] <= float(self.get_parameter("road_height_max_m").value)

        header = copy.copy(message.header)
        max_points = max(1, int(self.get_parameter("max_cloud_points").value))
        obstacles = self._limit_points(points_camera[obstacle_keep], max_points)
        clear = self._limit_points(points_camera[clear_keep], max_points)
        if obstacles.shape[0] and self._obstacle_pub is not None:
            self._obstacle_pub.publish(
                point_cloud2.create_cloud_xyz32(header, obstacles.tolist())
            )
        if clear.shape[0] and self._clear_pub is not None:
            self._clear_pub.publish(point_cloud2.create_cloud_xyz32(header, clear.tolist()))

    @staticmethod
    def _limit_points(points: np.ndarray, limit: int) -> np.ndarray:
        if points.shape[0] <= limit:
            return points
        indices = np.linspace(0, points.shape[0] - 1, num=limit, dtype=np.int64)
        return points[indices]


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = RoadSegNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
