"""Bounded RGB-D capture only; never publishes commands or opens actuator ports."""
import argparse
from collections import deque
import json
from pathlib import Path
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener, TransformException

from .core import check_camera


def stamp_ns(msg):
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec


def camera_dict(msg):
    return {"width": msg.width, "height": msg.height, "k": list(msg.k),
            "d": list(msg.d), "distortion_model": msg.distortion_model,
            "frame_id": msg.header.frame_id}


def image_array(msg, color=False):
    channels, size = (3, 1) if color else (1, 2)
    if (msg.encoding not in (("rgb8", "bgr8") if color else ("16UC1",))
            or msg.step < msg.width * channels * size
            or len(msg.data) != msg.step * msg.height):
        raise ValueError("invalid image encoding or stride")
    shape = (msg.height, msg.width, 3) if color else (msg.height, msg.width)
    strides = (msg.step, 3, 1) if color else (msg.step, 2)
    dtype = np.uint8 if color else (">u2" if msg.is_bigendian else "<u2")
    data = np.ndarray(shape, dtype=dtype, buffer=bytes(msg.data), strides=strides).copy()
    if color and msg.encoding == "rgb8":
        data = cv2.cvtColor(data, cv2.COLOR_RGB2BGR)
    return data.astype(np.uint8 if color else np.uint16)


class Capture(Node):
    def __init__(self):
        super().__init__("grasp_snapshot_capture")
        self.images = {key: deque(maxlen=8) for key in ("color", "depth")}
        self.infos = {}
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        for name in self.images:
            self.create_subscription(Image, f"/camera/{name}/image_raw",
                lambda m, n=name: self.images[n].append(m), qos_profile_sensor_data)
            self.create_subscription(CameraInfo, f"/camera/{name}/camera_info",
                lambda m, n=name: self.infos.update({n: m}), qos_profile_sensor_data)

    def pair(self):
        if len(self.infos) < 2:
            return None
        pairs = [(abs(stamp_ns(c) - stamp_ns(d)), c, d)
                 for c in self.images["color"] for d in self.images["depth"]]
        if not pairs:
            return None
        skew, color, depth = min(pairs, key=lambda p: p[0])
        if skew > 20_000_000:
            return None
        if any(self.infos[n].header.frame_id != m.header.frame_id
               for n, m in (("color", color), ("depth", depth))):
            raise ValueError("CameraInfo optical frame mismatch")
        try:
            tf = self.buffer.lookup_transform(color.header.frame_id, depth.header.frame_id,
                                              Time.from_msg(depth.header.stamp))
        except TransformException:
            return None
        transform = np.eye(4)
        q, p = tf.transform.rotation, tf.transform.translation
        transform[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        transform[:3, 3] = [p.x, p.y, p.z]
        rgb, dep = image_array(color, True), image_array(depth)
        check_camera(camera_dict(self.infos["color"]), rgb.shape)
        check_camera(camera_dict(self.infos["depth"]), dep.shape)
        if np.count_nonzero(dep) == 0:
            return None
        meta = {"schema_version": 1, "depth_unit": "mm", "depth_scale_m": 0.001,
                "color_stamp_ns": stamp_ns(color), "depth_stamp_ns": stamp_ns(depth),
                "depth_frame": depth.header.frame_id, "color_frame": color.header.frame_id,
                "color_info": camera_dict(self.infos["color"]),
                "depth_info": camera_dict(self.infos["depth"]),
                "depth_to_color": transform.tolist(), "device": "Gemini 335L",
                "mount_calibrated": False}
        return meta, rgb, dep


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--seconds", type=float, default=30)
    cli, ros_args = parser.parse_known_args(args)
    if cli.output.exists():
        parser.error("output already exists; use a new snapshot directory")
    if not 0 < cli.seconds <= 300:
        parser.error("capture timeout must be within (0,300]")
    rclpy.init(args=ros_args)
    node, result = Capture(), None
    try:
        deadline = time.monotonic() + cli.seconds
        while time.monotonic() < deadline and result is None:
            rclpy.spin_once(node, timeout_sec=0.1)
            result = node.pair()
        if result is None:
            raise RuntimeError("no valid synchronized RGB-D pair with camera TF before timeout")
        meta, rgb, dep = result
        cli.output.mkdir(parents=True, exist_ok=False)
        if not cv2.imwrite(str(cli.output / "color.png"), rgb) or not cv2.imwrite(
                str(cli.output / "depth-mm.png"), dep):
            raise OSError("image save failed")
        (cli.output / "snapshot.json").write_text(json.dumps(meta, indent=2))
        print(f"SNAPSHOT_OK {cli.output} skew_ms="
              f"{abs(meta['color_stamp_ns']-meta['depth_stamp_ns'])/1e6:.3f}")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
