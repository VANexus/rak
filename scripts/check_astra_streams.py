"""Bounded, read-only ROS RGB/depth inspection; writes evidence without commanding motion."""

import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


class Inspector(Node):
    def __init__(self):
        super().__init__("astra_stream_inspector")
        self.counts = {name: 0 for name in ("color", "depth", "color_info", "depth_info")}
        self.images = {}
        self.infos = {}
        self.arrivals = {name: [] for name in ("color", "depth")}
        self.errors = []
        for name in ("color", "depth"):
            self.create_subscription(
                Image, f"/camera/{name}/image_raw",
                lambda msg, name=name: self.image(name, msg), qos_profile_sensor_data)
            self.create_subscription(
                CameraInfo, f"/camera/{name}/camera_info",
                lambda msg, name=name: self.info(name, msg), qos_profile_sensor_data)

    def image(self, name, msg):
        self.counts[name] += 1
        self.arrivals[name].append(time.monotonic())
        channels = 3 if name == "color" else 1
        itemsize = 1 if name == "color" else 2
        supported = ("rgb8", "bgr8") if name == "color" else ("16UC1",)
        if (msg.encoding not in supported or not msg.width or not msg.height
                or msg.step < msg.width * channels * itemsize
                or len(msg.data) != msg.step * msg.height):
            self.errors.append(f"{name}: invalid encoding, dimensions, stride or payload")
            return
        dtype = np.uint8 if name == "color" else (">u2" if msg.is_bigendian else "<u2")
        shape = (msg.height, msg.width, 3) if name == "color" else (msg.height, msg.width)
        strides = (msg.step, 3, 1) if name == "color" else (msg.step, 2)
        data = np.ndarray(shape, dtype=dtype, buffer=bytes(msg.data), strides=strides).copy()
        self.images[name] = (msg, data)

    def info(self, name, msg):
        self.counts[name + "_info"] += 1
        self.infos[name] = msg

    def result(self, elapsed, output):
        result = {"elapsed_seconds": elapsed, "counts": self.counts, "streams": {},
                  "errors": sorted(set(self.errors)), "depth_unit": "mm",
                  "depth_color_alignment_verified": False}
        for name, (msg, data) in self.images.items():
            arrivals = self.arrivals[name]
            observed_hz = ((len(arrivals) - 1) / (arrivals[-1] - arrivals[0])
                           if len(arrivals) > 1 else 0.0)
            info = self.infos.get(name)
            calibrated = bool(info and np.isfinite(info.k).all()
                              and info.k[0] > 0 and info.k[4] > 0
                              and info.width == msg.width and info.height == msg.height
                              and info.header.frame_id == msg.header.frame_id)
            details = {"width": msg.width, "height": msg.height, "encoding": msg.encoding,
                       "frame_id": msg.header.frame_id, "step": msg.step,
                       "observed_hz": observed_hz, "intrinsics_present": calibrated,
                       "stamp_seconds": msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9}
            if info:
                details["k"] = list(info.k)
                details["distortion_model"] = info.distortion_model
            if name == "depth":
                valid = data[data > 0]
                details.update(valid_pixels=int(valid.size),
                               valid_fraction=float(valid.size / data.size),
                               median_mm=float(np.median(valid)) if valid.size else None,
                               center_mm=int(data[msg.height // 2, msg.width // 2]))
                cv2.imwrite(str(output / "depth-mm.png"), data)
                preview = cv2.applyColorMap(
                    np.uint8(np.clip(data, 0, 4000) / 4000 * 255), cv2.COLORMAP_TURBO)
                preview[data == 0] = 0
            else:
                preview = cv2.cvtColor(data, cv2.COLOR_RGB2BGR) if msg.encoding == "rgb8" else data
            cv2.imwrite(str(output / (name + "-preview.png")), preview)
            result["streams"][name] = details
        result["rgbd_ok"] = (not self.errors and all(self.counts.values())
                             and len(self.images) == 2
                             and result["streams"]["depth"]["valid_pixels"] > 0)
        return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=12)
    parser.add_argument("--output", type=Path, default=Path("log/astra-inspection"))
    args = parser.parse_args()
    if not 0 < args.seconds <= 300:
        parser.error("--seconds must be within (0, 300]")
    args.output.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = Inspector()
    started = time.monotonic()
    try:
        while rclpy.ok() and time.monotonic() - started < args.seconds:
            rclpy.spin_once(node, timeout_sec=0.1)
        result = node.result(time.monotonic() - started, args.output)
        (args.output / "result.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
        print(json.dumps(result, indent=2), flush=True)
        return 0 if result["rgbd_ok"] else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
