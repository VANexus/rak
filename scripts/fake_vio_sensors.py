#!/usr/bin/env python3
"""Synthetic RGB-D pair for the VIO gate (test fixture, no camera needed).

    python3 scripts/fake_vio_sensors.py --seconds 40

Publishes what `depth_camera.launch.py` + the colour stream would publish, in the
frames `vio.launch.py` remaps to:

  /camera/color/image_raw      sensor_msgs/Image        bgr8, synthetic texture
  /camera/color/camera_info    sensor_msgs/CameraInfo   320x240, fx=fy=250
  /camera/depth/image_raw      sensor_msgs/Image        16UC1, millimetres, 1.0 m
  /camera/depth/camera_info    sensor_msgs/CameraInfo   same intrinsics

The texture is deterministic pseudo-noise with a slow horizontal shift, so features
exist and the frame-to-frame motion is non-zero: rtabmap_odom needs both to emit an
odometry twist we can assert on.
"""

from __future__ import annotations

import argparse
import struct
import time

import rclpy
from sensor_msgs.msg import CameraInfo, Image

WIDTH, HEIGHT = 320, 240
FX = FY = 250.0


def _texture(shift: int) -> bytes:
    """bgr8 row-major pseudo-random blocks, shifted by `shift` pixels."""
    row = bytearray(WIDTH * 3)
    for x in range(WIDTH):
        seed = (x + shift) * 2654435761 & 0xFFFFFFFF
        v = (seed >> 13) & 0xFF
        row[3 * x] = v
        row[3 * x + 1] = (v * 7) & 0xFF
        row[3 * x + 2] = (v * 13) & 0xFF
    out = bytearray()
    for y in range(HEIGHT):
        seed = ((y * 40503) + (shift * 17)) & 0xFFFFFFFF
        if seed % 97 < 48:
            out += row
        else:                                    # local contrast break
            alt = bytearray(row[3:] + row[:3])
            out += alt
    return bytes(out)


def _camera_info(stamp, frame: str) -> CameraInfo:
    info = CameraInfo()
    info.header.stamp = stamp
    info.header.frame_id = frame
    info.width, info.height = WIDTH, HEIGHT
    info.distortion_model = "plumb_bob"
    info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
    info.k = [FX, 0.0, WIDTH / 2.0, 0.0, FY, HEIGHT / 2.0, 0.0, 0.0, 1.0]
    info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    info.p = [FX, 0.0, WIDTH / 2.0, 0.0, 0.0, FY, HEIGHT / 2.0, 0.0, 0.0, 0.0, 1.0, 0.0]
    return info


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, default=40.0)
    parser.add_argument("--rate", type=float, default=10.0)
    parser.add_argument("--frame", default="camera_color_optical_frame")
    parser.add_argument("--depth-frame", default="camera_depth_optical_frame")
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("fake_vio_sensors")
    color_pub = node.create_publisher(Image, "/camera/color/image_raw", 5)
    color_info_pub = node.create_publisher(CameraInfo, "/camera/color/camera_info", 5)
    depth_pub = node.create_publisher(Image, "/camera/depth/image_raw", 5)
    depth_info_pub = node.create_publisher(CameraInfo, "/camera/depth/camera_info", 5)

    depth_row = struct.pack("<" + "H" * WIDTH, *([1000] * WIDTH))
    depth_data = depth_row * HEIGHT
    period = 1.0 / args.rate
    deadline = time.time() + args.seconds
    shift = 0

    while time.time() < deadline and rclpy.ok():
        stamp = node.get_clock().now().to_msg()

        color = Image()
        color.header.stamp = stamp
        color.header.frame_id = args.frame
        color.height, color.width = HEIGHT, WIDTH
        color.encoding = "bgr8"
        color.step = WIDTH * 3
        color.data = _texture(shift)
        color_pub.publish(color)
        color_info_pub.publish(_camera_info(stamp, args.frame))

        depth = Image()
        depth.header.stamp = stamp
        depth.header.frame_id = args.depth_frame
        depth.height, depth.width = HEIGHT, WIDTH
        depth.encoding = "16UC1"
        depth.step = WIDTH * 2
        depth.data = depth_data
        depth_pub.publish(depth)
        depth_info_pub.publish(_camera_info(stamp, args.depth_frame))

        shift = (shift + 2) % WIDTH
        time.sleep(period)

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
