#!/usr/bin/env python3
"""Synthetic lidar + depth cloud for the 4D-fusion gates (test fixture, no hardware).

    python3 scripts/fake_4d_sensors.py --seconds 30 --range 1.5

Publishes what the real drivers publish, in the same frames:
  /scan                 sensor_msgs/LaserScan   frame laser_link, planar, --range metres
  /camera/depth/points  sensor_msgs/PointCloud2 frame laser_link, 200 points at --range
  /map                  nav_msgs/OccupancyGrid  only with --map (all-free latched map,
                        0.05 m cells, 20x20 m) so a planner has a global frame to work in

Why a separate process instead of publishing from the verifier: on this box a rclpy
publisher/subscriber created inside the same process as the verifier's other endpoint
never matched the Nav2 costmap (frames=0 while `ros2 topic echo` received them,
2026-11-15). A plain subprocess behaves like a real driver and is what the gates use.
"""

from __future__ import annotations

import argparse
import struct
import time

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import LaserScan, PointCloud2, PointField


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, default=30.0)
    parser.add_argument("--range", type=float, default=1.5)
    parser.add_argument("--rate", type=float, default=15.0)
    parser.add_argument("--frame", default="laser_link")
    parser.add_argument("--map", action="store_true",
                        help="also publish an all-free /map (for the planner gate)")
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("fake_4d_sensors")
    scan_pub = node.create_publisher(LaserScan, "/scan", 10)
    cloud_pub = node.create_publisher(PointCloud2, "/camera/depth/points", 10)

    map_pub = None
    if args.map:
        map_pub = node.create_publisher(
            OccupancyGrid, "/map",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        grid = OccupancyGrid()
        grid.header.frame_id = "map"
        grid.info.resolution = 0.05
        grid.info.width = grid.info.height = 400        # 20 m x 20 m, origin (-10,-10)
        grid.info.origin.position.x = -10.0
        grid.info.origin.position.y = -10.0
        grid.info.origin.orientation.w = 1.0
        grid.data = [0] * (grid.info.width * grid.info.height)
        map_pub.publish(grid)

    period = 1.0 / args.rate
    deadline = time.time() + args.seconds
    while time.time() < deadline and rclpy.ok():
        stamp = node.get_clock().now().to_msg()

        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = args.frame
        scan.angle_min, scan.angle_max = -1.0, 1.0
        scan.angle_increment = 0.05
        scan.range_min, scan.range_max = 0.05, 10.0
        scan.ranges = [args.range] * 41
        scan_pub.publish(scan)

        cloud = PointCloud2()
        cloud.header.stamp = stamp
        cloud.header.frame_id = args.frame
        cloud.height, cloud.width = 1, 200
        cloud.fields = [
            PointField(name=name, offset=4 * i, datatype=PointField.FLOAT32, count=1)
            for i, name in enumerate(("x", "y", "z"))
        ]
        cloud.point_step, cloud.row_step = 12, 12 * 200
        cloud.is_dense = True
        cloud.data = b"".join(
            struct.pack("<fff", args.range * 0.95, 0.05 * (i % 40) - 1.0, 0.3)
            for i in range(200)
        )
        cloud_pub.publish(cloud)

        time.sleep(period)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
