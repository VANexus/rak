"""Live mask + trace diagnostics: ASCII render and start/end centroids."""
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from rak_road_perception.road_follow_math import (
    FOLLOW_DEFAULTS,
    TRACE_DEFAULTS,
    trace_centerline,
)


def main() -> None:
    params = {**TRACE_DEFAULTS, **FOLLOW_DEFAULTS}
    rclpy.init()
    node = Node("trace_diag")
    got = {}
    node.create_subscription(
        Image, "/road_perception/road_mask", lambda m: got.update(m=m), 10)
    t0 = time.time()
    while time.time() - t0 < 10 and "m" not in got:
        rclpy.spin_once(node, timeout_sec=0.3)
    message = got.get("m")
    if message is None:
        print("no mask frame in 10 s")
        rclpy.shutdown()
        return
    h, w = message.height, message.width
    mask = np.frombuffer(bytes(message.data), np.uint8).reshape(h, w)
    pts, spin, has = trace_centerline(mask, params)
    cols, rows = 64, 20
    art = [["#" if mask[int(r * h / rows):int((r + 1) * h / rows),
                        int(c * w / cols):int((c + 1) * w / cols)].mean() > 127
            else "." for c in range(cols)] for r in range(rows)]
    for y, xn in pts:
        art[min(rows - 1, int(y * rows / h))][min(cols - 1, int((xn + 0.5) * cols))] = "O"
    print("\n".join("".join(line) for line in art))
    print(f"road_frac={mask.mean() / 255:.3f} has={has} spin={spin} n={len(pts)}")
    if pts:
        print(f"bottom={pts[0][1]:+.3f} top={pts[-1][1]:+.3f}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()