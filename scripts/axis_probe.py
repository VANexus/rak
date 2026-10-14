#!/usr/bin/env python3
"""[AI] Time-series probe of the mecanum axis mapping.

Commands one axis at a time and prints a dense time series of DRIVE_CMD (0x100,
slots [FL, FR, RL, RR] as packed by can_session.cpp) so the steady state is
unambiguous — a single end-of-run sample proved unreliable.

Usage: python3 axis_probe.py [channel]
"""
import statistics
import struct
import sys
import time

import socket

ID_DRIVECMD = 0x100
ID_WHEELSPD = 0x202
CAN_FRAME_FMT = "=IB3x8s"


def dec(data, n=4):
    return [int.from_bytes(data[i * 2:i * 2 + 2], "little", signed=True) * 0.01
            for i in range(n)]


def main() -> int:
    channel = sys.argv[1] if len(sys.argv) > 1 else "can0"
    sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    sock.bind((channel,))
    sock.settimeout(0.02)

    buf = {"cmd": [0.0] * 4, "spd": [0.0] * 4}

    def drain(duration):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            try:
                cid, dlc, data = struct.unpack(CAN_FRAME_FMT, sock.recv(16))
            except (socket.timeout, BlockingIOError):
                continue
            cid &= 0x1FFFFFFF
            if cid == ID_DRIVECMD and dlc >= 8:
                buf["cmd"] = dec(data)
            elif cid == ID_WHEELSPD and dlc >= 8:
                buf["spd"] = dec(data)

    import rclpy
    from geometry_msgs.msg import TwistStamped
    from rclpy.node import Node

    rclpy.init()
    node = Node("axis_probe")
    pub = node.create_publisher(TwistStamped, "/mecanum_drive_controller/reference", 10)
    msg = TwistStamped()
    msg.header.frame_id = "base_link"

    def sample(vx, vy, wz, seconds):
        """Publish continuously, collect DRIVE_CMD samples after 0.8s settle."""
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z = vx, vy, wz
        end = time.monotonic() + seconds
        settle = time.monotonic() + 0.8
        rows = []
        while time.monotonic() < end and rclpy.ok():
            msg.header.stamp = node.get_clock().now().to_msg()
            pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.02)
            drain(0.06)
            if time.monotonic() >= settle:
                rows.append((list(buf["cmd"]), list(buf["spd"])))
        return rows

    def stop():
        msg.twist.linear.x = msg.twist.linear.y = msg.twist.angular.z = 0.0
        for _ in range(10):
            msg.header.stamp = node.get_clock().now().to_msg()
            pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.03)
        drain(0.4)

    def report(label, rows):
        if not rows:
            print(f"  {label}: NO SAMPLES")
            return
        cmds = [r[0] for r in rows]
        med = [statistics.median(c[i] for c in cmds) for i in range(4)]
        spd = [statistics.median(r[1][i] for r in rows) for i in range(4)]
        pattern = "".join("+" if v > 0.05 else "-" if v < -0.05 else "0" for v in med)
        sp = "".join("+" if v > 0.05 else "-" if v < -0.05 else "0" for v in spd)
        f = lambda v: " ".join(f"{x:+.2f}" for x in v)
        print(f"  {label:16s} n={len(rows):3d}  DRIVE_CMD {f(med)}  [{pattern}]")
        print(f"  {'':16s}          WHEEL_SPD {f(spd)}  [{sp}]")

    print(f"== axis probe, channel={channel} (steady-state medians) ==")
    print("   DRIVE_CMD slots [FL, FR, RL, RR]; '-'=reverse '+'=forward\n")

    print("  -- vx = +0.15 (expect ALL SAME SIGN) --")
    report("vx+0.15", sample(0.15, 0.0, 0.0, 3.0)); stop()

    print("  -- vy = +0.15 (expect FL- FR+ RL+ RR-) --")
    report("vy+0.15", sample(0.0, 0.15, 0.0, 3.0)); stop()

    print("  -- wz = +0.60 (expect FL- FR+ RL- RR+) --")
    report("wz+0.60", sample(0.0, 0.0, 0.60, 3.0)); stop()

    print("  -- baseline (no command) --")
    report("idle", sample(0.0, 0.0, 0.0, 2.0)); stop()

    node.destroy_node()
    rclpy.shutdown()
    sock.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
