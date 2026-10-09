"""CLI for rak_serial_link -- open the serial port, send/listen interactively.

Usage:
  serial_link_cli --port /dev/wheel_motor --baud 921600
  serial_link_cli send 0x100                    # build & send an empty CHASSIS_VEL_CMD
  serial_link_cli send 0x010 EMERGENCY_STOP     # send SAFETY_CMD by name
  serial_link_cli listen --seconds 5           # print decoded frames for 5 s

This is the "pure communication" debugging tool -- no ROS, no /cmd_vel, just
the wire.
"""
from __future__ import annotations

import argparse
import sys
import time

from serial import Serial, SerialException

from .protocol import (
    FrameStreamDecoder,
    SAFETY_CMD_NAMES,
    build_chassis_vel_cmd,
    build_safety_cmd,
)


def _open(port: str, baud: int) -> Serial:
    try:
        return Serial(port, baud, timeout=0.1)
    except (SerialException, OSError) as exc:
        print(f"failed to open {port}: {exc}", file=sys.stderr)
        sys.exit(1)


def cmd_send(args) -> int:
    ser = _open(args.port, args.baud)
    try:
        if args.safety:
            code = {v: k for k, v in SAFETY_CMD_NAMES.items()}[args.safety.upper()]
            frame = build_safety_cmd(code, counter=args.counter)
        elif args.can_id == 0x100:
            frame = build_chassis_vel_cmd(
                vx_mm_s=args.vx, vy_mm_s=args.vy, wz_mrad_s=args.wz,
                counter=args.counter, enable=True,
            )
        else:
            print(f"no built-in builder for 0x{args.can_id:03x}; pick 0x100 or use --safety", file=sys.stderr)
            return 1
        ser.write(frame)
        print(f"sent {len(frame)} bytes: {frame.hex().upper()}")
        return 0
    finally:
        ser.close()


def cmd_listen(args) -> int:
    ser = _open(args.port, args.baud)
    dec = FrameStreamDecoder()
    end = time.monotonic() + args.seconds
    try:
        while time.monotonic() < end:
            data = ser.read(64)
            if not data:
                continue
            for f in dec.feed(data):
                print(
                    f"id=0x{f.can_id:03x} name={f.name:20s} cnt={f.counter} "
                    f"flags=0x{f.flags:x} raw={f.raw.hex().upper()}"
                )
        print(f"---\ndecoded {dec.frames} frames, {dec.errors} sync errors")
        return 0
    finally:
        ser.close()


def main() -> int:
    p = argparse.ArgumentParser(prog="serial_link_cli")
    p.add_argument("--port", default="/dev/wheel_motor")
    p.add_argument("--baud", type=int, default=921600)
    sub = p.add_subparsers(dest="cmd", required=True)

    s = sub.add_parser("send", help="send one frame")
    s.add_argument("can_id", type=lambda x: int(x, 16))
    s.add_argument("--counter", type=int, default=0)
    s.add_argument("--safety", help="SAFETY_CMD name (overrides can_id)")
    s.add_argument("--vx", type=int, default=0)
    s.add_argument("--vy", type=int, default=0)
    s.add_argument("--wz", type=int, default=0)
    s.set_defaults(func=cmd_send)

    l = sub.add_parser("listen", help="print decoded frames for N seconds")
    l.add_argument("--seconds", type=float, default=5.0)
    l.set_defaults(func=cmd_listen)

    args = p.parse_args()
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())