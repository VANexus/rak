#!/usr/bin/env python3
"""[AI] Read-only CAN v4 monitor for the chassis.

Binds a SocketCAN raw socket and reports frame rates plus the decoded DIAG
state. Sends nothing at all — unlike can_link_smoke.py, which ends with STOP
and would close the firmware's remote gate underneath a running stack.

Usage: python3 can_monitor.py [channel] [seconds]
"""
import struct
import sys
import time

import socket

ID_SAFETY = 0x010
ID_NODEINFO = 0x030
ID_DRIVECMD = 0x100
ID_WHEELPOS_A = 0x200
ID_WHEELPOS_B = 0x201
ID_WHEELSPD = 0x202
ID_DIAG = 0x220

NAMES = {
    ID_SAFETY: "SAFETY",
    ID_NODEINFO: "NODE_INFO",
    ID_DRIVECMD: "DRIVE_CMD(host tx)",
    ID_WHEELPOS_A: "WHEEL_POS_A",
    ID_WHEELPOS_B: "WHEEL_POS_B",
    ID_WHEELSPD: "WHEEL_SPD",
    ID_DIAG: "DIAG",
}

CAN_FRAME_FMT = "=IB3x8s"  # can_id, can_dlc, data (16 bytes)


def main() -> int:
    channel = sys.argv[1] if len(sys.argv) > 1 else "can0"
    seconds = float(sys.argv[2]) if len(sys.argv) > 2 else 3.0

    sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    sock.bind((channel,))
    sock.settimeout(0.05)

    counts = {}
    diag = None
    speeds = []   # decoded WHEEL_SPD samples (rad/s)
    cmds = []     # decoded DRIVE_CMD samples (rad/s)
    # Watchdog so a stale DIAG from before the window is not reported as live.
    diag_seen_at = None
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        try:
            cid, dlc, data = struct.unpack(CAN_FRAME_FMT, sock.recv(16))
        except (socket.timeout, BlockingIOError):
            continue
        cid &= 0x1FFFFFFF
        counts[cid] = counts.get(cid, 0) + 1
        if cid == ID_WHEELSPD and dlc >= 8:
            wheel = [int.from_bytes(data[i:i + 2], "little", signed=True) * 0.01
                     for i in (0, 2, 4, 6)]
            speeds.append(wheel)
        if cid == ID_DRIVECMD and dlc >= 8:
            cmd = [int.from_bytes(data[i:i + 2], "little", signed=True) * 0.01
                   for i in (0, 2, 4, 6)]
            cmds.append(cmd)
        if cid == ID_DIAG and dlc >= 7:
            diag = (
                data[0] | (data[1] << 8),      # bat_mv
                data[2] | (data[3] << 8),      # fault
                data[4] | (data[5] << 8),      # state
                data[6],                       # mode
                data[7] if dlc > 7 else 0,     # busoff_cnt
            )
            diag_seen_at = time.monotonic()

    sock.close()

    print(f"== read-only CAN monitor: {channel}, window {seconds:.1f}s ==")
    for cid in sorted(counts, key=lambda c: (c not in NAMES, c)):
        name = NAMES.get(cid, f"id=0x{cid:03X}")
        print(f"  {name:22s} 0x{cid:03X}  {counts[cid] / seconds:7.1f} Hz")

    missing = [n for c, n in NAMES.items() if c not in counts and c != ID_SAFETY]
    if missing:
        print(f"  MISSING: {', '.join(missing)}")

    def _vec(samples, label):
        if not samples:
            print(f"  {label}: (none)")
            return
        last = samples[-1]
        peak = max(max(abs(v) for v in s) for s in samples)
        print(f"  {label}: FL={last[0]:+.2f} FR={last[1]:+.2f} "
              f"RL={last[2]:+.2f} RR={last[3]:+.2f} rad/s  (peak |.| {peak:.2f})")

    _vec(cmds, "DRIVE_CMD(host)")
    _vec(speeds, "WHEEL_SPD(feedback)")

    if diag is None or diag_seen_at is None:
        print("  DIAG: (not seen in window)")
    else:
        bat, fault, state, mode, busoff = diag
        modes = {0: "IDLE", 1: "REMOTE", 2: "ESTOP", 3: "FAULT"}
        age_ms = (time.monotonic() - diag_seen_at) * 1000.0
        print(f"  DIAG: BAT={bat}mV FAULT=0x{fault:04X} STATE=0x{state:04X} "
              f"MODE={mode}({modes.get(mode, '?')}) BUSOFF={busoff} age={age_ms:.0f}ms")
        print(f"        remote_gate={'OPEN' if mode == 1 else 'CLOSED'} "
              f"estop_latched={'yes' if state & 0x0001 else 'no'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
