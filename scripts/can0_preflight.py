#!/usr/bin/env python3
"""can0 真机预检（G0 门）：链路在不在、板子在不在，一屏说清。

无 ROS 依赖，stdlib only。只读，不发任何 CAN 帧（单主站规则）。
判据全部来自 /sys/class/net/can0 + 2 s 被动抓帧计数：
  - interface 缺失 / down        -> 提示 can0-up.service（含 Orin pinmux）
  - ERROR-WARNING/PASSIVE 且 rx=0 -> TX 无应答 = 板子没上线（供电/接线/终端电阻）
  - rx>0                          -> 总线有帧，可进 G1 bringup

用法: python3 scripts/can0_preflight.py [--iface can0] [--window 2.0]
退出码: 0 = 板在总线上；1 = 链路/板子缺席；2 = 参数/权限问题。
"""
import argparse
import socket
import sys
import time

SYSFS = "/sys/class/net/{iface}/statistics/{name}"


def read_stat(iface, name):
    try:
        with open(SYSFS.format(iface=iface, name=name)) as fh:
            return int(fh.read().strip())
    except OSError:
        return None


def link_dump(iface):
    import subprocess
    r = subprocess.run(["ip", "-details", "link", "show", iface],
                       capture_output=True, text=True)
    return r.stdout if r.returncode == 0 else ""


def link_state(dump):
    # "state UP" / "state ERROR-PASSIVE" 出现在 can state 行；两处都找
    import re
    m = re.search(r"state (\S+)", dump)
    state = m.group(1) if m else "?"
    berr = re.search(r"berr-counter tx (\d+) rx (\d+)", dump)
    return state, ("bitrate" in dump), (
        (int(berr.group(1)), int(berr.group(2))) if berr else None)


def count_frames(iface, window):
    """被动收帧计数。socket 绑定失败返回 None（无权限/接口不在）。"""
    try:
        s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        s.bind((iface,))
    except (OSError, PermissionError) as e:
        print(f"[!!] 抓帧失败（{e}）；rx 计数改用 sysfs 增量")
        return None
    frames = 0
    deadline = time.monotonic() + window
    while time.monotonic() < deadline:
        s.settimeout(max(0.05, deadline - time.monotonic()))
        try:
            frame, _ = s.recvfrom(16)
        except socket.timeout:
            break
        frames += 1
    s.close()
    return frames


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--iface", default="can0")
    ap.add_argument("--window", type=float, default=2.0)
    args = ap.parse_args()
    iface = args.iface

    ok = True
    dump = link_dump(iface)
    if not dump:
        print(f"[G0 FAIL] {iface} 不存在。修复：安装并启用 can0-up.service"
              f"（含 Orin AON pinmux，漏写时 can0 照常 UP 但一帧都收不到）。")
        return 1
    state, has_bitrate, berr = link_state(dump)
    up = "UP" in dump.split("\n")[0]
    print(f"[G0] {iface}: state={state} up={up} bitrate配置={'有' if has_bitrate else '缺'}"
          + (f" berr(tx,rx)=({berr[0]},{berr[1]})" if berr else ""))
    if not up:
        print("[G0 FAIL] 接口未 UP。修复：sudo systemctl start can0-up.service")
        return 1
    if not has_bitrate or "1000000" not in dump:
        print("[G0 WARN] 未确认 1 Mbps——下位机按 1 Mbps 配置，不匹配会整条哑掉。")
        ok = False

    rx0, tx0, terr0 = (read_stat(iface, n) for n in ("rx_packets", "tx_packets", "tx_errors"))
    frames = count_frames(iface, args.window)
    if frames is None:
        rx1 = read_stat(iface, "rx_packets")
        frames = (rx1 - rx0) if (rx1 is not None and rx0 is not None) else 0
    tx1, terr1 = read_stat(iface, "tx_packets"), read_stat(iface, "tx_errors")
    dt = max(args.window, 0.1)
    print(f"[G0] {args.window:.1f}s 被动收帧 = {frames}"
          f"；tx_errors {terr0} -> {terr1}"
          f"（Δ{None if terr1 is None or terr0 is None else terr1 - terr0}）")

    if frames and frames > 0:
        print("[G0 PASS] 总线有入帧：板子在说话，进 G1（chassis_bringup）。")
        return 0
    dtx = None if (terr0 is None or terr1 is None) else terr1 - terr0
    if dtx is not None and dtx > 0:
        print("[G0 FAIL] 零入帧 + TX 错误持续增长 = 我发帧无人 ACK："
              "底盘板没上线。排查顺序：板供电 -> CAN_H/CAN_L 接线 -> 终端电阻"
              " -> 板上固件是否起来（串口/boot）。")
    elif berr and berr[0] > 0 and berr[1] == 0:
        print("[G0 FAIL] berr tx=%d rx=0：历史 TX 从未被 ACK（板子不在总线上）；"
              "本轮无新流量只是没有发送者。排查：供电/接线/终端电阻，"
              "再跑 G1 bringup 复测。" % berr[0])
    else:
        print("[G0 FAIL] 零入帧且无 TX 错误 = 没有任何流量：等待期间无发送者。"
              "先跑一次 bringup（它会发 NMT），或检查 pinmux/driver。")
    ok = False
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
