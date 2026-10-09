#!/usr/bin/env python3
"""[AI] 麦轮运动 TUI 测试工具 —— USB UART v2 轮级直通。

用途: 键盘 W/A/S/D + Q/E 调节机体速度, 主机侧做麦轮逆运动学,
以 v2 DRIVE_COMMAND(前左/前右/后左/后右, rad/s)发给 F407,
用于观察四轮麦轮运动能力(前后/左右/自转)。

安全:
- 默认 DISARMED: 只握手+收 DRIVE_STATE, 不发非零目标。
- 必须 --arm 并在确认提示输入大写 ARM 才允许发 ARMED 命令。
- 20 Hz 保活(lease 100 ms), 退出/Ctrl-C 一律先发 DISARMED 零目标。
- ⚠️ 轮半径/轮距是占位值, 未实测签核; 速度上限保守, 不要放大。

键位: w/s 前后±  a/d 左右±  q/e 自转±  空格=全零  +/- 加减步长  x 退出。
运行: python3 scripts/mecanum_tui_v2.py [--device /dev/ttyACM1] [--arm]
"""
import argparse
import math
import os
import select
import sys
import termios
import threading
import time
import tty

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src", "rak_base_driver"))

from rak_base_driver import protocol as framing          # COBS/CRC 复用
from rak_base_driver import protocol_v2 as v2

LEASE_MS = 100          # v2 推荐 100 ms
KEEPALIVE_HZ = 20
STEP_LIN = 0.05         # m/s 每按一次的步长
STEP_ANG = 0.25         # rad/s
MAX_LIN = 0.30          # m/s 上限(占位, 未签核)
MAX_ANG = 1.20          # rad/s 上限
WHEEL_RADIUS = 0.05     # ⚠️ 占位: 未实测
LX_PLUS_LY = 0.30       # ⚠️ 占位: 半轮距+半轴距之和, 未实测

MODE_NAMES = {0: "BOOTING", 1: "DISARMED", 2: "ARMED", 3: "FAULT"}


def mecanum_ik(vx: float, vy: float, wz: float) -> tuple[float, float, float, float]:
    """机体速度 → 四轮线速度 m/s, 顺序 FL/FR/RL/RR(标准 X 构型)。"""
    a = LX_PLUS_LY * wz
    fl = (vx - vy - a) / WHEEL_RADIUS
    fr = (vx + vy + a) / WHEEL_RADIUS
    rl = (vx + vy - a) / WHEEL_RADIUS
    rr = (vx - vy + a) / WHEEL_RADIUS
    return fl, fr, rl, rr


def clamp(v: float, lim: float) -> float:
    return max(-lim, min(lim, v))


def find_device() -> str:
    base = "/dev/serial/by-id"
    if os.path.isdir(base):
        for name in sorted(os.listdir(base)):
            if "WCH" in name:
                return os.path.join(base, name)
    for cand in ("/dev/ttyACM0", "/dev/ttyACM1"):
        if os.path.exists(cand):
            return cand
    raise SystemExit("未找到 WCH 串口, 用 --device 指定")


class Link:
    def __init__(self, port: str):
        import serial
        self.ser = serial.Serial(port, 921600, timeout=0.05)
        self.lock = threading.Lock()
        self.seq = 1
        self.hello_ack = None
        self.drive_state = None
        self.diag = None
        self.faults = 0
        self.crc_errors = 0
        self.stop = False
        self.thread = threading.Thread(target=self._rx, daemon=True)
        self.thread.start()

    def _next_seq(self) -> int:
        with self.lock:
            seq = self.seq
            self.seq = seq + 1 if seq < 0x7FFF else 1
        return seq

    def send(self, message_type: int, payload: bytes) -> None:
        self.ser.write(v2.encode_v2_frame(message_type, self._next_seq(), payload))

    def _rx(self) -> None:
        buf = bytearray()
        while not self.stop:
            data = self.ser.read(256)
            if not data:
                continue
            buf.extend(data)
            while True:
                delim = buf.find(b"\x00")
                if delim < 0:
                    if len(buf) > 512:        # 防御: 无定界的垃圾流,丢弃重来
                        buf.clear()
                    break
                chunk, buf = bytes(buf[:delim]), bytearray(buf[delim + 1:])
                if not chunk:
                    continue
                try:
                    packet = v2.decode_v2_frame(chunk)
                except Exception:
                    self.crc_errors += 1
                    continue
                try:
                    if packet.message_type == v2.TYPE_HELLO_ACK:
                        self.hello_ack = v2.unpack_hello_ack(packet.payload)
                    elif packet.message_type == v2.TYPE_DRIVE_STATE:
                        self.drive_state = v2.unpack_drive_state(packet.payload)
                    elif packet.message_type == v2.TYPE_DIAGNOSTICS:
                        self.diag = v2.unpack_diagnostics(packet.payload)
                    elif packet.message_type == v2.TYPE_FAULT_EVENT:
                        self.faults = v2.unpack_fault_event(packet.payload)[0]
                except Exception:
                    self.crc_errors += 1

    def close(self) -> None:
        self.stop = True
        self.ser.close()


def draw(link: Link, vx: float, vy: float, wz: float, armed: bool) -> None:
    wheels = mecanum_ik(vx, vy, wz)
    ds = link.drive_state
    lines = [
        "=== 麦轮 TUI (USB v2 轮级直通) ===",
        f"机体目标: vx={vx:+.2f} m/s  vy={vy:+.2f} m/s  wz={wz:+.2f} rad/s   "
        f"[{'ARMED' if armed else 'DISARMED'}]",
        "四轮目标 rad/s: FL=%+.3f  FR=%+.3f  RL=%+.3f  RR=%+.3f" % wheels,
    ]
    if ds is not None:
        lines.append(
            f"DRIVE_STATE: mode={MODE_NAMES.get(ds.mode, ds.mode)}"
            f" valid={ds.feedback_valid_mask:#04x} epoch={ds.encoder_epoch}"
            f" fault={ds.fault_bitmap:#x}")
        lines.append(
            "实测轮速 rad/s: " + "  ".join(f"{v2.micro_to_si(x):+.3f}"
                                           for x in ds.wheel_velocity_urad_s))
        lines.append(
            "实测轮位 urad:  " + "  ".join(f"{x:>12d}" for x in ds.wheel_position_urad))
    else:
        lines.append("DRIVE_STATE: 等待遥测…")
    if link.hello_ack is not None:
        board, fw, boot, feats = link.hello_ack
        lines.append(f"HELLO_ACK: board={board:#06x} fw={fw >> 16}.{(fw >> 8) & 0xFF}"
                     f".{fw & 0xFF} boot={boot:#x} features={feats:#x}")
    if link.diag is not None:
        d = link.diag
        lines.append(f"DIAG: bat={d.battery_mv}mV crc_err={d.crc_error_count}"
                     f" seq_err={d.sequence_error_count} overrun={d.rx_overrun_count}"
                     f" txdrop={d.tx_drop_count}")
    lines += [
        f"fault_event={link.faults:#x}  本地crc_err={link.crc_errors}",
        "键位: w/s 前后  a/d 左右  q/e 自转  空格全零  +,- 步长  x 退出"
        f"  (步长 {STEP_LIN:.2f} m/s / {STEP_ANG:.2f} rad/s)",
    ]
    sys.stdout.write("\x1b[2J\x1b[H" + "\n".join(lines) + "\n")
    sys.stdout.flush()


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--device", default=None)
    ap.add_argument("--arm", action="store_true",
                    help="允许发 ARMED 轮速命令(默认只 DISARMED 观测)")
    args = ap.parse_args()

    armed = False
    if args.arm:
        print("⚠️  将以 v2 ARMED 模式驱动四轮。轮几何是占位值, 速度上限"
              f" {MAX_LIN} m/s / {MAX_ANG} rad/s。确认请输入大写 ARM: ", end="")
        sys.stdout.flush()
        if sys.stdin.readline().strip() != "ARM":
            print("未确认, 以 DISARMED 模式继续。")
        else:
            armed = True

    port = args.device or find_device()
    print(f"打开 {port} …")
    link = Link(port)
    link.send(v2.TYPE_HELLO, v2.pack_hello(0x52414B, v2.REQUIRED_FEATURES))

    vx = vy = wz = 0.0
    step_scale = 1.0
    old_term = termios.tcgetattr(0)
    try:
        tty.setcbreak(0)
        last_keepalive = 0.0
        while True:
            now = time.monotonic()
            if now - last_keepalive >= 1.0 / KEEPALIVE_HZ:
                last_keepalive = now
                wheels = mecanum_ik(vx, vy, wz) if armed else (0.0, 0.0, 0.0, 0.0)
                mode = v2.MODE_ARMED if armed else v2.MODE_DISARMED
                cmd = v2.DriveCommand(
                    mode=mode, lease_ms=LEASE_MS, valid_mask=0x0F,
                    wheel_velocity_urad_s=tuple(v2.si_to_micro(w, "wheel")
                                                for w in wheels))
                link.send(v2.TYPE_DRIVE_COMMAND, v2.pack_drive_command(cmd))

            r, _, _ = select.select([sys.stdin], [], [], 0.05)
            if r:
                ch = sys.stdin.read(1)
                lin = STEP_LIN * step_scale
                ang = STEP_ANG * step_scale
                if ch == "w":
                    vx = clamp(vx + lin, MAX_LIN)
                elif ch == "s":
                    vx = clamp(vx - lin, MAX_LIN)
                elif ch == "d":
                    vy = clamp(vy + lin, MAX_LIN)
                elif ch == "a":
                    vy = clamp(vy - lin, MAX_LIN)
                elif ch == "e":
                    wz = clamp(wz + ang, MAX_ANG)
                elif ch == "q":
                    wz = clamp(wz - ang, MAX_ANG)
                elif ch == " ":
                    vx = vy = wz = 0.0
                elif ch == "+":
                    step_scale = min(4.0, step_scale * 2)
                elif ch in "-_":
                    step_scale = max(0.25, step_scale / 2)
                elif ch in "xX\x03":
                    break

            if armed and math.hypot(vx, vy) == 0.0 and wz == 0.0:
                pass  # 零目标也按 20Hz 发送, 保持 lease 且保持 DISARMED→ARMED 状态一致
            draw(link, vx, vy, wz, armed)
    finally:
        termios.tcsetattr(0, termios.TCSADRAIN, old_term)
        try:
            zero = v2.DriveCommand(mode=v2.MODE_DISARMED, lease_ms=LEASE_MS,
                                   valid_mask=0x0F,
                                   wheel_velocity_urad_s=(0, 0, 0, 0))
            link.send(v2.TYPE_DRIVE_COMMAND, v2.pack_drive_command(zero))
            time.sleep(0.1)
        except Exception:
            pass
        link.close()
        print("\n已发送 DISARMED 零目标并退出。")


if __name__ == "__main__":
    main()
