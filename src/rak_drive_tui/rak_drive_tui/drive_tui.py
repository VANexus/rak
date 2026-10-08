"""Mecanum driving TUI for the Rak chassis over the USB v2 link.

[AI] Host-side mecanum IK (ADR-0007：IK 在上位机) → DRIVE_COMMAND
(4 轮 urad/s，FL/FR/RL/RR)，20 Hz lease 保活；面板显示 DRIVE_STATE
（每轮实测速度/位置、故障、epoch）与 DIAGNOSTICS（电池/温度/错误计数），
并用麦轮正运动学对轮位增量做航位推算显示 x/y/θ（无 IMU/雷达融合，
麦轮 y 向漂移是固有代价，仅供人工驾驶参考）。

几何（与固件 R30 标定表 / rak_chassis_bringup controllers.yaml 同源，
⚠️ 杠杆 150 mm 仍标"未实测签核"）：
    WHEEL_RADIUS = 0.0325 m, LX_PLUS_LY = 0.15 m
IK 符号矩阵与 scripts/mecanum_tui_v2.py（实驱联测验证过）一致：
    fl = (vx - vy - L·wz)/r   fr = (vx + vy + L·wz)/r
    rl = (vx + vy - L·wz)/r   rr = (vx - vy + L·wz)/r

安全：
- 默认 DISARMED；ARM 需显式按键；退出先发 DISARMED 零目标
- 固件侧仍有完整安全门：lease 看门狗、外部故障（IMU/标定/电池）、急停
- 速度上限保守（0.30 m/s / 1.20 rad/s，占位未签核），不要放大
"""

from __future__ import annotations

import math
import os
import threading
import time

import curses

import rclpy
from rclpy.node import Node

from rak_base_driver.protocol import (
    MODE_ARMED,
    MODE_DISARMED,
    ProtocolError,
)
from rak_base_driver.protocol_v2 import (
    LEASE_MAX_MS,
    LEASE_MIN_MS,
    REQUIRED_FEATURES,
    TYPE_DRIVE_COMMAND,
    TYPE_DRIVE_STATE,
    TYPE_HELLO,
    TYPE_HELLO_ACK,
    TYPE_DIAGNOSTICS,
    TYPE_FAULT_EVENT,
    VERSION,
    V2StreamDecoder,
    DriveCommand,
    encode_v2_frame,
    pack_drive_command,
    pack_hello,
    unpack_drive_state,
    unpack_diagnostics,
    unpack_hello_ack,
)

WHEEL_RADIUS = 0.0325     # m（固件 R30 标定表 / controllers.yaml 同源）
LX_PLUS_LY = 0.15         # m（半轮距+半轴距之和，未实测签核）
MAX_LIN = 0.30            # m/s
MAX_ANG = 1.20            # rad/s
STEP_LIN = 0.05           # m/s 每按
STEP_ANG = 0.25           # rad/s 每按
STEP_SCALE_MAX = 4        # PgUp 放大倍数上限

WHEELS = ("front_left", "front_right", "rear_left", "rear_right")


def mecanum_ik(vx: float, vy: float, wz: float) -> tuple[float, ...]:
    """机体速度 → 四轮角速度 rad/s，顺序 FL/FR/RL/RR（标准 X 构型）。"""
    a = LX_PLUS_LY * wz
    r = WHEEL_RADIUS
    fl = (vx - vy - a) / r
    fr = (vx + vy + a) / r
    rl = (vx + vy - a) / r
    rr = (vx - vy + a) / r
    return fl, fr, rl, rr


def mecanum_fk(w: tuple[float, float, float, float]) -> tuple[float, float, float]:
    """四轮角速度 → 机体 (vx, vy, wz)。IK 的逆。"""
    w1, w2, w3, w4 = w
    r = WHEEL_RADIUS
    vx = r * (w1 + w2 + w3 + w4) / 4.0
    vy = r * (-w1 + w2 + w3 - w4) / 4.0
    wz = r * (-w1 + w2 - w3 + w4) / (4.0 * LX_PLUS_LY)
    return vx, vy, wz


def fault_bits(bitmap: int) -> str:
    if bitmap == 0:
        return "none"
    return ",".join(f"bit{b}" for b in range(32) if bitmap & (1 << b))


class DriveSession:
    """Serial worker: HELLO 握手 + 20 Hz DRIVE_COMMAND + 上报解析/里程积分。"""

    def __init__(self, port: str, baud: int, lease_ms: int, rate_hz: float):
        self._port = port
        self._baud = baud
        self._lease_ms = lease_ms
        self._period = 1.0 / rate_hz
        self._lock = threading.Lock()
        self._vx = 0.0
        self._vy = 0.0
        self._wz = 0.0
        self._armed = False
        self._quit = False
        self._sequence = 0
        self._status = "offline: not connected"
        self._handshake_ok = False
        self._drive_mode = -1
        self._fault_bitmap = 0
        self._feedback_mask = 0
        self._epoch = -1
        self._echoed_sequence = -1
        self._wheel_vel_urad = (0, 0, 0, 0)     # 实测
        self._wheel_pos_urad = (0, 0, 0, 0)
        self._last_pos_urad = None              # 里程积分用的上一拍轮位
        self._last_pos_time = None
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_th = 0.0
        self._battery_mv = 0
        self._temp_centi = 0
        self._link_errors = (0, 0, 0, 0)        # crc/seq/rx_overrun/tx_drop
        self._thread = threading.Thread(target=self._run, daemon=True)

    # -- TUI 线程接口 --------------------------------------------------------
    def start(self) -> None:
        self._thread.start()

    def request_stop(self) -> None:
        with self._lock:
            self._quit = True

    def set_twist(self, vx: float, vy: float, wz: float) -> None:
        with self._lock:
            self._vx = max(-MAX_LIN, min(MAX_LIN, vx))
            self._vy = max(-MAX_LIN, min(MAX_LIN, vy))
            self._wz = max(-MAX_ANG, min(MAX_ANG, wz))

    def twist(self) -> tuple[float, float, float]:
        with self._lock:
            return self._vx, self._vy, self._wz

    def set_armed(self, armed: bool) -> None:
        with self._lock:
            self._armed = armed

    def armed(self) -> bool:
        with self._lock:
            return self._armed

    def status(self) -> str:
        with self._lock:
            return self._status

    def set_status(self, text: str) -> None:
        with self._lock:
            self._status = text

    def feedback(self) -> dict:
        with self._lock:
            return {
                "mode": self._drive_mode,
                "fault": self._fault_bitmap,
                "vmask": self._feedback_mask,
                "epoch": self._epoch,
                "echo": self._echoed_sequence,
                "wheel_vel": self._wheel_vel_urad,
                "wheel_pos": self._wheel_pos_urad,
                "battery_mv": self._battery_mv,
                "temp_centi": self._temp_centi,
                "errors": self._link_errors,
                "odom": (self._odom_x, self._odom_y, self._odom_th),
            }

    def reset_odom(self) -> None:
        with self._lock:
            self._odom_x = self._odom_y = self._odom_th = 0.0
            self._last_pos_urad = None

    # -- serial worker -------------------------------------------------------
    def _run(self) -> None:
        import serial  # pyserial

        while True:
            with self._lock:
                if self._quit:
                    return
            try:
                self._session_once(serial)
            except (OSError, serial.SerialException, ProtocolError) as exc:
                self.set_status(f"offline: {exc}")
                with self._lock:
                    self._handshake_ok = False
                    self._echoed_sequence = -1
                    self._last_pos_urad = None
            time.sleep(1.0)

    def _next_sequence(self) -> int:
        self._sequence = (self._sequence + 1) & 0xFFFF
        return self._sequence

    def _send_drive(self, ser, mode: int,
                    wheel_urad: tuple[int, int, int, int]) -> None:
        command = DriveCommand(
            mode=mode,
            lease_ms=self._lease_ms,
            valid_mask=0x0F if mode == MODE_ARMED else 0,
            wheel_velocity_urad_s=wheel_urad,
        )
        ser.write(encode_v2_frame(
            TYPE_DRIVE_COMMAND, self._next_sequence(),
            pack_drive_command(command)))

    def _session_once(self, serial_module) -> None:
        with serial_module.Serial(self._port, self._baud, timeout=0.05) as ser:
            decoder = V2StreamDecoder()
            self.set_status("handshake: HELLO sent")
            payload = pack_hello(0x52414B31, REQUIRED_FEATURES)  # "RAK1"
            ser.write(encode_v2_frame(TYPE_HELLO, self._next_sequence(), payload))

            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline:
                with self._lock:
                    if self._quit:
                        return
                self._drain_rx(ser, decoder)
                with self._lock:
                    ok = self._handshake_ok
                if ok:
                    break
                time.sleep(0.02)
            else:
                raise ProtocolError("no HELLO_ACK within 2 s")

            self.set_status("link up")
            next_tx = time.monotonic()
            while True:
                with self._lock:
                    quit_flag = self._quit
                    armed = self._armed
                    vx, vy, wz = self._vx, self._vy, self._wz
                if quit_flag:
                    self._send_drive(ser, MODE_DISARMED, (0, 0, 0, 0))
                    return
                now = time.monotonic()
                if now >= next_tx:
                    if armed:
                        wheels = tuple(
                            int(round(w * 1e6)) for w in mecanum_ik(vx, vy, wz))
                        self._send_drive(ser, MODE_ARMED, wheels)
                    else:
                        self._send_drive(ser, MODE_DISARMED, (0, 0, 0, 0))
                    next_tx = now + self._period
                self._drain_rx(ser, decoder)
                time.sleep(0.005)

    def _drain_rx(self, ser, decoder: V2StreamDecoder) -> None:
        chunk = ser.read(256)
        if not chunk:
            return
        for frame in decoder.feed(chunk):
            self._handle_frame(frame)

    def _handle_frame(self, frame) -> None:
        if frame.message_type == TYPE_HELLO_ACK:
            board_id, fw, _boot, accepted = unpack_hello_ack(frame.payload)
            missing = REQUIRED_FEATURES & ~accepted
            if missing:
                self.set_status(f"board 0x{board_id:04X}: missing features 0x{missing:X}")
            else:
                with self._lock:
                    self._handshake_ok = True
                self.set_status(f"board 0x{board_id:04X} fw 0x{fw:08X} ready")
        elif frame.message_type == TYPE_DRIVE_STATE:
            state = unpack_drive_state(frame.payload)
            now = time.monotonic()
            with self._lock:
                self._drive_mode = state.mode
                self._fault_bitmap = state.fault_bitmap
                self._feedback_mask = state.feedback_valid_mask
                self._epoch = state.encoder_epoch
                self._echoed_sequence = state.echoed_command_sequence
                self._wheel_vel_urad = tuple(state.wheel_velocity_urad_s)
                self._wheel_pos_urad = tuple(state.wheel_position_urad)
            self._integrate_odom(tuple(state.wheel_position_urad), now)
        elif frame.message_type == TYPE_DIAGNOSTICS:
            diag = unpack_diagnostics(frame.payload)
            with self._lock:
                self._battery_mv = diag.battery_mv
                self._temp_centi = diag.board_temp_centi_c
                self._link_errors = (
                    diag.crc_error_count, diag.sequence_error_count,
                    diag.rx_overrun_count, diag.tx_drop_count)
        elif frame.message_type == TYPE_FAULT_EVENT:
            pass  # fault_bitmap 已随 DRIVE_STATE 周期性携带，事件不重复累计

    def _integrate_odom(self, pos_urad: tuple[int, ...], now: float) -> None:
        """轮位增量 → 麦轮正运动学 → 位姿积分（航位推算，仅供人工参考）。"""
        with self._lock:
            last = self._last_pos_urad
            last_t = self._last_pos_time
            self._last_pos_urad = pos_urad
            self._last_pos_time = now
            if last is None or last_t is None or now <= last_t:
                return
            dt = now - last_t
            d = [float(pos_urad[i] - last[i]) for i in range(4)]
            if self._epoch >= 0:
                pass  # epoch 变化（重标定）时不做特殊处理，误差一拍内自愈
            w = tuple(v * 1e-6 for v in d)  # 每轮角位移 rad
            vx, vy, wz = mecanum_fk(w)
            vx, vy = vx / dt, vy / dt
            wz = wz / dt
            th = self._odom_th
            self._odom_x += (vx * math.cos(th) - vy * math.sin(th)) * dt
            self._odom_y += (vx * math.sin(th) + vy * math.cos(th)) * dt
            self._odom_th += wz * dt


def run_tui(stdscr, session: DriveSession, lease_ms: int) -> None:
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.keypad(True)

    step_scale = 1.0

    while True:
        key = stdscr.getch()
        if key in (ord("x"), ord("X"), 27):
            session.set_armed(False)
            session.request_stop()
            return
        vx, vy, wz = session.twist()
        sl = STEP_LIN * step_scale
        sa = STEP_ANG * step_scale
        if key == curses.KEY_UP:
            session.set_twist(vx + sl, vy, wz)
        elif key == curses.KEY_DOWN:
            session.set_twist(vx - sl, vy, wz)
        elif key == curses.KEY_LEFT:
            session.set_twist(vx, vy + sl, wz)
        elif key == curses.KEY_RIGHT:
            session.set_twist(vx, vy - sl, wz)
        elif key == ord("q"):
            session.set_twist(vx, vy, wz + sa)
        elif key == ord("e"):
            session.set_twist(vx, vy, wz - sa)
        elif key == ord(" "):
            session.set_twist(0.0, 0.0, 0.0)
        elif key == curses.KEY_PPAGE:
            step_scale = min(float(STEP_SCALE_MAX), step_scale * 2.0)
        elif key == curses.KEY_NPAGE:
            step_scale = max(0.25, step_scale / 2.0)
        elif key in (ord("a"), ord("A")):
            session.set_armed(not session.armed())
        elif key in (ord("r"), ord("R")):
            session.reset_odom()

        fb = session.feedback()
        ox, oy, oth = fb["odom"]
        vx, vy, wz = session.twist()

        stdscr.erase()
        armed = session.armed()
        stdscr.addstr(0, 0, "RAK drive TUI  (USB v2 DRIVE_COMMAND, lease %d ms, keepalive 20 Hz)" % lease_ms)
        stdscr.addstr(1, 0, f"link: {session.status()}   board mode={fb['mode']} echo_seq={fb['echo']}")
        stdscr.addstr(2, 0, f"faults={fault_bits(fb['fault'])}  vmask=0x{fb['vmask']:X}  epoch={fb['epoch']}")
        stdscr.addstr(3, 0, f"battery={fb['battery_mv']} mV   temp={fb['temp_centi']/100.0:.1f} C   "
                           f"err(crc/seq/ovr/tx)={fb['errors']}")
        stdscr.addstr(4, 0, "-" * 78)
        stdscr.addstr(5, 0, f"target: vx={vx:+.2f} m/s  vy={vy:+.2f} m/s  wz={wz:+.2f} rad/s   "
                           f"step x{step_scale:g}   {'ARMED' if armed else 'disarm'}")
        stdscr.addstr(6, 0, f"odom : x={ox:+.3f} m  y={oy:+.3f} m  th={math.degrees(oth):+.1f} deg   "
                           "(wheel dead-reckoning, no fusion)")
        stdscr.addstr(7, 0, "-" * 78)
        for i, name in enumerate(WHEELS):
            wv = fb["wheel_vel"][i] * 1e-6
            wp = fb["wheel_pos"][i] * 1e-6
            stdscr.addstr(8 + i, 0,
                          f"{name:<18} vel={wv:+7.2f} rad/s  pos={wp:+10.3f} rad")
        bottom = 8 + len(WHEELS) + 1
        stdscr.addstr(bottom, 0,
                      "keys: arrows move | q/e rotate | space stop | PgUp/PgDn step | a arm | r reset odom | x quit")
        stdscr.addstr(bottom + 1, 0,
                      "caution: wheels need clearance; estop = long-press board button or power")
        stdscr.refresh()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Node("drive_tui")
    node.declare_parameter("port", "")
    node.declare_parameter("baud", 921600)
    node.declare_parameter("lease_ms", 100)
    node.declare_parameter("rate_hz", 20.0)

    port = str(node.get_parameter("port").value)
    baud = int(node.get_parameter("baud").value)
    lease_ms = int(node.get_parameter("lease_ms").value)
    rate_hz = float(node.get_parameter("rate_hz").value)

    if not port:
        node.get_logger().error(
            "parameter 'port' is required, e.g. "
            "'ros2 run rak_drive_tui drive_tui --ros-args -p port:=/dev/ttyACM0'")
        node.destroy_node()
        rclpy.shutdown()
        return
    if not LEASE_MIN_MS <= lease_ms <= LEASE_MAX_MS:
        node.get_logger().error(
            f"lease_ms must be in [{LEASE_MIN_MS}, {LEASE_MAX_MS}]")
        node.destroy_node()
        rclpy.shutdown()
        return

    session = DriveSession(port, baud, lease_ms, rate_hz)

    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()
    session.start()
    try:
        curses.wrapper(run_tui, session, lease_ms)
    finally:
        session.set_armed(False)
        session.request_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
