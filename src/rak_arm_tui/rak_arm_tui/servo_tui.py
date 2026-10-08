"""Hand-teach TUI for the six Rak arm servos over the USB v2 link.

[AI] Commissioning tool for the 2026-10 arm bring-up. Talks the F407's
USB v2 protocol (reuses rak_base_driver codec): HELLO handshake, then a
periodic ARM_COMMAND (mode=1, 6-axis urad positions) at the configured
rate, and renders ARM_STATE feedback.

Axis semantics are fixed by the firmware (app_car.h R42):
axis[0..5] = J1..J6, position_urad 0..2,700,000 == 500..2500 us pulse.
J1/J6 are continuous-rotation servos: the midpoint (135 deg) is STOP and
offsets are speed, so the TUI shows them as "speed offset".
J2 is mechanically limited to 0..135 deg; J3/J4/J5 span 0..270 deg.

Safety:
- starts DISARMED at mid positions; arming requires an explicit keypress
- the F407 applies positions only while the lease is refreshed
- quitting sends a DISARM frame first
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
    TYPE_ARM_COMMAND,
    TYPE_ARM_STATE,
    TYPE_HELLO,
    TYPE_HELLO_ACK,
    VERSION,
    ArmCommand,
    encode_v2_frame,
    pack_arm_command,
    pack_hello,
    unpack_arm_state,
    unpack_hello_ack,
)

DEG_TO_URAD = math.pi / 180.0 * 1e6
FULL_SCALE_DEG = 270.0


class V2StreamDecoder:
    """[AI] rak_base_driver 的 StreamDecoder 固定校验 v1 版本号，v2 帧会被
    'unsupported protocol version' 静默丢弃；此处以 require_version=2 重实现
    同样的增量分隔符解码（字节级行为一致，仅版本门不同）。"""

    def __init__(self, max_encoded_size: int = 160):
        import rak_base_driver.protocol as _p

        self._decode = _p.decode_frame
        self._buffer = bytearray()
        self._max = max_encoded_size
        self.errors = 0

    def feed(self, data: bytes):
        frames = []
        for byte in data:
            if byte == 0:
                if self._buffer:
                    try:
                        frames.append(self._decode(bytes(self._buffer), require_version=VERSION))
                    except ProtocolError:
                        self.errors += 1
                    self._buffer.clear()
            else:
                self._buffer.append(byte)
                if len(self._buffer) > self._max:
                    self.errors += 1
                    self._buffer.clear()
        return frames

# [AI] 2026-10-08 TUI 实测标定（用户探极限后确认，第二轮更新）：
# 全部 6 轴为位置型 MG995/MG996R，0~270°（J2 机械约束 0~255°）。
# 零位（q_urdf=0，用户指定）= 自旋轴指令 0°、J2=135°、J3=180°、J5=0°。
# 连杆长度：1-2:8cm 2-3:10cm 3-4:11cm 4-5:6cm 5-6:9cm（properties_arm.xacro 同步）。
# J3 俯仰方向与 J2/J5 反向——已反映在 rak_arm.urdf.xacro 的轴定义里。
JOINTS = (
    # (name, board pin, kind, min_deg, max_deg, home_deg)
    ("J1 waist",       "PB15", "angle",   0.0, 270.0,   0.0),
    ("J2 shoulder",    "PB14", "angle",   0.0, 255.0, 135.0),
    ("J3 elbow",       "PC9",  "angle",   0.0, 270.0, 180.0),
    ("J4 forearm",     "PC8",  "angle",   0.0, 270.0,   0.0),
    ("J5 wrist_pitch", "PC7",  "angle",   0.0, 270.0,   0.0),
    ("J6 wrist_roll",  "PC6",  "angle",   0.0, 270.0,   0.0),
)


def deg_to_urad(deg: float) -> int:
    return int(round(deg * DEG_TO_URAD))


def urad_to_pulse_us(urad: int) -> int:
    # 0..2,700,000 urad -> 500..2500 us (firmware car_arm_command_urad)
    return 500 + int(round(urad / (FULL_SCALE_DEG * DEG_TO_URAD) * 2000))


class V2ArmSession:
    """Serial worker: HELLO handshake + periodic ARM_COMMAND + ARM_STATE RX."""

    def __init__(self, port: str, baud: int, lease_ms: int, rate_hz: float):
        self._port = port
        self._baud = baud
        self._lease_ms = lease_ms
        self._period = 1.0 / rate_hz
        self._lock = threading.Lock()
        self._targets_deg = [home for (_, _, _, _, _, home) in JOINTS]
        self._armed = False
        self._quit = False
        self._sequence = 0
        self._status = "offline: not connected"
        self._handshake_ok = False
        self._arm_mode = 0
        self._fault_bitmap = 0
        self._echoed_sequence = -1
        # [AI] 极限记录：实际发出过的每轴角度范围 + 人工标记
        self._sent_min_deg = [270.0] * 6
        self._sent_max_deg = [0.0] * 6
        self._marks = []
        self._thread = threading.Thread(target=self._run, daemon=True)

    # -- interface used by the TUI thread ----------------------------------
    def start(self) -> None:
        self._thread.start()

    def request_stop(self) -> None:
        with self._lock:
            self._quit = True

    def set_target(self, index: int, deg: float) -> None:
        with self._lock:
            self._targets_deg[index] = deg

    def target(self, index: int) -> float:
        with self._lock:
            return self._targets_deg[index]

    def set_armed(self, armed: bool) -> None:
        with self._lock:
            self._armed = armed

    def armed(self) -> bool:
        with self._lock:
            return self._armed

    def status(self) -> str:
        with self._lock:
            return self._status

    def feedback(self) -> tuple[int, int, int]:
        with self._lock:
            return self._arm_mode, self._fault_bitmap, self._echoed_sequence

    def sent_range(self) -> tuple[list[float], list[float]]:
        with self._lock:
            return list(self._sent_min_deg), list(self._sent_max_deg)

    def mark_pose(self, label: str) -> dict:
        """[AI] 人工标记当前目标位姿（用于现场指认真实机械极限）。"""
        with self._lock:
            entry = {
                "label": label,
                "wall_time": time.strftime("%H:%M:%S"),
                "targets_deg": list(self._targets_deg),
                "sent_min_deg": list(self._sent_min_deg),
                "sent_max_deg": list(self._sent_max_deg),
            }
            self._marks.append(entry)
            return entry

    def clear_record(self) -> None:
        with self._lock:
            self._sent_min_deg = [270.0] * 6
            self._sent_max_deg = [0.0] * 6
            self._marks = []

    def record(self) -> dict:
        with self._lock:
            return {
                "recorded_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
                "axis_order": [name for (name, _, _, _, _, _) in JOINTS],
                "note": (
                    "All six axes are positional MG995/MG996R servos "
                    "(2026-10-08 TUI measurement: J1/J3/J4/J5/J6 0-270 deg, "
                    "J2 0-255 deg, hardware constrained). sent_min/max are "
                    "commanded angles while ARMED; zero pose (straight arm) "
                    "= pitch 90 deg, roll 0 deg."
                ),
                "sent_min_deg": list(self._sent_min_deg),
                "sent_max_deg": list(self._sent_max_deg),
                "marks": list(self._marks),
            }

    # -- serial worker ------------------------------------------------------
    def _run(self) -> None:
        import serial  # pyserial; optional import kept local like node.py

        while True:
            with self._lock:
                if self._quit:
                    return
            try:
                self._session_once(serial)
            except (OSError, serial.SerialException, ProtocolError) as exc:
                self._set_status(f"offline: {exc}")
            time.sleep(1.0)

    def _set_status(self, text: str) -> None:
        with self._lock:
            self._status = text

    set_status_text = _set_status   # [AI] TUI 线程写入状态用别名

    def _session_once(self, serial_module) -> None:
        with serial_module.Serial(self._port, self._baud, timeout=0.05) as ser:
            decoder = V2StreamDecoder()
            self._set_status("handshake: HELLO sent")
            self._send_hello(ser)
            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline:
                with self._lock:
                    if self._quit:
                        return
                self._drain_rx(ser, decoder)
                with self._lock:
                    handshake_ok = self._handshake_ok
                if handshake_ok:
                    break
                time.sleep(0.02)
            else:
                raise ProtocolError("no HELLO_ACK within 2 s")

            self._set_status("link up")
            next_tx = time.monotonic()
            while True:
                with self._lock:
                    quit_flag = self._quit
                    armed = self._armed
                    targets = tuple(self._targets_deg)
                if quit_flag:
                    self._send_arm(ser, MODE_DISARMED, tuple([0] * 6))
                    return
                now = time.monotonic()
                if now >= next_tx:
                    mode = MODE_ARMED if armed else MODE_DISARMED
                    # [AI] 无关节级软限位（用户 2026-10-07：先测真实极限）。
                    # 唯一钳位在固件 500~2500µs（=0~270° PWM 物理量程）。
                    payload_deg = list(targets) if mode == MODE_ARMED else [0] * 6
                    if mode == MODE_ARMED:
                        with self._lock:
                            for i, d in enumerate(payload_deg):
                                self._sent_min_deg[i] = min(self._sent_min_deg[i], d)
                                self._sent_max_deg[i] = max(self._sent_max_deg[i], d)
                    self._send_arm(ser, mode, tuple(
                        deg_to_urad(d) for d in payload_deg))
                    next_tx = now + self._period
                self._drain_rx(ser, decoder)
                time.sleep(0.005)

    def _next_sequence(self) -> int:
        self._sequence = (self._sequence + 1) & 0xFFFF
        return self._sequence

    def _send_hello(self, ser) -> None:
        payload = pack_hello(0x52414B31, REQUIRED_FEATURES)  # "RAK1"
        ser.write(encode_v2_frame(TYPE_HELLO, self._next_sequence(), payload))

    def _send_arm(self, ser, mode: int, urad: tuple[int, ...]) -> None:
        command = ArmCommand(
            mode=mode,
            lease_ms=self._lease_ms,
            valid_mask=(1 << 6) - 1 if mode == MODE_ARMED else 0,
            joint_position_urad=urad,
        )
        ser.write(encode_v2_frame(
            TYPE_ARM_COMMAND, self._next_sequence(),
            pack_arm_command(command, 0)))

    def _drain_rx(self, ser, decoder: StreamDecoder) -> None:
        chunk = ser.read(256)
        if not chunk:
            return
        for frame in decoder.feed(chunk):
            self._handle_frame(frame)

    def _handle_frame(self, frame) -> None:
        if frame.message_type == TYPE_HELLO_ACK:
            board_id, fw, boot, accepted = unpack_hello_ack(frame.payload)
            missing = REQUIRED_FEATURES & ~accepted
            if missing:
                self._set_status(f"board 0x{board_id:04X}: missing features 0x{missing:X}")
            else:
                with self._lock:
                    self._handshake_ok = True
                self._set_status(f"board 0x{board_id:04X} fw 0x{fw:08X} ready")
        elif frame.message_type == TYPE_ARM_STATE:
            state = unpack_arm_state(frame.payload, 0)
            with self._lock:
                self._arm_mode = state.mode
                self._fault_bitmap = state.fault_bitmap
                self._echoed_sequence = state.echoed_command_sequence


def fault_bits(bitmap: int) -> str:
    if bitmap == 0:
        return "none"
    return ",".join(f"bit{b}" for b in range(32) if bitmap & (1 << b))


def run_tui(stdscr, session: V2ArmSession, lease_ms: int, record_path: str) -> None:
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.keypad(True)

    selected = 0
    step_small = 2.0
    step_big = 10.0

    while True:
        key = stdscr.getch()
        if key in (ord("q"), ord("Q"), 27):  # q / ESC
            session.set_armed(False)
            session.request_stop()
            return
        lo, hi = JOINTS[selected][3], JOINTS[selected][4]
        if key == curses.KEY_UP:
            selected = (selected - 1) % len(JOINTS)
        elif key == curses.KEY_DOWN:
            selected = (selected + 1) % len(JOINTS)
        elif key == curses.KEY_LEFT:
            # [AI] 实测限位钳制（2026-10-08 标定完成后恢复）
            session.set_target(selected, max(lo, session.target(selected) - step_small))
        elif key == curses.KEY_RIGHT:
            session.set_target(selected, min(hi, session.target(selected) + step_small))
        elif key == curses.KEY_PPAGE:
            session.set_target(selected, min(hi, session.target(selected) + step_big))
        elif key == curses.KEY_NPAGE:
            session.set_target(selected, max(lo, session.target(selected) - step_big))
        elif key == curses.KEY_HOME:
            session.set_target(selected, JOINTS[selected][5])  # 零位（全直）
        elif key in (ord("a"), ord("A")):
            session.set_armed(not session.armed())
        elif key in (ord("m"), ord("M")):
            session.mark_pose(f"mark{len(session.record()['marks']) + 1}")
        elif key in (ord("w"), ord("W")):
            write_record(session.record(), record_path)
            session.set_status_text(f"record written to {record_path}")
        elif key in (ord("c"), ord("C")):
            session.clear_record()

        stdscr.erase()
        status = session.status()
        mode, faults, echoed = session.feedback()
        armed = session.armed()
        smin, smax = session.sent_range()
        stdscr.addstr(0, 0, "RAK arm servo TUI  (USB v2 ARM_COMMAND, lease %d ms)" % lease_ms)
        stdscr.addstr(1, 0, f"link: {status}   board mode={mode} echo_seq={echoed} faults={fault_bits(faults)}")
        stdscr.addstr(2, 0, "keys: up/down select | left/right +-2 | PgUp/PgDn +-10 | Home zero | a arm | m mark | w write | c clear | q quit")
        stdscr.addstr(3, 0, "-" * 78)
        n_marks = len(session.record()["marks"])
        for i, (name, pin, kind, llo, lhi, _home) in enumerate(JOINTS):
            deg = session.target(i)
            us = urad_to_pulse_us(deg_to_urad(deg))
            marker = "->" if i == selected else "  "
            flag = "ARMED " if armed else "disarm"
            stdscr.addstr(
                4 + i, 0,
                f"{marker} {name:<15} {pin:<5} {deg:6.1f} deg  {us:4d} us  "
                f"sent[{smin[i]:5.1f}..{smax[i]:5.1f}]  [{llo:.0f}..{lhi:.0f}]  {flag}")
        bottom = 4 + len(JOINTS) + 1
        stdscr.addstr(bottom, 0,
                      f"marks: {n_marks}   record file: {record_path}")
        stdscr.addstr(bottom + 1, 0,
                      "caution: no software limits — mechanical stops stall servos; stop immediately on buzzing/heat")
        stdscr.refresh()


def write_record(record: dict, path: str) -> None:
    import json

    with open(path, "w", encoding="utf-8") as fh:
        json.dump(record, fh, ensure_ascii=False, indent=2)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Node("arm_servo_tui")
    node.declare_parameter("port", "")
    node.declare_parameter("baud", 921600)
    node.declare_parameter("lease_ms", 100)
    node.declare_parameter("rate_hz", 50.0)
    node.declare_parameter(
        "record_path", "~/rak_arm_limits.json")

    port = str(node.get_parameter("port").value)
    baud = int(node.get_parameter("baud").value)
    lease_ms = int(node.get_parameter("lease_ms").value)
    rate_hz = float(node.get_parameter("rate_hz").value)
    record_path = os.path.expanduser(
        str(node.get_parameter("record_path").value))

    if not port:
        node.get_logger().error(
            "parameter 'port' is required (e.g. port:=/dev/wheel_motor)")
        node.destroy_node()
        rclpy.shutdown()
        return
    if not LEASE_MIN_MS <= lease_ms <= LEASE_MAX_MS:
        node.get_logger().error(
            f"lease_ms must be in [{LEASE_MIN_MS}, {LEASE_MAX_MS}]")
        node.destroy_node()
        rclpy.shutdown()
        return

    session = V2ArmSession(port, baud, lease_ms, rate_hz)

    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()
    session.start()
    try:
        curses.wrapper(run_tui, session, lease_ms, record_path)
    finally:
        session.set_armed(False)
        session.request_stop()
        try:
            write_record(session.record(), record_path)
            node.get_logger().info(f"limit record written to {record_path}")
        except OSError as exc:
            node.get_logger().error(f"failed to write record: {exc}")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
