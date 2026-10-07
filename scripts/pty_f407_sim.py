"""Pseudo-F407 firmware simulator for UART v1 / v1.1 protocol testing.

Opens a pty (pseudo-terminal) and pretends to be the base F407 firmware.
Run with:

    python3 scripts/pty_f407_sim.py /tmp/f407_pty --protocol v1.1

Then point base_uart_driver at the slave side:

    ros2 launch rak_bringup base_uart.launch.py port:=/tmp/f407_pty

The simulator decodes HELLO/SET_BASE_VELOCITY, replies with HELLO_ACK/BASE_STATE,
tracks lease (50ms) and max_exec_ms timers (for v1.1 atomic actions), validates
action_type vs twist consistency (fault bit 7 on mismatch), and FAULTs on
unexpected frames.
"""

from __future__ import annotations

import argparse
import os
import pty
import select
import signal
import struct
import sys
import termios
import time

# ---------------------------------------------------------------------------
# Protocol constants (mirrors rak_base_driver.protocol)
# ---------------------------------------------------------------------------
PROTOCOL_VERSION = 1
HEADER = struct.Struct("<BBHH")  # version, type, sequence, payload_length
CRC = struct.Struct("<H")
MAX_PAYLOAD = 128

# v1 message types
TYPE_HELLO = 0x01
TYPE_HELLO_ACK = 0x02
TYPE_SET_BASE_VELOCITY = 0x10
TYPE_BASE_STATE = 0x11
TYPE_FAULT_EVENT = 0x12
TYPE_BASE_ODOMETRY = 0x13  # v1.1 only; recognized but not emitted

# v1 BASE_STATE.state
STATE_BOOTING = 0
STATE_DISARMED = 1
STATE_ARMED = 2
STATE_FAULT = 3

# fault bits
FAULT_EMERGENCY_STOP = 1 << 0
FAULT_LEASE_TIMEOUT = 1 << 1
FAULT_CAN_PEER_OFFLINE = 1 << 2
FAULT_PEER_FAULT = 1 << 3
FAULT_MOTOR_DRIVER = 1 << 4
FAULT_UNDERVOLTAGE = 1 << 5
FAULT_OVERTEMPERATURE = 1 << 6
FAULT_INVALID_FEEDBACK = 1 << 7

# feature bits
FEATURE_BASE_VELOCITY = 1 << 0
FEATURE_BASE_TELEMETRY = 1 << 1
FEATURE_V1_1_ACTIONS = 1 << 3  # v1.1

# v1 SET_BASE_VELOCITY (15 bytes)
SET_BASE_VELOCITY_V1 = struct.Struct("<BHiii")

# v1.1 SET_BASE_VELOCITY (19 bytes): mode + lease_ms + action_type + reserved
# pad + vx/vy/wz + max_exec_ms. See docs/protocol/jetson_f407_uart_v1.1.md.
SET_BASE_VELOCITY_V1_1 = struct.Struct("<BHBBiiiH")

# action types
ACTION_CONTINUOUS = 0
ACTION_FORWARD = 1
ACTION_BACKWARD = 2
ACTION_STRAFE_LEFT = 4
ACTION_STRAFE_RIGHT = 5
ACTION_ROTATE_CW = 6
ACTION_ROTATE_CCW = 7
ACTION_STOP = 8

ACTION_NAMES = {
    0: "CONTINUOUS",
    1: "FORWARD",
    2: "BACKWARD",
    4: "STRAFE_LEFT",
    5: "STRAFE_RIGHT",
    6: "ROTATE_CW",
    7: "ROTATE_CCW",
    8: "STOP",
}

# ---------------------------------------------------------------------------
# CRC16-CCITT-FALSE (matches rak_base_driver.protocol.crc16_ccitt_false)
# ---------------------------------------------------------------------------
def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


# ---------------------------------------------------------------------------
# COBS encode/decode (matches rak_base_driver.protocol.cobs_encode/decode)
# ---------------------------------------------------------------------------
def cobs_encode(data: bytes) -> bytes:
    out = bytearray()
    out.append(0x00)  # placeholder for code byte
    code_index = 0
    for i, b in enumerate(data, start=1):
        if b == 0:
            out[code_index] = i - (code_index or 0)
            code_index = i
            out.append(0x00)
        else:
            out.append(b)
    out[code_index] = len(out) - code_index
    return bytes(out)


def cobs_decode(data: bytes) -> bytes:
    """Decode one COBS-encoded block. Caller (StreamDecoder) catches ValueError
    when the block is malformed and discards it."""
    out = bytearray()
    i = 0
    while i < len(data):
        code = data[i]
        if code == 0:
            raise ValueError("zero code byte in COBS body")
        # need to read (code-1) bytes after the code byte
        if i + code > len(data):
            raise ValueError("truncated COBS block")
        for j in range(1, code):
            out.append(data[i + j])
        if code < 0xFF and (i + code) < len(data):
            out.append(0x00)
        i += code
    return bytes(out)


# ---------------------------------------------------------------------------
# Frame codec
# ---------------------------------------------------------------------------
class Frame:
    def __init__(self, version, mtype, sequence, payload):
        self.version = version
        self.mtype = mtype
        self.sequence = sequence
        self.payload = payload


class StreamDecoder:
    """Accumulate bytes, yield one Frame per COBS-delimited packet.

    Garbage chunks (no zero delimiter, too long, malformed COBS, bad CRC,
    unknown version) are dropped silently. The first valid frame is the only
    kind of payload we want from the wire.
    """

    def __init__(self):
        self._buf = bytearray()

    def feed(self, chunk: bytes) -> list[Frame]:
        self._buf.extend(chunk)
        out: list[Frame] = []
        while True:
            i = self._buf.find(b"\x00")
            if i < 0:
                # bound buffer size; flush if it grows too large without a delimiter
                if len(self._buf) > MAX_PAYLOAD + 16:
                    self._buf.clear()
                break
            if i == 0:
                self._buf.pop(0)
                continue
            pkt = bytes(self._buf[:i])
            self._buf = self._buf[i + 1:]
            if not pkt:
                continue
            try:
                raw = cobs_decode(pkt)
            except (ValueError, IndexError):
                continue  # malformed; discard
            if len(raw) < HEADER.size + CRC.size:
                continue
            version, mtype, seq, plen = HEADER.unpack_from(raw)
            if version != PROTOCOL_VERSION:
                continue
            if plen > MAX_PAYLOAD:
                continue
            expected = HEADER.size + plen + CRC.size
            if len(raw) != expected:
                continue
            crc_rx = CRC.unpack_from(raw, len(raw) - CRC.size)[0]
            if crc16_ccitt_false(raw[:-CRC.size]) != crc_rx:
                continue
            out.append(Frame(version, mtype, seq, raw[HEADER.size:-CRC.size]))
        return out


def encode_frame(frame: Frame) -> bytes:
    body = HEADER.pack(frame.version, frame.mtype, frame.sequence, len(frame.payload))
    body += frame.payload
    body += CRC.pack(crc16_ccitt_false(body))
    return cobs_encode(body) + b"\x00"


# ---------------------------------------------------------------------------
# F407 simulator state
# ---------------------------------------------------------------------------
class F407Sim:
    def __init__(self, protocol_version: str):
        self.protocol_version = protocol_version  # "v1" or "v1.1"
        self.state = STATE_BOOTING
        self.fault_bitmap = 0
        self.mode = 0  # 0=DISARMED, 1=ARMED
        self.last_command_seq = None
        self.last_command_received = 0.0
        self.last_measured_vx = 0
        self.last_measured_vy = 0
        self.last_measured_wz = 0
        self.device_uptime_start = time.monotonic()
        self.next_tx_seq = 0
        self.boot_id = 0xC0FFEE01
        self.action_type = ACTION_CONTINUOUS
        self.max_exec_ms = 0
        self.action_deadline = 0.0  # monotonic; 0 = inactive
        self.lease_ms = 50
        self.lease_deadline = 0.0
        self.last_action_type_seen = None
        self.last_invalid_feedback_log = None
        self.host_features = 0

    def elapsed_ms(self) -> int:
        return int((time.monotonic() - self.device_uptime_start) * 1000) & 0xFFFFFFFF

    def maybe_timeout(self) -> None:
        """Check lease + max_exec timers. Mutates state if either expired."""
        now = time.monotonic()
        # v1 lease timeout
        if self.last_command_seq is not None and now > self.lease_deadline:
            self.fault_bitmap |= FAULT_LEASE_TIMEOUT
            self.state = STATE_DISARMED
            self.mode = 0
            self.last_command_seq = None
        # v1.1 atomic-action timeout (independent of lease)
        if self.action_deadline and now > self.action_deadline:
            self.fault_bitmap |= FAULT_LEASE_TIMEOUT  # reuse bit 1 or pick new?
            self.action_deadline = 0.0
            self.state = STATE_DISARMED
            self.mode = 0

    def make_hello_ack(self, host_features: int) -> Frame:
        # Echo bit 0|1; allow v1.1 actions if our protocol is v1.1
        accepted = FEATURE_BASE_VELOCITY | FEATURE_BASE_TELEMETRY
        if self.protocol_version == "v1.1" and (host_features & FEATURE_V1_1_ACTIONS):
            accepted |= FEATURE_V1_1_ACTIONS
        self.host_features = accepted
        board_id = 0x0407  # STM32F407 family code
        fw_version = (1 << 16) | (1 << 8) | 0  # v1.1.0
        payload = struct.pack("<HIII", board_id, fw_version, self.boot_id, accepted)
        return Frame(PROTOCOL_VERSION, TYPE_HELLO_ACK, self.next_tx_seq, payload)

    def validate_action_consistency(self, action_type: int, vx: int, vy: int, wz: int) -> bool:
        if action_type == ACTION_CONTINUOUS:
            return True  # no constraint
        if action_type == ACTION_FORWARD:
            return vx > 0 and vy == 0 and wz == 0
        if action_type == ACTION_BACKWARD:
            return vx < 0 and vy == 0 and wz == 0
        if action_type == ACTION_STRAFE_LEFT:
            return vx == 0 and vy > 0 and wz == 0
        if action_type == ACTION_STRAFE_RIGHT:
            return vx == 0 and vy < 0 and wz == 0
        if action_type == ACTION_ROTATE_CW:
            return vx == 0 and vy == 0 and wz > 0
        if action_type == ACTION_ROTATE_CCW:
            return vx == 0 and vy == 0 and wz < 0
        if action_type == ACTION_STOP:
            return vx == 0 and vy == 0 and wz == 0
        return False  # unknown action_type

    def handle_set_base_velocity(self, frame: Frame) -> None:
        """Decode v1 or v1.1 SET_BASE_VELOCITY, apply consistency + timers."""
        # 15 (v1) or 19 (v1.1) byte payload
        if len(frame.payload) == SET_BASE_VELOCITY_V1.size:
            mode, lease_ms, vx, vy, wz = SET_BASE_VELOCITY_V1.unpack(frame.payload)
            action_type = ACTION_CONTINUOUS
            max_exec_ms = 0
        elif len(frame.payload) == SET_BASE_VELOCITY_V1_1.size:
            # Layout: B(mode) H(lease_ms) B(action_type) B(reserved) i(vx) i(vy) i(wz) H(max_exec_ms)
            mode, lease_ms, action_type, _reserved, vx, vy, wz, max_exec_ms = (
                SET_BASE_VELOCITY_V1_1.unpack(frame.payload)
            )
        else:
            raise ValueError(f"unexpected payload size {len(frame.payload)}")

        now = time.monotonic()
        self.last_command_seq = frame.sequence
        self.last_command_received = now
        self.mode = mode
        self.lease_ms = lease_ms
        self.lease_deadline = now + lease_ms / 1000.0

        # Validate action_type ↔ twist consistency (v1.1 only)
        if action_type != ACTION_CONTINUOUS:
            if not self.validate_action_consistency(action_type, vx, vy, wz):
                self.fault_bitmap |= FAULT_INVALID_FEEDBACK
                self.state = STATE_FAULT
                self.action_deadline = 0.0
                return
            # valid atomic action
            self.action_type = action_type
            self.max_exec_ms = max_exec_ms
            if max_exec_ms > 0:
                self.action_deadline = now + max_exec_ms / 1000.0
            else:
                self.action_deadline = 0.0
        else:
            self.action_type = ACTION_CONTINUOUS
            self.max_exec_ms = 0
            self.action_deadline = 0.0

        # Simulated measured velocity (just echo the command for test purposes)
        self.last_measured_vx = vx
        self.last_measured_vy = vy
        self.last_measured_wz = wz
        if mode == 1 and self.fault_bitmap == 0:
            self.state = STATE_ARMED
        elif mode == 0:
            self.state = STATE_DISARMED

    def make_base_state(self) -> Frame:
        payload = struct.pack(
            "<HBIiiiI",
            self.last_command_seq if self.last_command_seq is not None else 0,
            self.state,
            self.fault_bitmap & 0xFFFFFFFF,
            self.last_measured_vx,
            self.last_measured_vy,
            self.last_measured_wz,
            self.elapsed_ms(),
        )
        return Frame(PROTOCOL_VERSION, TYPE_BASE_STATE, self.next_tx_seq, payload)

    def make_fault_event(self) -> Frame:
        self.next_tx_seq = (self.next_tx_seq + 1) & 0xFFFF
        payload = struct.pack("<IH", self.fault_bitmap & 0xFFFFFFFF,
                              self.last_command_seq if self.last_command_seq is not None else 0)
        return Frame(PROTOCOL_VERSION, TYPE_FAULT_EVENT, self.next_tx_seq, payload)

    def next_tx(self) -> Frame:
        self.next_tx_seq = (self.next_tx_seq + 1) & 0xFFFF
        return self.make_base_state()


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------
def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("symlink", help="path to create the pty symlink at (e.g. /tmp/f407_pty)")
    p.add_argument("--protocol", choices=["v1", "v1.1"], default="v1.1")
    p.add_argument("--boot-id", type=lambda s: int(s, 0), default=0xC0FFEE01)
    args = p.parse_args()

    sim = F407Sim(args.protocol)
    sim.boot_id = args.boot_id

    master_fd, slave_fd = pty.openpty()
    slave_name = os.ttyname(slave_fd)
    # Put slave in raw mode and turn off ECHO. Default pty cooked mode echoes
    # back every byte the master writes, which would feed the master-side
    # decoder its own BASE_STATE replies. cobs_decode would then choke on the
    # partial echoes. Raw + noecho is what a real F407 UART link looks like.
    attrs = termios.tcgetattr(slave_fd)
    # Disable input processing (ICANON, ECHO, ISIG, etc.)
    attrs[3] &= ~(termios.ICANON | termios.ECHO | termios.ECHONL | termios.ISIG |
                    termios.IEXTEN | termios.IXON | termios.IXOFF | termios.ICRNL |
                    termios.IGNCR | termios.INLCR)
    # Disable output processing
    attrs[4] &= ~(termios.OPOST | termios.ONLCR | termios.OCRNL)
    # VMIN=0, VTIME=0 -> read() returns immediately with whatever is available
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(slave_fd, termios.TCSANOW, attrs)

    if os.path.exists(args.symlink):
        os.unlink(args.symlink)
    os.symlink(slave_name, args.symlink)
    print(f"[f407-sim] slave pty: {slave_name}")
    print(f"[f407-sim] symlink:   {args.symlink}")
    print(f"[f407-sim] protocol:  {args.protocol}")
    print(f"[f407-sim] boot_id:   0x{sim.boot_id:08x}")
    print(f"[f407-sim] waiting for host HELLO...")

    decoder = StreamDecoder()
    os.set_blocking(master_fd, False)

    def cleanup(*_):
        try:
            os.unlink(args.symlink)
        except FileNotFoundError:
            pass
        try:
            os.close(master_fd)
        except OSError:
            pass
        sys.exit(0)

    signal.signal(signal.SIGINT, cleanup)
    signal.signal(signal.SIGTERM, cleanup)

    last_state_tx = 0.0
    STATE_HZ = 100  # 100Hz BASE_STATE
    STATE_PERIOD = 1.0 / STATE_HZ

    while True:
        # 1) read all available from master
        try:
            r, _, _ = select.select([master_fd], [], [], 0)
        except OSError:
            break
        if r:
            try:
                chunk = os.read(master_fd, 256)
            except OSError:
                break
            if not chunk:
                break
            for frame in decoder.feed(chunk):
                if frame.mtype == TYPE_HELLO:
                    host_build_id, requested_features = struct.unpack("<II", frame.payload)
                    print(f"[f407-sim] HELLO host_build_id=0x{host_build_id:08x} "
                          f"requested_features=0x{requested_features:08x}")
                    ack = sim.make_hello_ack(requested_features)
                    os.write(master_fd, encode_frame(ack))
                    print(f"[f407-sim] -> HELLO_ACK accepted_features=0x{sim.host_features:08x}")
                elif frame.mtype == TYPE_SET_BASE_VELOCITY:
                    try:
                        sim.handle_set_base_velocity(frame)
                    except Exception as exc:
                        print(f"[f407-sim] dropped malformed SET_BASE_VELOCITY: {exc}")
                        continue
                    name = ACTION_NAMES.get(sim.action_type, f"UNKNOWN({sim.action_type})")
                    if sim.fault_bitmap & FAULT_INVALID_FEEDBACK:
                        print(f"[f407-sim] SET_BASE_VELOCITY seq={frame.sequence} "
                              f"INVALID ({name}, vx={sim.last_measured_vx}, "
                              f"vy={sim.last_measured_vy}, wz={sim.last_measured_wz}) -> fault bit 7")
                    else:
                        print(f"[f407-sim] SET_BASE_VELOCITY seq={frame.sequence} "
                              f"{name} vx={sim.last_measured_vx} vy={sim.last_measured_vy} "
                              f"wz={sim.last_measured_wz} max_exec={sim.max_exec_ms}ms "
                              f"lease={sim.lease_ms}ms")
                else:
                    print(f"[f407-sim] unknown frame type=0x{frame.mtype:02x} (ignored)")

        # 2) timer checks
        sim.maybe_timeout()

        # 3) periodic BASE_STATE
        now = time.monotonic()
        if now - last_state_tx >= STATE_PERIOD:
            last_state_tx = now
            frame = sim.next_tx()
            try:
                os.write(master_fd, encode_frame(frame))
            except OSError:
                break

        # 4) small sleep to avoid busy loop
        time.sleep(0.001)


if __name__ == "__main__":
    sys.exit(main())