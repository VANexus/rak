"""Version 2 wheel/joint protocol helpers for the Jetson/F407 link."""

from __future__ import annotations

from dataclasses import dataclass
import math
import struct
from typing import Sequence

from .protocol import Frame, ProtocolError, decode_frame, encode_frame

VERSION = 2
MAX_PAYLOAD = 128

TYPE_HELLO = 0x01
TYPE_HELLO_ACK = 0x02
TYPE_TIME_SYNC_REQUEST = 0x03
TYPE_TIME_SYNC_RESPONSE = 0x04
TYPE_DRIVE_COMMAND = 0x10
TYPE_ARM_COMMAND = 0x11
TYPE_SAFETY_COMMAND = 0x12
TYPE_DRIVE_STATE = 0x20
TYPE_ARM_STATE = 0x21
TYPE_DIAGNOSTICS = 0x22
TYPE_FAULT_EVENT = 0x23

FEATURE_DRIVE_WHEELS = 1 << 0
FEATURE_ARM_POSITION = 1 << 1
FEATURE_WHEEL_ODOMETRY = 1 << 2
FEATURE_ARM_MEASURED_POSITION = 1 << 3
FEATURE_GRIPPER_SERVO = 1 << 4
FEATURE_DEVICE_TIME_SYNC = 1 << 5
REQUIRED_FEATURES = FEATURE_DRIVE_WHEELS | FEATURE_ARM_POSITION | FEATURE_WHEEL_ODOMETRY

MODE_DISARMED = 0
MODE_ARMED = 1
SUBSYSTEM_DRIVE = 1
SUBSYSTEM_ARM = 2
SAFETY_STOP = 1
SAFETY_LATCH_INHIBIT = 2
SAFETY_CLEAR_INHIBIT = 3
LEASE_MIN_MS = 10
LEASE_MAX_MS = 200
WHEEL_ORDER = ("front_left", "front_right", "rear_left", "rear_right")
ARM_JOINT_ORDER = tuple(f"joint_{index}" for index in range(1, 7))
GRIPPER_SERVO = "gripper_servo"

_HELLO = struct.Struct("<II")
_HELLO_ACK = struct.Struct("<HIII")
_DRIVE_PREFIX = struct.Struct("<BHB")
_ARM_PREFIX = struct.Struct("<BHBB")
_DRIVE_STATE_PREFIX = struct.Struct("<HBBIIQ")
_ARM_STATE_PREFIX = struct.Struct("<HBBBBIQ")
_SAFETY = struct.Struct("<BB")
_TIME_REQUEST = struct.Struct("<Q")
_TIME_RESPONSE = struct.Struct("<QQQ")
_FAULT_EVENT = struct.Struct("<IHB")


@dataclass(frozen=True)
class DriveCommand:
    mode: int
    lease_ms: int
    valid_mask: int
    wheel_velocity_urad_s: tuple[int, int, int, int]


@dataclass(frozen=True)
class ArmCommand:
    mode: int
    lease_ms: int
    valid_mask: int
    joint_position_urad: tuple[int, ...]


@dataclass(frozen=True)
class DriveState:
    echoed_command_sequence: int
    mode: int
    feedback_valid_mask: int
    fault_bitmap: int
    encoder_epoch: int
    sample_time_us: int
    wheel_position_urad: tuple[int, int, int, int]
    wheel_velocity_urad_s: tuple[int, int, int, int]


@dataclass(frozen=True)
class ArmState:
    echoed_command_sequence: int
    mode: int
    axis_count: int
    measured_valid_mask: int
    measured_source: int
    fault_bitmap: int
    sample_time_us: int
    command_position_urad: tuple[int, ...]
    measured_position_urad: tuple[int, ...]


@dataclass(frozen=True)
class Diagnostics:
    device_uptime_us: int
    battery_mv: int
    board_temp_centi_c: int
    crc_error_count: int
    sequence_error_count: int
    rx_overrun_count: int
    tx_drop_count: int
    control_deadline_miss_count: int
    reset_count: int


def _mode_and_lease(mode: int, lease_ms: int) -> None:
    if mode not in (MODE_DISARMED, MODE_ARMED):
        raise ProtocolError(f"unknown actuator mode {mode}")
    if not LEASE_MIN_MS <= lease_ms <= LEASE_MAX_MS:
        raise ProtocolError(f"lease must be in [{LEASE_MIN_MS}, {LEASE_MAX_MS}] ms")


def _i32(values: Sequence[int], size: int, name: str) -> tuple[int, ...]:
    if len(values) != size:
        raise ProtocolError(f"{name} must contain exactly {size} values")
    result = tuple(int(value) for value in values)
    if any(value < -0x80000000 or value > 0x7FFFFFFF for value in result):
        raise ProtocolError(f"{name} value outside int32 range")
    return result


def si_to_micro(value: float, name: str) -> int:
    if not math.isfinite(value):
        raise ProtocolError(f"{name} must be finite")
    scaled = value * 1_000_000
    raw = math.floor(scaled + 0.5) if scaled >= 0 else math.ceil(scaled - 0.5)
    if not -0x80000000 <= raw <= 0x7FFFFFFF:
        raise ProtocolError(f"{name} outside int32 wire range")
    return raw


def micro_to_si(raw: int) -> float:
    return raw / 1_000_000.0


def encode_v2_frame(message_type: int, sequence: int, payload: bytes) -> bytes:
    if len(payload) > MAX_PAYLOAD:
        raise ProtocolError("payload exceeds 128 bytes")
    return encode_frame(Frame(VERSION, message_type, sequence, payload))


def decode_v2_frame(packet: bytes) -> Frame:
    return decode_frame(packet, require_version=VERSION)


class V2StreamDecoder:
    """[AI] 增量分隔符解码（v2 版本门）。

    protocol.StreamDecoder 固定用 require_version=PROTOCOL_VERSION(v1)，
    v2 帧会被 'unsupported protocol version' 静默丢弃——v2 会话必须用本类。
    字节级行为与 StreamDecoder 一致，仅版本门不同。
    """

    def __init__(self, max_encoded_size: int = 160):
        self._buffer = bytearray()
        self._max = max_encoded_size
        self.errors = 0

    def feed(self, data: bytes) -> list[Frame]:
        frames = []
        for byte in data:
            if byte == 0:
                if self._buffer:
                    try:
                        frames.append(decode_frame(bytes(self._buffer), require_version=VERSION))
                    except ProtocolError:
                        self.errors += 1
                    self._buffer.clear()
            else:
                self._buffer.append(byte)
                if len(self._buffer) > self._max:
                    self.errors += 1
                    self._buffer.clear()
        return frames


def pack_hello(host_build_id: int, requested_features: int) -> bytes:
    try:
        return _HELLO.pack(host_build_id, requested_features)
    except struct.error as exc:
        raise ProtocolError("HELLO field out of range") from exc


def unpack_hello_ack(payload: bytes) -> tuple[int, int, int, int]:
    if len(payload) != _HELLO_ACK.size:
        raise ProtocolError("HELLO_ACK payload must be 14 bytes")
    fields = _HELLO_ACK.unpack(payload)
    if fields[3] & REQUIRED_FEATURES != REQUIRED_FEATURES:
        raise ProtocolError("F407 lacks required v2 capabilities")
    return fields


def pack_drive_command(command: DriveCommand) -> bytes:
    _mode_and_lease(command.mode, command.lease_ms)
    values = _i32(command.wheel_velocity_urad_s, 4, "wheel velocity")
    if command.mode == MODE_ARMED and command.valid_mask != 0x0F:
        raise ProtocolError("ARMED drive command must validate all wheels")
    if command.mode == MODE_DISARMED and (command.valid_mask != 0 or any(values)):
        raise ProtocolError("DISARMED drive command must have zero mask and targets")
    return _DRIVE_PREFIX.pack(
        command.mode, command.lease_ms, command.valid_mask
    ) + struct.pack("<4i", *values)


def unpack_drive_command(payload: bytes) -> DriveCommand:
    if len(payload) != 20:
        raise ProtocolError("DRIVE_COMMAND payload must be 20 bytes")
    mode, lease_ms, valid_mask = _DRIVE_PREFIX.unpack_from(payload)
    values = struct.unpack_from("<4i", payload, _DRIVE_PREFIX.size)
    result = DriveCommand(mode, lease_ms, valid_mask, values)
    pack_drive_command(result)
    return result


def pack_arm_command(command: ArmCommand, accepted_features: int = 0) -> bytes:
    axis_count = len(command.joint_position_urad)
    if axis_count not in (6, 7):
        raise ProtocolError("ARM_COMMAND requires six joints and optional gripper")
    if axis_count == 7 and not accepted_features & FEATURE_GRIPPER_SERVO:
        raise ProtocolError("seventh gripper axis was not negotiated")
    _mode_and_lease(command.mode, command.lease_ms)
    values = _i32(command.joint_position_urad, axis_count, "joint position")
    expected_mask = (1 << axis_count) - 1
    if command.mode == MODE_ARMED and command.valid_mask != expected_mask:
        raise ProtocolError("ARMED arm command must validate every declared axis")
    if command.mode == MODE_DISARMED and (command.valid_mask != 0 or any(values)):
        raise ProtocolError("DISARMED arm command must have zero mask and targets")
    return _ARM_PREFIX.pack(
        command.mode, command.lease_ms, axis_count, command.valid_mask
    ) + struct.pack(f"<{axis_count}i", *values)


def unpack_arm_command(payload: bytes, accepted_features: int = 0) -> ArmCommand:
    if len(payload) < _ARM_PREFIX.size:
        raise ProtocolError("ARM_COMMAND payload shorter than prefix")
    mode, lease_ms, axis_count, valid_mask = _ARM_PREFIX.unpack_from(payload)
    if axis_count not in (6, 7) or len(payload) != _ARM_PREFIX.size + axis_count * 4:
        raise ProtocolError("ARM_COMMAND axis count does not match payload length")
    if axis_count == 7 and not accepted_features & FEATURE_GRIPPER_SERVO:
        raise ProtocolError("seventh gripper axis was not negotiated")
    positions = struct.unpack_from(f"<{axis_count}i", payload, _ARM_PREFIX.size)
    result = ArmCommand(mode, lease_ms, valid_mask, positions)
    pack_arm_command(result, accepted_features)
    return result


def pack_safety_command(operation: int, subsystem_mask: int) -> bytes:
    if operation not in (SAFETY_STOP, SAFETY_LATCH_INHIBIT, SAFETY_CLEAR_INHIBIT):
        raise ProtocolError(f"unknown safety operation {operation}")
    if subsystem_mask == 0 or subsystem_mask & ~3:
        raise ProtocolError("invalid safety subsystem mask")
    return _SAFETY.pack(operation, subsystem_mask)


def pack_time_sync_request(host_send_monotonic_ns: int) -> bytes:
    try:
        return _TIME_REQUEST.pack(host_send_monotonic_ns)
    except struct.error as exc:
        raise ProtocolError("time-sync request out of range") from exc


def unpack_time_sync_request(payload: bytes) -> int:
    if len(payload) != _TIME_REQUEST.size:
        raise ProtocolError("TIME_SYNC_REQUEST payload must be 8 bytes")
    return _TIME_REQUEST.unpack(payload)[0]


def pack_time_sync_response(
    host_send_monotonic_ns: int, device_rx_uptime_us: int, device_tx_uptime_us: int
) -> bytes:
    try:
        return _TIME_RESPONSE.pack(
            host_send_monotonic_ns, device_rx_uptime_us, device_tx_uptime_us
        )
    except struct.error as exc:
        raise ProtocolError("time-sync response out of range") from exc


def unpack_time_sync_response(payload: bytes) -> tuple[int, int, int]:
    if len(payload) != _TIME_RESPONSE.size:
        raise ProtocolError("TIME_SYNC_RESPONSE payload must be 24 bytes")
    return _TIME_RESPONSE.unpack(payload)


def unpack_drive_state(payload: bytes) -> DriveState:
    expected_size = _DRIVE_STATE_PREFIX.size + 48
    if len(payload) != expected_size:
        raise ProtocolError(f"DRIVE_STATE payload must be {expected_size} bytes")
    prefix = _DRIVE_STATE_PREFIX.unpack_from(payload)
    if prefix[1] not in (MODE_DISARMED, MODE_ARMED):
        raise ProtocolError("unknown DRIVE_STATE mode")
    if prefix[2] & ~0x0F:
        raise ProtocolError("reserved DRIVE_STATE feedback bits are set")
    offset = _DRIVE_STATE_PREFIX.size
    positions = struct.unpack_from("<4q", payload, offset)
    velocities = struct.unpack_from("<4i", payload, offset + 32)
    return DriveState(*prefix, positions, velocities)


def unpack_arm_state(payload: bytes, accepted_features: int = 0) -> ArmState:
    if len(payload) < _ARM_STATE_PREFIX.size:
        raise ProtocolError("ARM_STATE payload shorter than prefix")
    prefix = _ARM_STATE_PREFIX.unpack_from(payload)
    axis_count = prefix[2]
    if axis_count not in (6, 7):
        raise ProtocolError("ARM_STATE axis count must be six or seven")
    if axis_count == 7 and not accepted_features & FEATURE_GRIPPER_SERVO:
        raise ProtocolError("seventh gripper axis was not negotiated")
    if prefix[1] not in (MODE_DISARMED, MODE_ARMED):
        raise ProtocolError("unknown ARM_STATE mode")
    if prefix[4] not in (0, 1, 2, 3):
        raise ProtocolError("unknown ARM_STATE measured source")
    expected_size = _ARM_STATE_PREFIX.size + axis_count * 8
    if len(payload) != expected_size:
        raise ProtocolError(f"ARM_STATE payload must be {expected_size} bytes")
    if prefix[3] & ~((1 << axis_count) - 1):
        raise ProtocolError("reserved ARM_STATE feedback bits are set")
    offset = _ARM_STATE_PREFIX.size
    commands = struct.unpack_from(f"<{axis_count}i", payload, offset)
    measured = struct.unpack_from(f"<{axis_count}i", payload, offset + axis_count * 4)
    return ArmState(*prefix, commands, measured)


def unpack_fault_event(payload: bytes) -> tuple[int, int, int]:
    if len(payload) != _FAULT_EVENT.size:
        raise ProtocolError("FAULT_EVENT payload must be 7 bytes")
    return _FAULT_EVENT.unpack(payload)


def unpack_diagnostics(payload: bytes) -> Diagnostics:
    if len(payload) != 36:
        raise ProtocolError("DIAGNOSTICS payload must be 36 bytes")
    return Diagnostics(*struct.unpack("<QHhIIIIII", payload))


def drive_command_from_si(
    mode: int, lease_ms: int, wheel_velocity_rad_s: Sequence[float]
) -> DriveCommand:
    values = tuple(si_to_micro(value, "wheel velocity") for value in wheel_velocity_rad_s)
    mask = 0x0F if mode == MODE_ARMED else 0
    result = DriveCommand(mode, lease_ms, mask, values)  # type: ignore[arg-type]
    pack_drive_command(result)
    return result


def arm_command_from_si(
    mode: int,
    lease_ms: int,
    joint_position_rad: Sequence[float],
    accepted_features: int = 0,
) -> ArmCommand:
    values = tuple(si_to_micro(value, "joint position") for value in joint_position_rad)
    mask = (1 << len(values)) - 1 if mode == MODE_ARMED and len(values) in (6, 7) else 0
    result = ArmCommand(mode, lease_ms, mask, values)
    pack_arm_command(result, accepted_features)
    return result
