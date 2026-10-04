"""Jetson-to-F407 UART v1 framing and payload helpers."""

from __future__ import annotations

from dataclasses import dataclass
import math
import struct
from typing import Iterable

PROTOCOL_VERSION = 1
MAX_PAYLOAD = 128
HEADER = struct.Struct("<BBHH")
CRC = struct.Struct("<H")
VELOCITY = struct.Struct("<BHiii")
BASE_STATE = struct.Struct("<HBIiiiI")
HELLO = struct.Struct("<II")
HELLO_ACK = struct.Struct("<HIII")
FAULT_EVENT = struct.Struct("<IH")

TYPE_HELLO = 0x01
TYPE_HELLO_ACK = 0x02
TYPE_SET_BASE_VELOCITY = 0x10
TYPE_BASE_STATE = 0x11
TYPE_FAULT_EVENT = 0x12

MODE_DISARMED = 0
MODE_ARMED = 1
STATE_BOOTING = 0
STATE_DISARMED = 1
STATE_ARMED = 2
STATE_FAULT = 3

FEATURE_BASE_VELOCITY = 1 << 0
FEATURE_BASE_TELEMETRY = 1 << 1
REQUIRED_FEATURES = FEATURE_BASE_VELOCITY | FEATURE_BASE_TELEMETRY


class ProtocolError(ValueError):
    """A malformed or incompatible wire frame."""


@dataclass(frozen=True)
class Frame:
    version: int
    message_type: int
    sequence: int
    payload: bytes


@dataclass(frozen=True)
class BaseState:
    echoed_command_sequence: int
    state: int
    fault_bitmap: int
    vx_mm_s: int
    vy_mm_s: int
    wz_mrad_s: int
    device_uptime_ms: int


def crc16_ccitt_false(data: bytes, init: int = 0xFFFF) -> int:
    crc = init
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def cobs_encode(data: bytes) -> bytes:
    """Encode one packet without its zero delimiter."""
    out = bytearray([0])
    code_index = 0
    code = 1
    for byte in data:
        if byte == 0:
            out[code_index] = code
            code_index = len(out)
            out.append(0)
            code = 1
        else:
            out.append(byte)
            code += 1
            if code == 0xFF:
                out[code_index] = code
                code_index = len(out)
                out.append(0)
                code = 1
    out[code_index] = code
    return bytes(out)


def cobs_decode(data: bytes) -> bytes:
    if not data:
        raise ProtocolError("empty COBS packet")
    out = bytearray()
    index = 0
    while index < len(data):
        code = data[index]
        if code == 0:
            raise ProtocolError("zero byte inside COBS packet")
        index += 1
        end = index + code - 1
        if end > len(data):
            raise ProtocolError("truncated COBS block")
        out.extend(data[index:end])
        index = end
        if code != 0xFF and index < len(data):
            out.append(0)
    return bytes(out)


def encode_raw_frame(frame: Frame) -> bytes:
    if not 0 <= frame.sequence <= 0xFFFF:
        raise ProtocolError("sequence out of range")
    if len(frame.payload) > MAX_PAYLOAD:
        raise ProtocolError("payload exceeds 128 bytes")
    header = HEADER.pack(
        frame.version, frame.message_type, frame.sequence, len(frame.payload)
    )
    content = header + frame.payload
    return content + CRC.pack(crc16_ccitt_false(content))


def encode_frame(frame: Frame) -> bytes:
    return cobs_encode(encode_raw_frame(frame)) + b"\x00"


def decode_raw_frame(raw: bytes, *, require_version: int = PROTOCOL_VERSION) -> Frame:
    if len(raw) < HEADER.size + CRC.size:
        raise ProtocolError("frame shorter than header and CRC")
    version, message_type, sequence, payload_length = HEADER.unpack_from(raw)
    if version != require_version:
        raise ProtocolError(f"unsupported protocol version {version}")
    if payload_length > MAX_PAYLOAD:
        raise ProtocolError("payload exceeds 128 bytes")
    expected_length = HEADER.size + payload_length + CRC.size
    if len(raw) != expected_length:
        raise ProtocolError("payload length does not match frame length")
    expected_crc = CRC.unpack_from(raw, len(raw) - CRC.size)[0]
    if crc16_ccitt_false(raw[:-CRC.size]) != expected_crc:
        raise ProtocolError("CRC mismatch")
    return Frame(version, message_type, sequence, raw[HEADER.size:-CRC.size])


def decode_frame(encoded: bytes, *, require_version: int = PROTOCOL_VERSION) -> Frame:
    packet = encoded[:-1] if encoded.endswith(b"\x00") else encoded
    return decode_raw_frame(cobs_decode(packet), require_version=require_version)


class StreamDecoder:
    """Incremental delimiter-framed decoder; malformed packets are discarded."""

    def __init__(self, max_encoded_size: int = MAX_PAYLOAD + 16):
        self._buffer = bytearray()
        self._max_encoded_size = max_encoded_size
        self.errors = 0
        self._dropping = False

    def feed(self, data: bytes) -> list[Frame]:
        frames = []
        for byte in data:
            if byte == 0:
                if self._dropping:
                    self._dropping = False
                    self._buffer.clear()
                    continue
                if not self._buffer:
                    continue
                try:
                    frames.append(decode_frame(bytes(self._buffer)))
                except ProtocolError:
                    self.errors += 1
                self._buffer.clear()
            elif not self._dropping:
                self._buffer.append(byte)
                if len(self._buffer) > self._max_encoded_size:
                    self.errors += 1
                    self._buffer.clear()
                    self._dropping = True
        return frames


def sequence_is_newer(candidate: int, previous: int) -> bool:
    delta = (candidate - previous) & 0xFFFF
    return 1 <= delta <= 0x7FFF


def pack_velocity(
    mode: int, lease_ms: int, vx_mm_s: int, vy_mm_s: int, wz_mrad_s: int
) -> bytes:
    if mode not in (MODE_DISARMED, MODE_ARMED):
        raise ProtocolError("unknown command mode")
    if not 10 <= lease_ms <= 50:
        raise ProtocolError("lease must be in [10, 50] ms")
    if mode == MODE_DISARMED and (vx_mm_s or vy_mm_s or wz_mrad_s):
        raise ProtocolError("DISARMED command must request zero velocity")
    try:
        return VELOCITY.pack(mode, lease_ms, vx_mm_s, vy_mm_s, wz_mrad_s)
    except struct.error as exc:
        raise ProtocolError("velocity value is out of int32 range") from exc


def unpack_base_state(payload: bytes) -> BaseState:
    if len(payload) != BASE_STATE.size:
        raise ProtocolError("BASE_STATE payload must be 23 bytes")
    return BaseState(*BASE_STATE.unpack(payload))


def pack_base_state(state: BaseState) -> bytes:
    try:
        return BASE_STATE.pack(
            state.echoed_command_sequence,
            state.state,
            state.fault_bitmap,
            state.vx_mm_s,
            state.vy_mm_s,
            state.wz_mrad_s,
            state.device_uptime_ms,
        )
    except struct.error as exc:
        raise ProtocolError("BASE_STATE field out of range") from exc


def twist_to_wire(vx_m_s: float, vy_m_s: float, wz_rad_s: float) -> tuple[int, int, int]:
    values = (vx_m_s, vy_m_s, wz_rad_s)
    if not all(math.isfinite(value) for value in values):
        raise ProtocolError("Twist contains a non-finite value")
    converted = (round(vx_m_s * 1000), round(vy_m_s * 1000), round(wz_rad_s * 1000))
    if any(value < -0x80000000 or value > 0x7FFFFFFF for value in converted):
        raise ProtocolError("Twist exceeds wire range")
    return converted


def hello_payload(host_build_id: int, requested_features: int = REQUIRED_FEATURES) -> bytes:
    return HELLO.pack(host_build_id, requested_features)


def hello_ack_compatible(payload: bytes) -> tuple[int, int, int, int]:
    if len(payload) != HELLO_ACK.size:
        raise ProtocolError("HELLO_ACK payload must be 14 bytes")
    values = HELLO_ACK.unpack(payload)
    if values[3] & REQUIRED_FEATURES != REQUIRED_FEATURES:
        raise ProtocolError("F407 did not accept required base capabilities")
    return values


def fault_event(payload: bytes) -> tuple[int, int]:
    if len(payload) != FAULT_EVENT.size:
        raise ProtocolError("FAULT_EVENT payload must be 6 bytes")
    return FAULT_EVENT.unpack(payload)


def frames_from_chunks(chunks: Iterable[bytes]) -> list[Frame]:
    decoder = StreamDecoder()
    return [frame for chunk in chunks for frame in decoder.feed(chunk)]
