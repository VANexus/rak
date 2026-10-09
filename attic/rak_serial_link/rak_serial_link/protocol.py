"""Pure wire codec for CAN v3 frames over a UART.

Byte layout (every record, 8 bytes; little-endian; see vehical_rak_chassis
docs/PROTOCOL.md for the canonical spec):

    [0..5]   signals, packed little-endian, 16-bit each (3 signals x 2 bytes)
    [6]      high nibble = alive counter (0..15), low nibble = flags
    [7]      CRC-8 (poly 0x07, no reflection, no final xor) over [0..6]
             initial value = data_id (per-frame constant)

This is the same byte format used by rak_can_driver for python-can; the
serial module intentionally reimplements it (no cross-import) so the wire
layer has no upstream package dependencies.

Sync: UART has no frame delimiter, so the receiver walks the byte stream
looking for any 8-byte window whose CRC matches a known message id. Once
matched, the alive counter is used to align subsequent frames: each valid
frame increments its counter by +1 (mod 16), so an off-by-N alignment is
detected immediately and the stream is re-aligned to the most recent
known-good frame.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

PROTOCOL_VERSION = 3
CRC8_POLY = 0x07


def crc8(data: bytes, data_id: int) -> int:
    """CRC-8/ATM: poly 0x07, no reflection, no final xor, init = data_id."""
    crc = data_id & 0xFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ CRC8_POLY) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


# --- Frame table (subset; the host only needs to know what it could send / see) ---
# id : (data_id, name)
MSG_BY_ID: dict[int, dict] = {
    0x010: dict(data_id=0x01, name="SAFETY_CMD"),
    0x020: dict(data_id=0x02, name="EMCY_CHASSIS"),
    0x021: dict(data_id=0x03, name="EMCY_ARM"),
    0x030: dict(data_id=0x04, name="XFER_CH_TO_ARM"),
    0x031: dict(data_id=0x05, name="XFER_ARM_TO_CH"),
    0x100: dict(data_id=0x06, name="CHASSIS_VEL_CMD"),
    0x101: dict(data_id=0x07, name="ARM_JOINT_CMD_A"),
    0x102: dict(data_id=0x08, name="ARM_JOINT_CMD_B"),
    0x200: dict(data_id=0x09, name="CHASSIS_VEL_FB"),
    0x201: dict(data_id=0x0A, name="CHASSIS_DIAG"),
    0x210: dict(data_id=0x0B, name="ARM_JOINT_FB_A"),
    0x211: dict(data_id=0x0C, name="ARM_JOINT_FB_B"),
    0x212: dict(data_id=0x0D, name="ARM_DIAG"),
    0x230: dict(data_id=0x0E, name="NODE_INFO_CHASSIS"),
    0x231: dict(data_id=0x0F, name="ARM_DIAG"),
}


def _pack_i16(value: int) -> bytes:
    if not -0x8000 <= value <= 0x7FFF:
        raise OverflowError(f"int16 overflow: {value}")
    return value.to_bytes(2, "little", signed=True)


def _pack_u16(value: int) -> bytes:
    if not 0 <= value <= 0xFFFF:
        raise OverflowError(f"uint16 overflow: {value}")
    return value.to_bytes(2, "little", signed=False)


def build_chassis_vel_cmd(vx_mm_s: int, vy_mm_s: int, wz_mrad_s: int,
                          counter: int = 0, enable: bool = True,
                          speed_limit_low: bool = False) -> bytes:
    """Pack a CHASSIS_VEL_CMD record (id 0x100)."""
    if not 0 <= counter <= 15:
        raise OverflowError(f"counter must fit 4 bits: {counter}")
    flags = (1 if enable else 0) | (2 if speed_limit_low else 0)
    body = bytearray()
    body += _pack_i16(vx_mm_s)
    body += _pack_i16(vy_mm_s)
    body += _pack_i16(wz_mrad_s)
    body.append(((counter & 0xF) << 4) | (flags & 0xF))
    body.append(crc8(bytes(body), 0x06))
    return bytes(body)


def build_safety_cmd(cmd_code: int, counter: int = 0) -> bytes:
    """Pack a SAFETY_CMD record (id 0x010). cmd_code per PROTOCOL.md §7.1."""
    if not 0 <= counter <= 15:
        raise OverflowError(f"counter must fit 4 bits: {counter}")
    if not 1 <= cmd_code <= 9:
        raise ValueError(f"cmd_code out of range [1..9]: {cmd_code}")
    body = bytearray()
    body += _pack_u16(cmd_code)
    body += _pack_u16(0)  # rsvd16
    body += _pack_u16(0)  # rsvd16_2
    body.append(((counter & 0xF) << 4) | 0x0)  # flags low nibble = cmd_code per spec
    body.append(crc8(bytes(body), 0x01))
    return bytes(body)


SAFETY_CMD_NAMES: dict[int, str] = {
    1: "START", 2: "STOP", 4: "EMERGENCY_STOP", 5: "CLEAR_ESTOP",
    6: "CLEAR_FAULT", 7: "SOFT_RESET", 8: "HOME_REQUEST", 9: "RELEASE_HOLD",
}


@dataclass(frozen=True)
class ReceivedFrame:
    can_id: int
    raw: bytes
    counter: int
    flags: int

    @property
    def name(self) -> str:
        return MSG_BY_ID.get(self.can_id, {}).get("name", "UNKNOWN")


# --- Stream synchronizer -------------------------------------------------

class FrameStreamDecoder:
    """Walk a byte stream and yield ReceivedFrame records.

    Tries every 8-byte window for a CRC match against any known message id.
    Once one matches, the next frame must have counter+1 (mod 16); otherwise
    the stream is re-aligned. The 4-bit counter monotonicity is the only
    bound on the serial byte stream alignment -- there's no delimiter on
    UART, so we lean on CRC + counter.

    This is intentionally a small re-implementation of the logic in
    rak_can_driver/protocol.py; the duplicate is on purpose so this package
    has zero upstream dependencies.
    """

    def __init__(self):
        self._buffer = bytearray()
        self._last_counter: Optional[int] = None
        self._last_can_id: Optional[int] = None
        self.errors = 0
        self.frames = 0

    def feed(self, data: bytes) -> list[ReceivedFrame]:
        out: list[ReceivedFrame] = []
        self._buffer.extend(data)
        # Try to find the longest run of valid frames; slide window.
        while len(self._buffer) >= 8:
            accepted = self._try_window(0)
            if accepted is None:
                # slide one byte and retry
                del self._buffer[0]
                self.errors += 1
            else:
                out.append(accepted)
                # consume the 8 bytes that matched
                del self._buffer[:8]
                self.frames += 1
        return out

    def _try_window(self, offset: int) -> Optional[ReceivedFrame]:
        window = bytes(self._buffer[offset:offset + 8])
        if len(window) < 8:
            return None
        body = window[:7]
        crc_in_frame = window[7]
        # Try every known message id at this window
        for can_id, msg in MSG_BY_ID.items():
            expected_crc = crc8(body, msg["data_id"])
            if expected_crc != crc_in_frame:
                continue
            counter = (window[6] >> 4) & 0xF
            flags = window[6] & 0xF
            # Counter monotonicity check
            if (self._last_counter is not None and
                    self._last_can_id == can_id and
                    (counter - self._last_counter) & 0xF not in (1, 0)):
                # Off-by-N alignment; caller will slide on next iteration
                return None
            self._last_counter = counter
            self._last_can_id = can_id
            return ReceivedFrame(can_id=can_id, raw=window, counter=counter, flags=flags)
        return None