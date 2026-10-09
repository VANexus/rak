"""Byte-exact golden vectors from vehical_rak_chassis/docs/PROTOCOL.md §9.

Run:  python3 -m pytest src/rak_serial_link/test/test_protocol.py -v
"""
import pytest

from rak_serial_link.protocol import (
    CRC8_POLY,
    FrameStreamDecoder,
    build_chassis_vel_cmd,
    build_safety_cmd,
    crc8,
)


# --- CRC spot checks -----------------------------------------------------

def test_crc8_init_constants():
    assert CRC8_POLY == 0x07


def test_crc8_v1():
    # V1 CHASSIS_VEL_CMD body: F4 01 38 FF 96 00 01  -- CRC expected 0x60
    assert crc8(bytes([0xF4, 0x01, 0x38, 0xFF, 0x96, 0x00, 0x01]), 0x06) == 0x60


def test_crc8_v4():
    # V4 SAFETY_CMD body: 04 00 00 00 00 00 50  -- CRC expected 0x1D
    assert crc8(bytes([0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x50]), 0x01) == 0x1D


# --- V1 CHASSIS_VEL_CMD round-trip ---------------------------------------

V1_BYTES = bytes.fromhex("F40138FF96000160")


def test_v1_encode():
    out = build_chassis_vel_cmd(vx_mm_s=500, vy_mm_s=-200, wz_mrad_s=150,
                                counter=0, enable=True)
    assert out == V1_BYTES, f"got {out.hex().upper()}"


def test_v1_signs():
    out = build_chassis_vel_cmd(vx_mm_s=500, vy_mm_s=-200, wz_mrad_s=150, counter=0)
    assert out[0:2] == bytes([0xF4, 0x01])  # 500 LE
    assert out[2:4] == bytes([0x38, 0xFF])  # -200 LE
    assert out[4:6] == bytes([0x96, 0x00])  # 150 LE


# --- V4 SAFETY_CMD round-trip -------------------------------------------

def test_v4_safety_emergency_stop():
    # V4 from PROTOCOL.md §9: cmd_code=4 (EMERGENCY_STOP), counter=5
    # Expected 8 bytes: 04 00 00 00 00 00 50 1D
    out = build_safety_cmd(cmd_code=4, counter=5)
    expected = bytes.fromhex("040000000000501D")
    assert out == expected, f"got {out.hex().upper()}"


# --- FrameStreamDecoder -------------------------------------------------

def test_decoder_accepts_v1_then_drops_garbage():
    dec = FrameStreamDecoder()
    # Inject: noise byte + V1 + noise + V1 (counters must increment)
    noise = b"\xAA"
    v1_0 = build_chassis_vel_cmd(vx_mm_s=500, vy_mm_s=-200, wz_mrad_s=150, counter=0)
    v1_1 = build_chassis_vel_cmd(vx_mm_s=500, vy_mm_s=-200, wz_mrad_s=150, counter=1)
    frames = dec.feed(noise + v1_0 + b"\xAA" + v1_1)
    assert len(frames) == 2, f"got {len(frames)} frames"
    assert frames[0].can_id == 0x100
    assert frames[0].counter == 0
    assert frames[1].can_id == 0x100
    assert frames[1].counter == 1


def test_decoder_counters_can_wrap():
    dec = FrameStreamDecoder()
    f15 = build_chassis_vel_cmd(0, 0, 0, counter=15)
    f0 = build_chassis_vel_cmd(0, 0, 0, counter=0)
    out = dec.feed(f15 + f0)
    assert len(out) == 2
    assert out[0].counter == 15
    assert out[1].counter == 0


def test_counter_overflow_raises():
    with pytest.raises(OverflowError):
        build_chassis_vel_cmd(0, 0, 0, counter=16)


def test_int16_overflow_raises():
    with pytest.raises(OverflowError):
        build_chassis_vel_cmd(vx_mm_s=40000, vy_mm_s=0, wz_mrad_s=0)