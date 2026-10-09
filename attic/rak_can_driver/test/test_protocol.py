"""Verify the rak host implementation byte-for-byte against the chassis's
golden vectors V1..V6 in vehical_rak_chassis/docs/PROTOCOL.md §9.

Run with:  python3 -m pytest src/rak_can_driver/test/test_protocol.py -v
"""

from __future__ import annotations

import pytest

from rak_can_driver.protocol import (
    ChassisVelCmd,
    ChassisDiag,
    ChassisVelFb,
    SafetyCmd,
    build_frame,
    crc8,
    decode_frame,
)


# ---------------------------------------------------------------------------
# CRC-8 spot-checks (sanity for the polynomial and init convention)
# ---------------------------------------------------------------------------

def test_crc8_empty_data():
    # CRC8 of empty bytes with init 0 is 0
    assert crc8(b"", 0x00) == 0x00


def test_crc8_known_vector_v1():
    # V1 body bytes (the 7 bytes covered by CRC): F4 01 38 FF 96 00 01
    # init = data_id of 0x100 = 0x06
    # expected final byte = 0x60
    assert crc8(bytes([0xF4, 0x01, 0x38, 0xFF, 0x96, 0x00, 0x01]), 0x06) == 0x60


def test_crc8_known_vector_v4():
    # V4 body bytes: 04 00 00 00 00 00 50
    # init = 0x010 data_id = 0x01
    # expected final byte = 0x1D
    assert crc8(bytes([0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x50]), 0x01) == 0x1D


# ---------------------------------------------------------------------------
# Golden vectors (from docs/PROTOCOL.md §9 + protocol-signals.yaml expect)
# ---------------------------------------------------------------------------

# V1 CHASSIS_VEL_CMD  Vx=500, Vy=-200, wz=150 (=0.150 rad/s), enable=1, counter=0
V1_BYTES = bytes.fromhex("F40138FF96000160")


def test_v1_chassis_vel_cmd_encode():
    cmd = ChassisVelCmd(vx_mm_s=500, vy_mm_s=-200, wz_mrad_s=150, enable=True)
    out = cmd.encode(counter=0)
    assert out == V1_BYTES, f"got {out.hex().upper()}"


def test_v1_chassis_vel_cmd_decode_round_trip():
    cmd = ChassisVelCmd(vx_mm_s=500, vy_mm_s=-200, wz_mrad_s=150, enable=True)
    out = cmd.encode(counter=0)
    parsed = decode_frame(0x100, out)
    assert parsed["vx_mm_s"] == 500
    assert parsed["vy_mm_s"] == -200
    assert parsed["wz_mrad_s"] == 150
    assert parsed["_counter"] == 0
    assert parsed["_flags"] == 0x1  # enable bit


# V2 ARM_JOINT_CMD_A  J1=10.00 deg, J2=-5.00 deg, J3=2.50 deg, enable=1, counter=1
# factor 0.01 deg/unit -> raw: 1000, -500, 250
V2_BYTES = bytes.fromhex("E8030CFEF A001121".replace(" ", ""))


def test_v2_arm_joint_cmd_a_encode():
    out = build_frame(
        0x101,
        dict(j1_pos_deg=1000, j2_pos_deg=-500, j3_pos_deg=250),
        counter=1,
        flags=0x1,
    )
    assert out == V2_BYTES, f"got {out.hex().upper()}"


# V3 CHASSIS_VEL_FB  measured Vx=498, Vy=-203, wz=149, mode=REMOTE(1), counter=12
# mode=REMOTE -> flags low nibble = 0b01; no estop_latched, no fault_active
V3_BYTES = bytes.fromhex("F20135FF9500C152")


def test_v3_chassis_vel_fb_decode():
    fb = ChassisVelFb.decode(V3_BYTES)
    assert fb.vx_mm_s == 498
    assert fb.vy_mm_s == -203
    assert fb.wz_mrad_s == 149
    assert fb.mode == 1  # REMOTE
    assert fb.estop_latched is False
    assert fb.fault_active is False
    # raw frame sanity
    parsed = decode_frame(0x200, V3_BYTES)
    assert parsed["_counter"] == 12


# V4 SAFETY_CMD  CMD=EMERGENCY_STOP(4), counter=5  -> flags=0 (safety uses low nibble=cmd_code)
V4_BYTES = bytes.fromhex("04000000000050" + "1D")


def test_v4_safety_cmd_encode():
    cmd = SafetyCmd(cmd_code=4, counter=5)  # 4 = EMERGENCY_STOP
    out = cmd.encode()
    assert out == V4_BYTES, f"got {out.hex().upper()}"


# V5 XFER_ARM_TO_CH  fault=0x0051, J4=-12.34 deg, BASE=360.00 deg, counter=15, flags=0x9
# factor 0.01 deg/unit -> raw: -1234, 36000; fault packed as u16 in byte 0..1
# signals = fault_code, j4_pos_deg, base_pos_deg
V5_BYTES = bytes.fromhex("5100 2EFB A08C F9BF".replace(" ", ""))


def test_v5_xfer_arm_to_ch_decode():
    parsed = decode_frame(0x031, V5_BYTES)
    assert parsed["fault_code"] == 0x0051
    assert parsed["j4_pos_deg"] == -1234     # -12.34 deg * 100
    assert parsed["base_pos_deg"] == 36000  # 360.00 deg * 100
    assert parsed["_counter"] == 15
    assert parsed["_flags"] == 0x9


# V6 NODE_INFO_CHASSIS  proto=3/hw=1 (packed: hw_rev in byte0, proto_ver in byte1
# -> LE u16 = (proto_ver << 8) | hw_rev = 0x0301), build=0xABCD, counter=0
# signals = proto_hw_packed, build_hash, rsvd16
V6_BYTES = bytes.fromhex("0103CDAB000000" + "5C")


def test_v6_node_info_chassis_decode():
    parsed = decode_frame(0x230, V6_BYTES)
    assert parsed["proto_hw_packed"] == 0x0301   # hw_rev=1 (low), proto_ver=3 (high)
    assert parsed["build_hash"] == 0xABCD
    assert parsed["_counter"] == 0


# ---------------------------------------------------------------------------
# Robustness: CRC rejection, counter wrap, range saturation
# ---------------------------------------------------------------------------

def test_decode_rejects_corrupt_crc():
    cmd = ChassisVelCmd(vx_mm_s=1, vy_mm_s=0, wz_mrad_s=0)
    frame = bytearray(cmd.encode(counter=0))
    frame[7] ^= 0x01  # flip one bit of CRC
    with pytest.raises(ValueError, match="CRC mismatch"):
        decode_frame(0x100, bytes(frame))


def test_counter_wraps_at_16():
    # Counter is 4 bits; sending counter=16 must raise.
    cmd = ChassisVelCmd(vx_mm_s=0, vy_mm_s=0, wz_mrad_s=0)
    with pytest.raises(OverflowError):
        cmd.encode(counter=16)


def test_overflow_int16():
    cmd = ChassisVelCmd(vx_mm_s=40000, vy_mm_s=0, wz_mrad_s=0)  # > int16 max
    with pytest.raises(OverflowError):
        cmd.encode(counter=0)


def test_deg_helper_roundtrip():
    from rak_can_driver.protocol import deg_to_raw, deg_to_raw_u16, raw_to_deg, raw_to_deg_u16
    # signed int16 (joint angles +/-327.67 deg)
    assert deg_to_raw(10.0) == 1000
    assert deg_to_raw(-5.0) == -500
    assert raw_to_deg(1000) == pytest.approx(10.0)
    assert raw_to_deg(-1234) == pytest.approx(-12.34)
    # uint16 (BASE_POS 0..655.35 deg)
    assert deg_to_raw_u16(360.0) == 36000
    assert raw_to_deg_u16(36000) == pytest.approx(360.0)
    # overflow clamps
    assert deg_to_raw(400.0) == 32767   # saturated to int16 max
    assert deg_to_raw(-400.0) == -32768 # saturated to int16 min
    assert deg_to_raw_u16(700.0) == 65535
    assert deg_to_raw_u16(-1.0) == 0
