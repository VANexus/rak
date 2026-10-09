import math
import struct

import pytest

from rak_base_driver.protocol import Frame, ProtocolError
from rak_base_driver.protocol_v2 import (
    ArmCommand,
    DriveCommand,
    FEATURE_GRIPPER_SERVO,
    MODE_ARMED,
    MODE_DISARMED,
    REQUIRED_FEATURES,
    SAFETY_LATCH_INHIBIT,
    SUBSYSTEM_ARM,
    SUBSYSTEM_DRIVE,
    TYPE_DRIVE_COMMAND,
    VERSION,
    arm_command_from_si,
    decode_v2_frame,
    drive_command_from_si,
    encode_v2_frame,
    pack_arm_command,
    pack_drive_command,
    pack_hello,
    pack_safety_command,
    pack_time_sync_response,
    si_to_micro,
    unpack_arm_command,
    unpack_arm_state,
    unpack_diagnostics,
    unpack_drive_state,
    unpack_drive_command,
    unpack_hello_ack,
    unpack_time_sync_response,
)


def test_v2_frame_and_drive_golden_vector():
    payload = pack_drive_command(
        DriveCommand(MODE_ARMED, 100, 0x0F, (1_000_000, -2_000_000, 0, 500_000))
    )
    wire = encode_v2_frame(TYPE_DRIVE_COMMAND, 0x1234, payload)
    assert wire.hex(" ") == (
        "06 02 10 34 12 14 03 01 64 05 0f 40 42 0f 05 80 7b e1 ff "
        "01 01 01 04 20 a1 07 03 d7 65 00"
    )
    assert decode_v2_frame(wire) == Frame(VERSION, TYPE_DRIVE_COMMAND, 0x1234, payload)
    assert unpack_drive_command(payload) == DriveCommand(
        MODE_ARMED, 100, 0x0F, (1_000_000, -2_000_000, 0, 500_000)
    )


def test_hello_requires_v2_wheel_arm_and_encoder_capabilities():
    assert pack_hello(0x12345678, REQUIRED_FEATURES) == struct.pack(
        "<II", 0x12345678, REQUIRED_FEATURES
    )
    ack = struct.pack("<HIII", 0x0407, 0x00020000, 12, REQUIRED_FEATURES | 0x38)
    assert unpack_hello_ack(ack) == (0x0407, 0x00020000, 12, REQUIRED_FEATURES | 0x38)
    with pytest.raises(ProtocolError, match="capabilities"):
        unpack_hello_ack(struct.pack("<HIII", 0x0407, 0x00020000, 12, 3))


def test_arm_command_six_and_optional_seventh_axis_round_trip():
    six = ArmCommand(
        MODE_ARMED, 100, 0x3F, (100_000, -200_000, 300_000, -400_000, 500_000, -600_000)
    )
    six_payload = pack_arm_command(six)
    assert six_payload.hex(" ") == (
        "01 64 00 06 3f a0 86 01 00 c0 f2 fc ff e0 93 04 00 "
        "80 e5 f9 ff 20 a1 07 00 40 d8 f6 ff"
    )
    assert unpack_arm_command(six_payload) == six
    seven = arm_command_from_si(
        MODE_ARMED,
        100,
        [0.1, -0.2, 0, 0, 0, 0, 0.03],
        FEATURE_GRIPPER_SERVO,
    )
    assert seven.joint_position_urad == (100_000, -200_000, 0, 0, 0, 0, 30_000)
    with pytest.raises(ProtocolError, match="not negotiated"):
        pack_arm_command(seven)
    payload = pack_arm_command(seven, FEATURE_GRIPPER_SERVO)
    with pytest.raises(ProtocolError, match="not negotiated"):
        unpack_arm_command(payload)
    assert unpack_arm_command(payload, FEATURE_GRIPPER_SERVO) == seven


def test_disarmed_commands_zero_targets_and_masks():
    assert len(pack_drive_command(DriveCommand(MODE_DISARMED, 100, 0, (0, 0, 0, 0)))) == 20
    with pytest.raises(ProtocolError, match="DISARMED"):
        pack_drive_command(DriveCommand(MODE_DISARMED, 100, 0, (1, 0, 0, 0)))
    with pytest.raises(ProtocolError, match="DISARMED"):
        pack_arm_command(ArmCommand(MODE_DISARMED, 100, 0, (0, 0, 0, 0, 0, 1)))


def test_wire_units_rounding_and_finite_range_checks():
    assert si_to_micro(0.0000005, "x") == 1
    assert si_to_micro(-0.0000005, "x") == -1
    assert drive_command_from_si(MODE_ARMED, 100, (1.2, -1.2, 0.0, 0.5)).wheel_velocity_urad_s == (
        1_200_000, -1_200_000, 0, 500_000
    )
    with pytest.raises(ProtocolError, match="finite"):
        si_to_micro(math.inf, "position")
    with pytest.raises(ProtocolError, match="range"):
        si_to_micro(3000.0, "position")


def test_command_payload_validation_rejects_bad_length_masks_and_lease():
    with pytest.raises(ProtocolError, match="20 bytes"):
        unpack_drive_command(bytes(19))
    with pytest.raises(ProtocolError, match="all wheels"):
        pack_drive_command(DriveCommand(MODE_ARMED, 100, 7, (0, 0, 0, 0)))
    with pytest.raises(ProtocolError, match="lease"):
        pack_drive_command(DriveCommand(MODE_ARMED, 201, 15, (0, 0, 0, 0)))
    with pytest.raises(ProtocolError, match="axis count"):
        unpack_arm_command(bytes(29))


def test_safety_command_and_time_sync_round_trip():
    assert pack_safety_command(SAFETY_LATCH_INHIBIT, SUBSYSTEM_DRIVE | SUBSYSTEM_ARM) == b"\x02\x03"
    with pytest.raises(ProtocolError, match="mask"):
        pack_safety_command(SAFETY_LATCH_INHIBIT, 0x80)
    response = (123_456_789_000, 987_654, 987_700)
    assert unpack_time_sync_response(pack_time_sync_response(*response)) == response


def test_measured_state_keeps_sample_time_epoch_and_validity_separate():
    drive_prefix = struct.pack("<HBBIIQ", 17, MODE_ARMED, 0x0F, 0, 4, 123_456)
    drive_payload = drive_prefix + struct.pack(
        "<4q4i", 1, -2, 3, -4, 5, -6, 7, -8
    )
    drive = unpack_drive_state(drive_payload)
    assert drive.echoed_command_sequence == 17
    assert drive.encoder_epoch == 4
    assert drive.sample_time_us == 123_456
    assert drive.wheel_position_urad == (1, -2, 3, -4)
    assert drive.wheel_velocity_urad_s == (5, -6, 7, -8)
    with pytest.raises(ProtocolError, match="reserved"):
        unpack_drive_state(drive_prefix[:3] + b"\x10" + drive_prefix[4:] + bytes(48))

    arm_prefix = struct.pack("<HBBBBIQ", 22, MODE_ARMED, 6, 0, 0, 0, 789)
    arm_payload = arm_prefix + struct.pack("<6i6i", *range(12))
    arm = unpack_arm_state(arm_payload)
    assert arm.measured_valid_mask == 0
    assert arm.measured_source == 0
    assert arm.command_position_urad == tuple(range(6))
    assert arm.measured_position_urad == tuple(range(6, 12))
    with pytest.raises(ProtocolError, match="source"):
        bad_prefix = struct.pack("<HBBBBIQ", 22, MODE_ARMED, 6, 0, 4, 0, 789)
        unpack_arm_state(bad_prefix + bytes(48))


def test_diagnostics_fixed_width_and_signed_temperature():
    payload = struct.pack("<QHhIIIIII", 1000, 11800, -1250, 1, 2, 3, 4, 5, 6)
    diag = unpack_diagnostics(payload)
    assert diag.device_uptime_us == 1000
    assert diag.battery_mv == 11800
    assert diag.board_temp_centi_c == -1250
    assert diag.control_deadline_miss_count == 5
    with pytest.raises(ProtocolError, match="36 bytes"):
        unpack_diagnostics(payload[:-1])
