import math
import struct

import pytest

from rak_base_driver.protocol import (
    FEATURE_BASE_TELEMETRY,
    FEATURE_BASE_VELOCITY,
    MODE_ARMED,
    MODE_DISARMED,
    TYPE_HELLO,
    BaseState,
    Frame,
    ProtocolError,
    StreamDecoder,
    cobs_decode,
    cobs_encode,
    crc16_ccitt_false,
    decode_frame,
    encode_frame,
    encode_raw_frame,
    hello_ack_compatible,
    pack_base_state,
    pack_velocity,
    sequence_is_newer,
    twist_to_wire,
    unpack_base_state,
)


def test_crc_check_value_and_complete_frame_vector():
    assert crc16_ccitt_false(b"123456789") == 0x29B1
    frame = Frame(1, TYPE_HELLO, 0x1234, bytes.fromhex("0001000203040506"))
    assert encode_raw_frame(frame).hex(" ") == (
        "01 01 34 12 08 00 00 01 00 02 03 04 05 06 44 58"
    )
    assert encode_frame(frame).hex(" ") == (
        "06 01 01 34 12 08 01 02 01 08 02 03 04 05 06 44 58 00"
    )
    assert decode_frame(encode_frame(frame)) == frame


@pytest.mark.parametrize(
    "payload",
    [
        b"",
        b"\x00",
        bytes(range(1, 255)),
        bytes([7]) * 600,
        bytes(range(256)) * 2,
    ],
)
def test_cobs_round_trip(payload):
    assert cobs_decode(cobs_encode(payload)) == payload


def test_stream_decoder_recovers_after_bad_and_partial_packets():
    good = encode_frame(Frame(1, TYPE_HELLO, 9, b"payload"))
    bad = bytearray(good)
    bad[-3] ^= 0x80
    decoder = StreamDecoder()
    assert decoder.feed(b"noise" + bytes(bad)) == []
    assert decoder.errors == 1
    assert decoder.feed(good[:5]) == []
    assert decoder.feed(good[5:]) == [Frame(1, TYPE_HELLO, 9, b"payload")]


def test_decoder_rejects_bad_length_crc_and_version():
    valid_raw = encode_raw_frame(Frame(1, TYPE_HELLO, 1, b"a"))
    from rak_base_driver.protocol import decode_raw_frame

    with pytest.raises(ProtocolError, match="length"):
        decode_raw_frame(valid_raw[:4] + b"\x02\x00" + valid_raw[6:])
    corrupted = bytearray(valid_raw)
    corrupted[-1] ^= 1
    with pytest.raises(ProtocolError, match="CRC"):
        decode_frame(cobs_encode(bytes(corrupted)))
    wrong_version = bytearray(valid_raw)
    wrong_version[0] = 2
    with pytest.raises(ProtocolError, match="version"):
        decode_raw_frame(bytes(wrong_version))


def test_sequence_window_handles_wrap_and_old_packets():
    assert sequence_is_newer(0, 0xFFFF)
    assert sequence_is_newer(0x7FFF, 0)
    assert not sequence_is_newer(0x8000, 0)
    assert not sequence_is_newer(0xFFFF, 0)
    assert not sequence_is_newer(14, 14)


def test_velocity_units_and_disarmed_invariant():
    assert twist_to_wire(0.12, -0.2, 0.75) == (120, -200, 750)
    assert pack_velocity(MODE_ARMED, 50, 120, -200, 750) == (
        b"\x01\x32\x00\x78\x00\x00\x00\x38\xff\xff\xff\xee\x02\x00\x00"
    )
    assert pack_velocity(MODE_DISARMED, 50, 0, 0, 0)[0] == MODE_DISARMED
    with pytest.raises(ProtocolError, match="DISARMED"):
        pack_velocity(MODE_DISARMED, 50, 1, 0, 0)
    with pytest.raises(ProtocolError, match="lease"):
        pack_velocity(MODE_ARMED, 51, 0, 0, 0)
    with pytest.raises(ProtocolError, match="non-finite"):
        twist_to_wire(math.nan, 0, 0)


def test_telemetry_payload_round_trip_and_handshake_gate():
    telemetry = BaseState(0x1234, 1, 0, 10, -20, 30, 0x01020304)
    assert unpack_base_state(pack_base_state(telemetry)) == telemetry
    features = FEATURE_BASE_VELOCITY | FEATURE_BASE_TELEMETRY
    payload = struct.pack("<HIII", 1, 3, 2, features)
    assert hello_ack_compatible(payload) == (1, 3, 2, features)
    with pytest.raises(ProtocolError, match="capabilities"):
        hello_ack_compatible(payload[:-4] + struct.pack("<I", FEATURE_BASE_VELOCITY))
