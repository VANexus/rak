"""Jetson host endpoint for CAN v3 chassis + arm protocol.

See vehical_rak_chassis/docs/PROTOCOL.md and protocol-signals.yaml for the
authoritative spec; the encode/decode helpers here are ported from
vehical_rak_chassis/tools/can_check.py and share its golden vectors V1..V6.
"""
from .protocol import (
    MSG_BY_ID,
    PROTOCOL_VERSION,
    ChassisVelCmd,
    ChassisVelFb,
    ChassisDiag,
    SafetyCmd,
    build_frame,
    crc8,
    decode_frame,
)

__all__ = [
    "PROTOCOL_VERSION",
    "MSG_BY_ID",
    "ChassisVelCmd",
    "ChassisVelFb",
    "ChassisDiag",
    "SafetyCmd",
    "crc8",
    "build_frame",
    "decode_frame",
]
