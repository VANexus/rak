# Jetson to Base F407 UART Protocol v1

Status: host-side protocol proposal for coordinated F407 implementation. This
document is not evidence that installed firmware implements v1. Until the base
firmware reports a compatible version and capabilities, the physical driver
must remain DISARMED.

## Ownership and physical link

The Jetson connects to the **base F407 only** over USB-UART at 921600 baud,
8N1. Commands and telemetry are scheduled at 100 Hz. The base F407 owns
mecanum inverse kinematics and the four 25 kHz wheel loops. The separate base
and arm F407 boards communicate over classic CAN 2.0A at 1 Mbps / 1 kHz; the
Jetson is not a node on that CAN bus. Arm and gripper commands are not defined
by this protocol.

The link carries desired body velocity in ROS `base_link` axes: +X forward,
+Y left, and +Z yaw according to REP-103. Wire velocity uses millimetres per
second for X/Y and milliradians per second for yaw.

## Frame format

Each wire packet is `COBS(decoded_frame) || 0x00`. COBS encoding excludes the
terminating zero. The decoded frame is:

| Offset | Field | Encoding |
|---:|---|---|
| 0 | protocol version | `u8`, exactly `1` |
| 1 | message type | `u8` |
| 2 | sequence | `u16` little-endian |
| 4 | payload length | `u16` little-endian, at most 128 |
| 6 | payload | exactly `payload_length` bytes |
| last 2 | CRC | `u16` little-endian |

CRC-16/CCITT-FALSE covers the version through the final payload byte:
polynomial `0x1021`, initial value `0xffff`, no reflection, xorout `0x0000`.
The check value for ASCII `123456789` is `0x29b1` (wire bytes `b1 29`).
Receivers reject unknown versions, unsupported message types, bad CRC, invalid
lengths and malformed payloads. Invalid packets do not refresh a command lease.

Sequence numbers are independent in each direction. A receiver accepts a new
sequence only when `(candidate - previous) mod 65536` is in `1..32767`.
Duplicates, old packets and the ambiguous half-range are rejected. Sequence
state is reset only after a new HELLO/boot identity is established.

## Message types

All multibyte integers are little-endian. Payload lengths are exact.

| ID | Name | Payload |
|---:|---|---|
| `0x01` | `HELLO` | `host_build_id:u32`, `requested_features:u32` (8 bytes) |
| `0x02` | `HELLO_ACK` | `board_id:u16`, `firmware_version:u32`, `boot_id:u32`, `accepted_features:u32` (14 bytes) |
| `0x10` | `SET_BASE_VELOCITY` | `mode:u8`, `lease_ms:u16`, `vx_mm_s:i32`, `vy_mm_s:i32`, `wz_mrad_s:i32` (15 bytes) |
| `0x11` | `BASE_STATE` | `echoed_command_sequence:u16`, `state:u8`, `fault_bitmap:u32`, `vx_mm_s:i32`, `vy_mm_s:i32`, `wz_mrad_s:i32`, `device_uptime_ms:u32` (23 bytes) |
| `0x12` | `FAULT_EVENT` | `fault_bitmap:u32`, `source_sequence:u16` (6 bytes) |

`firmware_version` is `(major << 16) | (minor << 8) | patch`. Feature bit 0 is
base velocity; bit 1 is base telemetry. Both are required. Other feature bits
are reserved and ignored by v1.

`mode` is `0=DISARMED`, `1=ARMED`. The host sends `lease_ms=50` at 100 Hz.
The F407 accepts leases only in `[10, 50]` ms and must stop accepting the last
velocity command when its lease expires. An expired lease transitions the
device to DISARMED and latches/reports the timeout fault. A later valid packet
does not re-arm the base; a fresh explicit operator ARM request is required.
DISARMED velocity fields must be zero.

`BASE_STATE.state` is `0=BOOTING`, `1=DISARMED`, `2=ARMED`, `3=FAULT`.
Fault bits are:

| Bit | Meaning |
|---:|---|
| 0 | emergency-stop input active |
| 1 | command lease timeout |
| 2 | downstream CAN peer offline |
| 3 | downstream peer reports fault |
| 4 | motor driver fault |
| 5 | undervoltage |
| 6 | over-temperature |
| 7 | invalid feedback |

Bits 8..31 are reserved and must be zero. Board and firmware owners must
confirm these assignments and the required physical safe-state behavior before
implementing them in firmware.

## Golden vectors

CRC check vector:

```text
input ASCII: 31 32 33 34 35 36 37 38 39
CRC value:   0x29b1
CRC bytes:   b1 29
```

COBS/frame vector (HELLO, sequence `0x1234`, payload
`00 01 00 02 03 04 05 06`):

```text
decoded without CRC:
01 01 34 12 08 00 00 01 00 02 03 04 05 06

decoded with CRC:
01 01 34 12 08 00 00 01 00 02 03 04 05 06 44 58

wire:
06 01 01 34 12 08 01 02 01 08 02 03 04 05 06 44 58 00
```
The decoded CRC is `0x5844` and is serialized `44 58`. The wire vector is the
COBS encoding followed by the `00` delimiter.

## Host safety state machine

1. Process startup and every serial reconnect begin DISARMED, with zero desired
   velocity and no remembered ARM request.
2. The host may send HELLO while DISARMED. It sends no ARM command until a
   well-formed HELLO_ACK reports v1 plus both required features.
3. A changed `boot_id` invalidates the handshake and sequence history. The host
   returns to DISARMED and repeats HELLO.
4. ARM is accepted only after explicit operator service request, a compatible
   handshake, a current healthy BASE_STATE echo, no fault bits, and
   `allow_hardware=true`. The launch default is false.
5. The host transmits a newly sequenced command every 10 ms. If no fresh safe
   Twist arrives within 100 ms, it sends zero and DISARMS. Non-finite,
   out-of-range, stale or malformed commands are never forwarded.
6. Missing BASE_STATE for 100 ms after handshake, an echo that does not advance
   within the same timeout, a nonzero fault, invalid state, serial disconnect,
   malformed frames beyond the configured error threshold, or an uptime/boot
   reset revokes the host ARM latch and writes DISARMED zero when the port is
   still available.
7. Communication recovery never automatically re-arms. The operator must
   inspect status and issue a new explicit ARM request.

The UART lease watchdog is implemented by the F407, not by ROS. Nav2 collision
monitor, task pause/cancel, and UART loss behavior are separate defenses and
none replaces the physical emergency stop.

## Validation and deployment gate

Host codec tests and pseudo-terminal tests do not operate physical devices.
Before a real base can be connected, the F407 firmware must independently
implement and pass the golden vectors, version/capability handshake, sequence
rules, 50 ms lease watchdog, DISARMED reset behavior, state echo, faults, and
physical E-stop. Confirm the USB device identity and verify the robot is
physically restrained / wheels clear before any supervised bench test.

The current repository does not contain F407 firmware. Consequently this
specification and driver cannot be represented as hardware-validated or
compatible with the older experimental eight-byte CRC-8 format.
