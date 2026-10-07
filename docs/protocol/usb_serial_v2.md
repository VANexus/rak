# Jetson-F407 USB Serial Protocol v2

Status: normative target contract for the single-F407 topology. The Jetson
codec and standalone F407 frame/payload codec are implemented independently
from the legacy v1 body-twist driver. The F407 v2 endpoint is not integrated
with its UART, control loops or telemetry and has not passed interoperability
tests.
Nothing in this document authorizes physical actuation.

## 1. Ownership and physical link

The Jetson connects to the one STM32F407 over its dedicated USB serial device.
The F407 owns the 25 kHz wheel loops, wheel encoder sampling, servo PWM, actuator
watchdogs, E-stop latch and low-level fault handling. ROS 2 owns body kinematics,
navigation, arm trajectory generation, odometry fusion and task lifecycle.

The target actuator interfaces are four mecanum wheel angular velocities in
`rad/s` and six arm joint positions in `rad`. A seventh position axis is optional
and denotes the independent gripper servo. The gripper value is a calibrated
servo angle in radians, not jaw width. Joint names, signs, zeroes, limits and
PWM mappings are installation calibration, not wire-protocol constants.

The current F407 source has six PWM outputs, no joint position feedback, and a
startup servo sweep. It does not satisfy this v2 contract. Before installing a
physical arm, remove startup motion, implement the negotiated v2 endpoint, add
the seventh PWM channel if required, and pass the physical arm unlock gates.

The USB serial transport is 921600 8N1 where a UART bridge is used. USB CDC
implementations may ignore the configured baud; framing, leases and deadlines
remain mandatory. Only one host process may own the serial device.

## 2. Common frame

Each packet is `COBS(decoded_frame) || 0x00`. The decoded frame is:

| Offset | Field | Encoding |
|---:|---|---|
| 0 | protocol version | `u8`, exactly `2` |
| 1 | message type | `u8` |
| 2 | sequence | `u16 LE` |
| 4 | payload length | `u16 LE`, at most 128 |
| 6 | payload | exactly the declared number of bytes |
| final 2 | CRC | `u16 LE` |

CRC is CRC-16/CCITT-FALSE over version through payload: polynomial `0x1021`,
initial value `0xffff`, no reflection, xorout `0x0000`; `"123456789"` gives
`0x29b1`, serialized `b1 29`. Multibyte fields are little-endian. Unknown
versions, message types, reserved bits, invalid lengths, malformed payloads,
bad CRCs and stale sequences are rejected without refreshing a lease or
partially updating an actuator command.

Each direction has an independent `u16` sequence. A packet is newer when
`(candidate - previous) mod 65536` is in `1..32767`. Duplicates, old packets
and the half-range ambiguity are rejected. A new boot identity resets receive
sequence state. Sequence ordering does not replace command IDs or task
idempotency at the ROS action layer.

## 3. Message types and payloads

All payload lengths are exact. `i32` actuator values use micro-units:
`1 raw = 1e-6 rad/s` for wheel velocity and `1 raw = 1e-6 rad` for position.
Conversions round half away from zero; out-of-range/non-finite host values are
rejected, never wrapped or saturated silently.

| Type | Name | Direction | Rate | Payload |
|---:|---|---|---:|---|
| `0x01` | HELLO | host -> F407 | 1 Hz until accepted, then 1 Hz | 8 bytes |
| `0x02` | HELLO_ACK | F407 -> host | response | 14 bytes |
| `0x03` | TIME_SYNC_REQUEST | host -> F407 | 2 Hz during sync | 8 bytes |
| `0x04` | TIME_SYNC_RESPONSE | F407 -> host | response | 24 bytes |
| `0x10` | DRIVE_COMMAND | host -> F407 | 100 Hz | 20 bytes |
| `0x11` | ARM_COMMAND | host -> F407 | 100 Hz | 29 bytes for 6 axes; 33 for 7 |
| `0x12` | SAFETY_COMMAND | host -> F407 | event | 2 bytes |
| `0x20` | DRIVE_STATE | F407 -> host | 100 Hz | 68 bytes |
| `0x21` | ARM_STATE | F407 -> host | 100 Hz | 66 bytes for 6 axes; 74 for 7 |
| `0x22` | DIAGNOSTICS | F407 -> host | 10 Hz | 36 bytes |
| `0x23` | FAULT_EVENT | F407 -> host | event | 7 bytes |

`HELLO = host_build_id:u32, requested_features:u32`.
`HELLO_ACK = board_id:u16, firmware_version:u32, boot_id:u32,
accepted_features:u32`. Firmware version is `(major<<16)|(minor<<8)|patch`.
`boot_id` changes on every reset. Board ID `0x0407` identifies STM32F407 and
does not claim a specific board revision.

Feature bits:

| Bit | Capability |
|---:|---|
| 0 | four wheel velocity command |
| 1 | six arm position commands |
| 2 | measured wheel encoder odometry |
| 3 | measured arm joint positions |
| 4 | seventh independent gripper servo |
| 5 | device uptime time-sync response |

Bits 0, 1 and 2 are required for v2 operation. Bits 3-5 are optional. A host
must remain disarmed unless required bits are accepted. In particular, v1
firmware and v1 body-twist commands are not a valid fallback for v2.

### 3.1 DRIVE_COMMAND

Layout: `mode:u8, lease_ms:u16, valid_mask:u8, wheel_velocity[4]:i32`.
Wheel order is fixed: front-left, front-right, rear-left, rear-right. Values
are angular velocity in `rad/s` at the wheel joint, matching
`mecanum_drive_controller` interfaces. Bit `i` in `valid_mask` marks that wheel
target valid. ARMED requires mask `0x0f`; DISARMED requires mask zero and four
zero targets.

The F407 converts calibrated angular velocity to its native count rate. It
must apply one common scale to all four targets if any wheel exceeds the
measured wheel limit. Per-wheel clipping is forbidden because it changes the
requested chassis motion direction. The motor ISR remains the sole writer of
PWM and fixed-rate loop state.

### 3.2 ARM_COMMAND

Layout: `mode:u8, lease_ms:u16, axis_count:u8, valid_mask:u8,
position[axis_count]:i32`. Positions are radians. `axis_count` is 6 for the
positioning arm; 7 is accepted only when feature bit 4 was negotiated. Slots
0-5 are the installation's six calibrated arm axes in its signed mapping file;
slot 6 is the independent gripper-servo angle. The wire protocol intentionally
does not assign anatomical names or invent zeroes, signs, limits or pulse
widths.

ARMED requires every declared axis valid. DISARMED requires a zero mask and
zero positions. A valid position command is not evidence of measured position.
`ARM_STATE` reports desired and measured arrays separately.

### 3.3 DRIVE_STATE and ARM_STATE

`DRIVE_STATE` layout:
`echoed_command_sequence:u16, mode:u8, feedback_valid_mask:u8,
fault_bitmap:u32, encoder_epoch:u32, sample_time_us:u64,
wheel_position_urad[4]:i64, wheel_velocity_urad_s[4]:i32`.

Wheel position is signed, cumulative wheel-joint angle since the current
`encoder_epoch`; it is not modulo one revolution. Increment `encoder_epoch`
whenever encoder position is reset or continuity is lost. Wheel position and
velocity come from measured quadrature encoders, never from commands. Valid
mask bits 0-3 correspond to the fixed wheel order. Invalid values are ignored
and their mask bits are clear.

`ARM_STATE` layout:
`echoed_command_sequence:u16, mode:u8, axis_count:u8,
measured_valid_mask:u8, measured_source:u8, fault_bitmap:u32,
sample_time_us:u64, command_position_urad[axis_count]:i32,
measured_position_urad[axis_count]:i32`.

`measured_source` is `0=unavailable`, `1=encoder`, `2=potentiometer`,
`3=other calibrated sensor`. A commanded servo pulse is never measured
feedback. When the current hardware has no position sensor, feature bit 3 is
clear, mask is zero and measured values are zero. MoveIt execution must not
claim measured tracking or success from command echoes alone.

For both state messages, `sample_time_us` is MCU monotonic uptime at acquisition,
not packet assembly or USB transmit time. The outer frame sequence is the
telemetry sequence. Echo sequence identifies the newest command actually
accepted by the MCU; it does not attest physical motion.

### 3.4 Safety, synchronization and diagnostics

`SAFETY_COMMAND = operation:u8, subsystem_mask:u8`. Operations are
`1=STOP`, `2=LATCH_INHIBIT`, `3=CLEAR_INHIBIT`. Subsystem mask is bit 0 drive,
bit 1 arm; zero and reserved bits are invalid. STOP ramps the base toward zero
and places the arm in its configured active hold behavior. LATCH_INHIBIT
revokes command acceptance and remains latched until an explicit clear after
the physical E-stop input is released and faults are cleared. CLEAR_INHIBIT
never arms an actuator. Hardware E-stop and MCU watchdogs remain authoritative
and do not depend on receiving a UART packet.

`TIME_SYNC_REQUEST = host_send_monotonic_ns:u64`.
`TIME_SYNC_RESPONSE = echoed_host_send_monotonic_ns:u64,
device_rx_uptime_us:u64, device_tx_uptime_us:u64`. The host uses repeated
round trips to estimate an offset and uncertainty; it must not assume wall-clock
or ROS time is synchronized to the MCU. This is a clock correlation aid, not a
motion scheduling primitive.

`DIAGNOSTICS = device_uptime_us:u64, battery_mV:u16,
board_temp_centi_C:i16, crc_error_count:u32, sequence_error_count:u32,
rx_overrun_count:u32, tx_drop_count:u32, control_deadline_miss_count:u32,
reset_count:u32`. `board_temp_centi_C=0x7fff` means unavailable.
`FAULT_EVENT = fault_bitmap:u32, source_command_sequence:u16,
subsystem:u8` where subsystem is 1=drive, 2=arm, 3=system.

Fault bits: bit 0 E-stop latched; bit 1 drive lease expired; bit 2 arm lease
expired; bit 3 wheel encoder feedback invalid; bit 4 wheel saturation active;
bit 5 wheel control/motor fault; bit 6 undervoltage; bit 7 arm actuator fault;
bit 8 arm position feedback unavailable; bit 9 clock correlation unavailable;
bit 10 control deadline missed. Bits 11-31 are reserved and transmitted zero.
Unavailable capability (for example no arm encoder) is primarily reported by
the feature bitmap and feedback-valid mask; it is not by itself an actuator
fault.

## 4. Timing, leases and safe state

Every actuator command carries its own lease. Lease range is 10-200 ms;
recommended host setting is 100 ms at a 100 Hz command rate. Firmware enforces
the negotiated range locally using a monotonic clock. The currently measured
200 ms setting is a temporary v1 firmware workaround, not evidence that v2
must use 200 ms. Do not reduce it to 50 ms until UART/USB latency and MCU RX
overruns are measured under motor load. Commands outside the accepted lease
range are rejected and do not refresh the watchdog.

Reset starts DISARMED with zero wheel targets. Arm servo outputs must not sweep
or move automatically at boot. On drive lease expiry, ramp all wheel targets
to zero using the locally measured deceleration limit, then brake/idle as
configured. On arm lease expiry, reject new trajectory motion and actively
hold the last safe position while power is available; do not drop servo power
as a generic stop action. Power loss can still cause mechanical fall and must
be addressed mechanically.

The host starts DISARMED on process start, serial reconnect, boot-ID change,
malformed-frame threshold, stale feedback, nonzero fault or missed command
echo. Recovery never automatically re-arms. Drive and arm enable are separately
requested and separately revoked. A stale or invalid command is not replayed.
UART STOP and software cancellation are not substitutes for the independent
physical E-stop.

All protocol timestamps use MCU monotonic uptime or host monotonic time.
`sample_time_us` is correlated through TIME_SYNC only for fusion and latency
measurement. ROS messages use the original acquisition time converted into
the host ROS clock domain and preserve validity/age; never stamp a delayed
measurement with packet processing time.

## 5. ROS, odometry, IMU and TF contract

The host hardware interface exports four wheel `velocity` command interfaces,
wheel `position` and `velocity` state interfaces, and six arm `position`
command interfaces. The seventh gripper position interface is exported only
when the negotiated feature and installation mapping both exist. Controller
joint ordering must match the fixed wire order, not XML order.

`mecanum_drive_controller` owns host-side inverse kinematics and publishes
wheel-joint odometry. It must set `enable_odom_tf: false`. A host adapter
converts measured wheel state to `joint_state`/controller state; it must not
derive odometry from commanded velocities. `robot_localization` EKF is the
only `odom -> base_link` TF publisher. It fuses measured wheel odometry and
Jetson LSM6DSRTR IMU data; wheel-derived yaw is excluded and lateral wheel
odometry receives conservatively high covariance. `slam_toolbox` alone
publishes `map -> odom`.

The Jetson SPI LSM6DSRTR is the body-heading authority. Its kernel acquisition
timestamp and calibrated axes feed `sensor_msgs/Imu` and the EKF; SPI/IIO
samples do not transit the F407 UART. The F407 onboard IMU is local only.
Any future MCU compensation must remain wheel-level and must not add an
independent body yaw target or fight Nav2's angular command. It is not an EKF
source.

Wheel odometry requires measured encoder deltas, calibrated counts/revolution,
wheel radius, signs and reset epoch. Until those values are measured and the
source publishes valid acquisition timestamps, mask and covariance, do not
feed it into the EKF. Encoder-only wheel odometry is particularly uncertain
in lateral motion. No firmware or protocol default is a substitute for
calibration.

## 6. Validation and rollout

### Golden vector: four-wheel command

Input: sequence `0x1234`, `DRIVE_COMMAND`, ARMED, 100 ms lease, mask `0x0f`,
wheel targets `(1, -2, 0, 0.5) rad/s`.

```text
payload:
01 64 00 0f 40 42 0f 00 80 7b e1 ff 00 00 00 00 20 a1 07 00

complete COBS wire packet (includes trailing 00):
06 02 10 34 12 14 03 01 64 05 0f 40 42 0f 05 80 7b e1 ff
01 01 01 04 20 a1 07 03 d7 65 00
```

Both implementations must reproduce these bytes exactly. The F407 standalone
codec vector is exercised by `tools/uart_v2_codec_test.c`; the Jetson vector is
in `src/rak_base_driver/test/test_protocol_v2.py`.

The v2 implementation must have host and MCU-independent golden-vector tests
for framing, CRC, COBS, every payload, units, masks, sequence wrap, invalid
lengths, reserved bits, stale frames, counter resets, lease expiry, simultaneous
wheel saturation and encoder epoch changes. PTY tests are software-only.

Roll out in stages: codec conformance; firmware build; no-actuation HELLO and
telemetry; wheel encoder read-only validation; wheels-clear low-speed drive;
arm read-only/no-sensor state validation; calibrated arm limits with the
mechanism supported; independent E-stop and loss-of-link tests; then supervised
integrated tests. Keep hardware disabled until these gates pass.

Legacy [UART v1](jetson_f407_uart_v1.md) remains an archived body-twist
contract for existing host code and historical tests. It is not compatible
with v2 and must not be negotiated as an automatic fallback.
