# Hardware Acceptance Gates

This document separates software simulation from physical acceptance. No hardware
acceptance is claimed by this repository change.

## Known Topology

- Jetson Orin Nano to base STM32F407: USB-UART, 921600 baud, 100 Hz, bidirectional.
- Base STM32F407 to arm STM32F407: classic CAN 2.0A, 1 Mbps, 1 kHz.
- Jetson does not connect directly to the CAN bus. The base MCU owns mecanum
  kinematics and its four 25 kHz wheel loops.
- The repository contains no matching base or arm F407 firmware source.

The proposed host UART format is documented in
[jetson_f407_uart_v1.md](protocol/jetson_f407_uart_v1.md). Existing firmware
compatibility is unknown until the base firmware is updated and its implementation
is checked against the protocol vectors.

## Before Connecting a Physical Base

All items below are prerequisites, not actions performed automatically by ROS:

- The F407 firmware owner confirms the UART version, capability bits, units,
  watchdog lease, sequence handling, fault mapping and reboot behavior.
- The F407 independently enforces command timeout, E-stop, fault lockout and a
  DISARMED state after reset or communication loss. Reconnection must not re-arm.
- Physical E-stop operation and motor-power isolation are tested independently of
  Jetson, ROS, Nav2, collision monitor and network access.
- Base direction, yaw sign, velocity units, feedback timestamps, odometry/IMU
  quality, footprint, acceleration and conservative velocity limits are measured.
- The operator identifies the USB device by stable hardware identity. Do not infer
  device identity from `/dev/ttyUSB0`; lidar adapters may also use serial devices.
- The robot is mechanically supported with wheels clear of the floor for initial
  motor tests. An operator remains at the E-stop with a clear test area.
- A reviewed launch configuration supplies finite positive measured limits. The
  defaults of no port, `allow_hardware=false`, zero limits and DISARMED remain in
  force until an operator deliberately changes them.

The `rak_base_driver` node publishes telemetry and accepts an explicit ARM service
request only after the port, protocol handshake, valid fresh feedback, clear faults
and positive configured limits are present. A launch argument alone is not proof
that the firmware or mechanical system is safe.

## Staged Base Test

After the prerequisites are signed off by the hardware owner:

1. Run the PTY regression first:

   ```bash
   PYTHONPATH=src/rak_base_driver python3 scripts/serial_100hz.py --sec 3
   ```

   This opens a pseudo-terminal only and transmits DISARMED zero commands. It does
   not test the physical board.
2. Start the driver against the confirmed port with hardware access explicitly
   enabled but leave it DISARMED. Confirm the board ID, firmware version, boot ID,
   negotiated capabilities, echo sequence, telemetry freshness and zero faults.
3. Verify that unplugging the UART, stopping host updates, rebooting the F407,
   injecting invalid frames and asserting E-stop each revoke motion permission.
   Verify the MCU stops independently and remains DISARMED after communication
   returns.
4. With wheels still clear, request ARM explicitly through the driver service and
   apply only low, bounded, single-axis commands. Verify forward, lateral and yaw
   signs and compare requested versus measured motion.
5. Exercise software pause/cancel and physical E-stop separately. Collision
   monitor and ROS task cancellation are not substitutes for the physical E-stop
   or MCU watchdog.
6. Only after bench results are reviewed may a supervised floor test use a fenced,
   clear area and the same conservative measured limits.

Do not connect the arm control path as part of base UART acceptance. The base-to-arm
CAN contract and arm firmware must be reviewed separately.

## Arm Unlock Gates

The six-axis URDF, limits, MoveIt configuration and `rak_arm_simulation` action
servers are software fixtures. Their geometry and joint limits are not hardware
data, and the fake action servers report kinematic estimates without physics.

Before any real arm integration, obtain and review:

- measured link transforms, masses, inertias and collision geometry;
- all six joint names, directions, zero positions, hard/soft limits and dynamic limits;
- independent gripper range and firmware command/feedback mapping;
- the F407-to-F407 CAN protocol, command lease, fault/limit reporting and reset behavior;
- a verified safe hold/brake strategy for communication loss and power loss;
- supervised limit, E-stop, collision and loss-of-feedback tests.

Until those are available, keep the arm on the fake action server and do not configure
a real `ros2_control` hardware plugin.

## Evidence Boundary

The following demonstrate software behavior only:

- codec unit tests and the PTY loopback;
- fake Nav2 task lifecycle tests;
- six-axis fixture validation and fake action-server tests;
- successful builds, launch parsing or a MoveIt planning request.

None establish F407 interoperability, measured odometry, collision-stop distance,
physical E-stop response, mechanical safety, or real arm trajectory tracking.
