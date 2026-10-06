# Hardware Acceptance Gates

> ⚠️ **拓扑已变更（2026-10-05），本文部分内容过期。**
> 臂板烧毁后只剩**一块** F407（同时管 4 轮 + 6 舵机），**CAN 链路失效**，
> Jetson 只通过 **USB** 与下位机通信；麦轮 IK 已移到上位机（`mecanum_drive_controller`）。
> 见 [ADR-0007](adr/0007-unified-ros2-control.md)。下文 §Known Topology 的
> "Base STM32F407 to arm STM32F407: classic CAN" 与
> "The base MCU owns mecanum kinematics" 两句已不成立；
> **验收流程与门槛原则仍然有效**，具体接线/设备名以 ADR-0007 与
> [`CONSTRUCTION.md`](CONSTRUCTION.md) §6 为准。
>
> 2026-10-06 协议更新：v1 F407 固件 R16-R18 曾台架运行；目标
> [UART v2](protocol/usb_serial_v2.md) 的 host/MCU codec 有软件 golden-vector
> 测试，但 v2 尚无串口控制 endpoint，以下硬件门槛仍未解除。

This document separates software simulation from physical acceptance. No hardware
acceptance is claimed by this repository change.

## Known Topology

- Jetson Orin Nano to the single base STM32F407: dedicated USB serial,
  921600 8N1 when bridged UART is used.
- The target v2 command link carries four wheel angular velocities and six arm
  positions, with optional seventh gripper servo; the F407 owns the wheel loops
  and servo PWM. Host `mecanum_drive_controller` owns inverse kinematics.
- The F407 R16-R18 source implements legacy v1 body velocity and has bench
  evidence. v2 frame/payload codecs exist in both repositories, but the v2
  endpoint is not integrated and is not interoperable yet.
- CAN v3 was for the former two-board topology. It is dormant and not part of
  the current Jetson-F407 control link.

The target host UART format is documented in
[usb_serial_v2.md](protocol/usb_serial_v2.md). The older
[jetson_f407_uart_v1.md](protocol/jetson_f407_uart_v1.md) is retained for the
bench-tested legacy firmware only.

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
- The current bringup launch uses `/dev/wheel_motor` as its port argument, but this
  is only a configured alias, not evidence that the connected device is the F407.
  `hardware_enable` remains false by default, so the driver does not open the port.
- The robot is mechanically supported with wheels clear of the floor for initial
  motor tests. An operator remains at the E-stop with a clear test area.
- A reviewed launch configuration supplies finite positive measured limits. Direct
  `BaseUartNode` defaults are no port, `allow_hardware=false`, zero limits and
  DISARMED. The bringup launch supplies `/dev/wheel_motor` but still defaults to
  `hardware_enable=false` and zero limits. The driver requires all three limits to
  be finite and positive before ARM; zero is not pass-through.

The `rak_base_driver` node publishes telemetry and accepts an explicit ARM service
request only after the port, protocol handshake, valid fresh feedback, clear faults
and positive configured limits are present. A launch argument alone is not proof
that the firmware or mechanical system is safe.

The top-level bringup contains a `motion_backend:=hardware` launch route. It has
passed installed launch parsing and backend-mapping checks, but has not been run
with Nav2 sensors or an F407. It does not provide F407 firmware. The navigation
launch defaults to `motion_backend:=sim`; keep that default for software validation
until the hardware owner has completed the prerequisites above.

## Staged Base Test

After the prerequisites are signed off by the hardware owner:

1. Run the UART v1 PTY regression first:

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

`scripts/serial_link_test.py` is a refusal-only compatibility stub for the retired
CRC-8 protocol; keep it disabled. Use only the documented PTY-only UART v1 tool
until the F407 protocol has been reviewed.

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
