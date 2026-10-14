# Hardware Acceptance Gates

> ⚠️ **拓扑与链路已变更（2026-10-05 / 2026-10-14），本文部分内容过期。**
> 臂板烧毁后只剩**一块** F407（同时管 4 轮 + 6 舵机）；**Jetson↔F407 走 CAN v4
> （SocketCAN `can0`）唯一链路**，USB v2 串口链路已按
> [ADR-0016](adr/0016-retire-usb-transport.md) 删除；麦轮 IK 已移到上位机
> （`mecanum_drive_controller`）。
> 见 [ADR-0007](adr/0007-unified-ros2-control.md)。下文 §Known Topology 的
> "Base STM32F407 to arm STM32F407: classic CAN" 与
> "The base MCU owns mecanum kinematics" 两句已不成立；
> **验收流程与门槛原则仍然有效**，具体接口名以 ADR-0007/0012/0016 与
> [`CONSTRUCTION.md`](CONSTRUCTION.md) §6 为准。
>
> 2026-10-13 链路验收：H/L 接线对调后 `scripts/can_link_smoke.py can0` 连续两次
> **10/10 通过**（BAT≈12500 mV、FAULT=0、BUSOFF=0）——链路冒烟已过，以下整车
> 硬件门槛仍未解除。

This document separates software simulation from physical acceptance. No hardware
acceptance is claimed by this repository change.

## Known Topology

- Jetson Orin Nano to the single base STM32F407: **CAN v4 over SocketCAN** (`can0`,
  classic CAN 2.0A at 1 Mbps). The host implementation is `CanSession` in
  `src/rak_chassis`; the protocol source of truth is the leaf repo's
  `vehical_rak_chassis/docs/PROTOCOL.md` (v4).
- The link carries four wheel angular velocity commands plus telemetry (battery,
  fault, mode, bus-off counters); the F407 owns the wheel loops and servo PWM.
  Host `mecanum_drive_controller` owns inverse kinematics. Arm position commands
  over CAN are not implemented yet (ADR-0012/0016 closed items).
- The USB v2 serial link, `docs/protocol/` and the direct-connect serial tools were
  deleted by ADR-0016 (snapshot in git tag `pre-usb-removal-20261013`).
- CAN v3 was for the former two-board topology; it is retired (ADR-0012).

## Before Connecting a Physical Base

All items below are prerequisites, not actions performed automatically by ROS:

- The F407 firmware owner confirms the protocol version (v4), capability bits, units,
  watchdog lease, sequence handling, fault mapping and reboot behavior.
- The F407 independently enforces command timeout, E-stop, fault lockout and a
  DISARMED state after reset or communication loss. Reconnection must not re-arm.
- Physical E-stop operation and motor-power isolation are tested independently of
  Jetson, ROS, Nav2, collision monitor and network access.
- Base direction, yaw sign, velocity units, feedback timestamps, odometry/IMU
  quality, footprint, acceleration and conservative velocity limits are measured.
- The operator confirms the CAN interface by stable identity: `can0` up at 1 Mbps
  (`ip -details -statistics link show can0`, no error frames / bus-off counters).
  Lidar adapters still use serial devices (`/dev/ttyUSB0` …); do not confuse them
  with the base link.
- `chassis_bringup.launch.py` starts `controller_manager` against `can_channel`
  (default `can0`). Motion stays gated: `allow_motion` defaults to **false**, so
  the hardware only handshakes and reports telemetry and sends zero targets.
  A launch argument alone is not evidence that the firmware or mechanical system
  is safe.
- The robot is mechanically supported with wheels clear of the floor for initial
  motor tests. An operator remains at the E-stop with a clear test area.
- A reviewed launch configuration supplies conservative measured limits
  (`chassis_controllers.yaml`; command-interface limits are enforced by
  `controller_manager`). The placeholder wheel geometry must be replaced with
  measured values before any speed increase.

The `rak_chassis` hardware interface reports telemetry and commands the wheels only
through `controller_manager` after the link is up and motion is explicitly allowed;
disallowed motion keeps the controller at zero targets. A launch argument alone is
not proof that the firmware or mechanical system is safe.

`task_navigation.launch.py` offers only `motion_backend:=sim|stub` (ADR-0016
removed the `hardware` backend); it does not connect a device. The real chassis is
reached through `chassis_bringup.launch.py` → `mecanum_drive_controller` → CAN.
Keep `motion_backend:=sim` for software validation until the hardware owner has
completed the prerequisites above.

## Staged Base Test

After the prerequisites are signed off by the hardware owner:

1. Run the CAN link smoke test first (protocol layer; it never sends a non-zero
   drive command, so the wheels do not turn):

   ```bash
   python3 scripts/can_link_smoke.py can0
   ```

   Qualified = **10/10 checks pass** (battery ≈12500 mV, FAULT=0, BUSOFF=0).
   Then check the physical and driver layers:

   ```bash
   ip -details -statistics link show can0   # state, bitrate, error counters
   dmesg | grep -i can                      # controller/bring-up messages
   ```
2. Start `chassis_bringup.launch.py` against the confirmed `can_channel` with
   `allow_motion:=false`. Confirm fresh telemetry and zero faults (battery voltage,
   FAULT=0, BUSOFF=0) before any motion is allowed.
3. Verify that disconnecting the CAN link, stopping host updates, rebooting the F407,
   injecting invalid frames and asserting E-stop each revoke motion permission.
   Verify the MCU stops independently and remains DISARMED after communication
   returns.
4. With wheels still clear, explicitly allow motion (`allow_motion:=true`) and
   apply only low, bounded, single-axis commands. Verify forward, lateral and yaw
   signs and compare requested versus measured motion.
5. Exercise software pause/cancel and physical E-stop separately. Collision
   monitor and ROS task cancellation are not substitutes for the physical E-stop
   or MCU watchdog.
6. Only after bench results are reviewed may a supervised floor test use a fenced,
   clear area and the same conservative measured limits.

### Bench evidence 2026-10-14 (wheels clear)

Recorded so the next person does not re-derive it:

- CAN link smoke `scripts/can_link_smoke.py can0`: **10/10** (FAULT=0x0000, BUSOFF=0,
  battery ≈12136 mV). F407 armed to `MODE=1(REMOTE)` with `remote_gate=OPEN`.
- Wire-level axis map measured by `scripts/axis_probe.py` (steady-state medians). It
  matches the host IK exactly, so **the host side is not the problem**:

  | axis | `DRIVE_CMD` pattern |
  | --- | --- |
  | `vx+` | `[++++]` |
  | `vy+` | `[-++-]` |
  | `wz+` | `[-+-+]` |

  Magnitude checks out too: `0.60 / 0.0325 = 2.77 rad/s`.
- **Left-side polarity inversion found and corrected.** `vx+` → `[++++]` drove the left
  pair backward and the right pair forward, so the base spun in place. Corrected through
  the `wheel_sign` hardware parameter (URDF default `-1 1 -1 1`); A/B verified by hand —
  the wrong value `1 -1 1 -1` makes `W` drive the base backward. The full symptom
  signature and the reason it cannot be an ABCD port permutation are in the AGENTS.md
  gotcha. Re-calibrate after any wheel, motor or encoder change.
- Feedback path is consistent end to end: commanding `vx=+0.05` produced a host peak of
  1.54 rad/s and `WHEEL_SPD` reported FL +0.92 / FR +0.83 rad/s with the wheels visibly
  turning.

**Still open for this gate:** after the polarity fix, physically re-verify all six
directions (`W S A D Q E`) on the bench, then the failure-revocation list in step 3
(CAN disconnect, host update stop, F407 reboot, invalid frames, E-stop). A wheel spin or
a software pause/resume is not acceptance — see the Evidence Boundary below.

Do not connect the arm control path as part of base link acceptance. The base-to-arm
CAN contract and arm firmware must be reviewed separately (the CAN-side 7-axis
command interface does not exist yet, ADR-0012/0016 closed items).

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

- protocol codec unit tests and the CAN link smoke test (`scripts/can_link_smoke.py`);
- fake Nav2 task lifecycle tests;
- six-axis fixture validation and fake action-server tests;
- successful builds, launch parsing or a MoveIt planning request.

None establish F407 interoperability, measured odometry, collision-stop distance,
physical E-stop response, mechanical safety, or real arm trajectory tracking.
