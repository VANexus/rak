# CLAUDE.md

> **CAN 协议已于 2026-10-29 换成 v5 = CANopen（下位机已生效）** —— 见 ADR-0021 与伞形仓
> `docs/CANOPEN-V5.md`。本仓 `src/rak_chassis` 的 `CanSession`/`can_decode` 仍说 **v4 帧**，
> 对当前在板固件（底盘 R73 / 机械臂 A14）**完全无效**：板子停在 Pre-operational，
> 不发 NMT Start 就没有任何 PDO。真机链路要等 P3（lely-core 主站）。
> 现在验证下位机只能用 `tools/canopen_bringup.py`（原始 SocketCAN），**不要**用 v4 脚本
> 或手搓 `cansend` 去判断"下位机坏了"。**禁止同一总线混跑 v4/v5**（ID 冲突）。


This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Orientation

Layered embodied-robot workspace for Jetson Orin Nano / ROS 2 Jazzy: perception + decision on
the Orin, real-time control on **two** STM32F407 leaf nodes (chassis + arm; see the topology
changes below — the "one F407" phase is history again).
Read these before changing behavior — they record decisions the code does not show:

- `README.md` — engineering entry point (Chinese): layout, build, launch entries, hardware.
- `docs/architecture.md` — package boundaries, target runtime architecture, acceptance stages,
  and an explicit list of gaps that are **not** closed.
- `docs/project_status.md` — what is actually done vs. only claimed (dated; read the date).
- `docs/CONSTRUCTION.md` — assembly order and deployment checklist; host-side board changes.
- `docs/adr/` — **immutable** architecture decisions (why, and what was rejected). Index in
  `docs/adr/README.md`. `ADR-0007` is the current control architecture and **supersedes ADR-0001**.
  `ADR-0016` (2026-10-14) removed the USB v2 link — CAN v4 is the only transport, and tools
  talk to drivers only through topic/action/service. `ADR-0010` defines the Jev↔Harness
  arbitration seam and the reserved "呼吸感" expression
  interface; it is **design-only** — `src/rak_harness` / `harness_node` do not exist yet.
- `docs/hardware_acceptance.md` — gates that must pass before any physical base/arm test.
- `docs/brain_arm_tools_design.md` — **DESIGN DRAFT** (2026-10-15, not accepted): tool-proposal
  contract, pinocchio-IK arm chain, tool menu, agentic-loop consumption (no CLI/MCP external
  control surface), phasing P0–P5. Needs an ADR before implementation.
- `docs/migration.md` — the 2026-10-04 unified `rak_` package rename (old name → new name table).
- `TODO.md` — prioritized roadmap (P0–P3). Tasks only; rationale lives in the ADRs.
- `AGENTS.md` — parallel agent-instruction file. It overlaps this one but is **not** identical;
  keep both in sync when you change repository-wide guidance.

**Trust hierarchy when docs disagree.** `AGENTS.md` is the current lead on topology (updated
2026-10-28/29); the sections below keep the historical reasoning. The 2026-10-04 docs
(`README.md`, `docs/architecture.md`, `docs/project_status.md`) describe the code as it stood
then; the 2026-10-05 docs (`TODO.md`, `docs/CONSTRUCTION.md`, `docs/adr/0007`) describe the
**accepted target** after the first hardware change. Do not treat an old status line as current.

## Topology changes (most important context)

**2026-10-28 — two boards again (current).** The arm board was replaced/restored, so the
2026-10-05 "one F407" phase is over. Now: **chassis board** (`vehical_rak_motion`, fw `R64`)
= 4 wheels + IMU; **arm board** (`vehical_rak_edge_ai`, fw `A2`) = **J1/J2 encoder motors**
(AT8236 + TIM6 25 kHz position loop, software zeroing) + J3..J6 servos. The Jetson wire
contract is unchanged (only the board that answers `0x110/0x111`/`0x210/0x211` moved); the
arm board adds additive `0x031`/`0x221` frames, which `rak_chassis` now subscribes to.
Protocol stays v4. `R64`/`A2` have never been flashed; J1/J2 polarity/gains are `[AI]`.

**2026-10-05 — the "one F407" phase (now superseded, kept for reasoning):**

| | Before (ADR-0001, 2026-10-04) | Now (ADR-0007, 2026-10-05) |
|---|---|---|
| MCUs | **two** F407 (base + arm), CAN-interconnected | **one** F407, drives 4 wheels **and** 7 servos (6 arm joints + 1 gripper) |
| Jetson↔base | UART 921600 | **2026-10-05: USB only → 2026-10-09 (ADR-0012): CAN v4 主链路 → 2026-10-14 (ADR-0016): USB v2 链路删除，CAN v4 是唯一传输**（SocketCAN `can0`，`Session` 抽象后上层无感） |
| CAN | base↔arm classic 2.0A 1 Mbps | 2026-10-05 死（对端消失）→ **2026-10-09 恢复为唯一主链路**（协议 v4，事实源在下位仓 `PROTOCOL.md`） |
| Mecanum IK | on the F407 | **moves to the host** (`mecanum_drive_controller`) |
| Heading authority | F407 onboard IMU | **Jetson's LSM6DSRTR (SPI)** feeds the EKF; F407 IMU is wheel-level only |

**Accepted target (ADR-0007, partially built as of 2026-10-08):** the whole robot goes
`ros2_control` — a `hardware_interface::SystemInterface` plugin (named `rak_chassis` since
ADR-0013, ex `rak_hardware`) exposing 4 wheel velocity commands + 7 servo position commands
(6 arm joints + 1 gripper), with `mecanum_drive_controller` and `joint_trajectory_controller`.
**`src/rak_chassis` exists and exports 4 wheel velocity interfaces plus 7 arm position
command interfaces (6 joints + gripper; contract `docs/arm_moveit_contract.md` §1, added
2026-10-14)** (chassis chain smoke-tested against the F407, firmware R37); the arm side
spawns `arm_controller` (JointTrajectoryController) + `gripper_controller` from
`chassis_bringup.launch.py` and was verified on the live F407 with `allow_motion:=false`
(feedback sync, controllers active, action path + `arm_tui` end-to-end) and then with
`allow_motion:=true` for a real j1 jog + Home round-trip (frame-level verified on can0:
event-driven `ARM_CMD` ramps at 0.1° quantization, no faults); J2–J6 motion, limits and
estop-under-motion remain unverified. The 7 arm joints' **real geometry** is defined **once** in
`src/rak_description/urdf/macros/arm_chain.xacro` (a single `<xacro:rak_arm_chain/>` macro,
2026-10-16) and shared by **both** `rak_arm.urdf.xacro` (MoveIt side) and
`rak_chassis.urdf.xacro` (the real robot's `robot_state_publisher` — this is what makes
`shoulder_link…tool0` TF exist at all; the former "zero-length control stub chain" is gone).
The gripper has no geometry and no physical feedback channel; a bare `gripper_link` frame
after `tool0` exists only because Jazzy requires every `<ros2_control>` joint to be in the
kinematic tree. `src/rak_chassis/test/validate_chassis_urdf.py` pins that 11-joint contract
under `colcon test`. Its transport seam is the `Session` abstraction:
`CanSession` (SocketCAN CAN v4) is the **only** implementation — ADR-0016 (2026-10-14)
deleted the USB v2 link, its `SerialSession`, `rak_base_driver` and `rak_serial_link`, the
direct-connect commissioning TUIs, the serial scripts and `docs/protocol/` (snapshot: git
tag `pre-usb-removal-20261013`). The CAN v3 host stack was retired earlier
(ADR-0012/0013, `attic/` deleted 2026-10-10) — it lives only in git history; do not build
on it. Tools may reach the hardware
only through the driver node (topic/action/service), never by opening a device themselves.

**ADR-0011 (accepted 2026-10-08)** adds the whole-body motion-arbitration layer on top of that
target: `src/rak_wbc` (ament_python, 100 Hz velocity-level HQP with Pinocchio + OSQP) — a
standalone node that is the **single regular motion writer**: it publishes `TwistStamped` into
`mecanum_drive_controller`'s `reference` and the arm position stream, taking Nav2's `/cmd_vel`
as its P2 input and `/wbc/ee_target` as P1. Harness proposals reach it only transitively
(executor → skills → `/wbc/*` goals); HQP never reads `/brain/decision`. Skeleton + desktop
numeric tests exist; hardware gates (arm interfaces, measured masses/geometry) are in the
ADR's closed items.

Two facts that are easy to get wrong:
- **Wheel-level saturation belongs to the MCU.** `mecanum_drive_controller` only clamps at the
  body-axis level (`linear.x/y`, `angular.z`) and does *no* per-wheel saturation. The MCU must
  scale all four wheel speeds by one factor when any exceeds the limit — never clamp axis-by-axis,
  or diagonal driving becomes straight driving (ADR-0007 D6).
- **`enable_odom_tf: false` and an explicit `reference_timeout`** are required once the controller
  exists, or it fights the EKF for `odom→base_footprint` (ADR-0002/0007 D4).

## Layout

`colcon build` runs from the repo root and picks up `src/` (ADR-0013: everything buildable
lives there; the 2026-10-09 regroup mapped 23 packages → 18 buildable + 2 archived, see
`docs/migration.md` for the full dated table). As of 2026-10-29 `check_workspace.py`'s
`EXPECTED` holds **22** owned packages (v3 `rak_can_driver` removed; `rak_harness` and
`rak_tool_executor` added upstream since the regroup).

| Package | Contents |
|---|---|
| `src/rak_interfaces` | Messages/actions/services only (`TrackedObject(s)`, `Action`, `Decision`, `ExecuteTask`, `TaskControl`, `TaskStatus`, `BaseTelemetry`, `SendAtomicAction`). No GPU/model deps |
| `src/rak_gpu` | CUDA / TensorRT / VPI RAII utilities. JetPack-only. Export target `rak_gpu::gpu` |
| `src/rak_stereo_driver` | V4L2 + nvJPEG stereo capture. Component `rak::stereo_driver::CaptureNode`, exec `stereo_node` |
| `src/rak_astra_driver` | Orbbec Astra depth/color bridge (OrbbecSDK 1.10.18). Exec `astra_node` |
| `src/rak_pointcloud` | GPU point cloud node (`gpu_pointcloud_node`) |
| `src/rak_description` | URDF/xacro, meshes, RViz. Legacy five-axis demo **plus** an isolated six-axis sim fixture |
| `src/rak_task_executor` | **Sole owner** of the navigation-task lifecycle: `/task/execute` Action, `/task/control` Service, `/task/status`; execs `task_executor`, `task_cli`, `fake_navigation`. Depends on `rak_intent`, never on `rak_agent` |
| `src/rak_intent` | Shared versioned text→intent contract (stdlib-only at import; optional RAI planner lazy-imports). Consumed by task executor + speech |
| `src/rak_wbc` | Velocity-level whole-body arbitration (ADR-0011): 100 Hz HQP (Pinocchio + OSQP), single regular motion writer; skeleton + desktop numeric tests, hardware gated |
| `src/rak_chassis` | (ex `rak_hardware`) F407 transport + ros2_control plugin: `Session` abstraction with `CanSession` (SocketCAN CAN v4, the only transport since ADR-0016); 4 wheel velocity + 7 arm position interfaces (arm since 2026-10-14, real motion still unverified) |
| `src/rak_commissioning` | Commissioning entries: keyboard velocity source `chassis_key_test` (publishes `Twist` over standard ROS interfaces), `arm_tui` (pure-ROS 6-axis teach TUI over `/arm_controller` action + `/joint_states`, added 2026-10-14) + `launch/chassis_test.launch.py`. ADR-0016 removed the direct-hardware TUIs; nothing here touches CAN. Testing only, not a production motion interface |
| `src/rak_moveit_config` | SRDF, IK, planning config: `demo.launch.py` (legacy five-axis) + `six_axis_sim.launch.py` (fixture). No real trajectory execution path |
| `src/rak_arm_simulation` | Software-only six-axis trajectory/gripper Action servers (exec `fake_joint_controllers`). No physics, no hardware interface |
| `src/rak_motion_stub` | Test-only `/cmd_vel` → `/cmd_vel_raw` relay (`cmd_vel_stub`). Not a hardware driver |
| `src/rak_speech` | Offline ASR (`asr_node`, Vosk) → transcript + intent via `rak_intent`; R818 board bridge (`vtn_node` → `/speech/wake` WakeEvent + latched `/speech/vtn_ready`, codec `vtn_protocol.py` unit-tested) and offline TTS (`tts_node` → `/audio_out` → `audio_play`). Never sends goals itself. Audio in: `/audio` (`audio_common_msgs/AudioData`, `source:=topic`) or a direct `arecord` device; array/board measurements + speed budgets in `docs/microphone_array.md` |
| `src/rak_bringup` | The single composition layer: all launch (`agent`, `task_navigation`, `chassis_bringup`, `slam`, `nav2`, `lidar`, `astra`, `depth_camera`, `gemini335l`, `stereo`), params, udev, DDS config, operator scripts (absorbed `rak_sensor_bringup`, `rak_navigation`, `rak_chassis_bringup`) |
| `src/rak_agent` | (ex `brain/`) Decision layer. `ament_python` ROS package `rak_agent`, module `rak_agent`; system Python runtime; episode corpus at repo-root `data/episodes/` |
| `src/vendor/*` | Leishen lidar + `rf2o`. Third-party, **read-only** |

`depth` / `mind` / `world` / `forge` / `nerve` are roadmap names; they do not exist here.

### Outside the repo: what `~` must contain

`~` is not a workspace. Only three entries have a reason to be there (everything else is
system templates plus agent/IDE caches):

- `~/rak` — **the only ROS 2 workspace** (this repo). Every `colcon build` and every
  `source` targets it.
- `~/ws_thirdparty` — third-party driver overlay shell holding `src/OrbbecSDK_ROS2`
  (upstream `v2.10.6`, **never compiled**, working tree clean). The camera actually runs on
  the apt package `ros-jazzy-orbbec-camera` 2.9.3, so **do not source this ws**: building
  inside it is only for deliberately shadowing the apt package of the same name via source
  order. Renamed from `~/orbbec_ros2_ws` on 2026-10-14.
- `~/vehical_rak_chassis` — STM32 F407 firmware project, **not a ROS ws**. Frozen source of
  truth for the CAN v4 protocol (referenced by `scripts/can_link_smoke.py` and
  `docs/adr/0016`); do not move or absorb it.

Two gitignored local directories live *inside* the repo (moved out of `~` on 2026-10-14;
`.gitignore` carries root-anchored `/tools/` and `/debug/`):

- `tools/` — local tooling and third-party copies: `tools/camera/` (OrbbecSDK 1.10.18),
  `tools/graspnet/` (capture/verification artifacts), `tools/remote_check.sh`,
  `tools/iwlwifi-patches/`.
- `debug/` — hardware debug scraps (`spi_loopback.c`, `i2c_meter.sh`, `spi_meter.sh`).

## Build & verify

Requires the on-board JetPack SDK; build files hardcode VPI `/opt/nvidia/vpi4`,
pkg-config `nvjpeg-13.2`, CUDA `/usr/local/cuda-13.2/targets/sbsa-linux/include`.
From the repo root in a shell with only `/opt/ros/jazzy` sourced:

```bash
source /opt/ros/jazzy/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build \
  --base-paths src --executor sequential \
  --build-base build/architecture --install-base install/architecture \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/architecture/local_setup.bash

python3 scripts/check_workspace.py --installed        # package/resource/launch/message checks
colcon test --build-base build/architecture --install-base install/architecture \
  --packages-select rak_description rak_moveit_config
colcon test-result --test-result-base build/architecture --verbose
ROS_DOMAIN_ID=77 python3 -m pytest src/rak_lifecycle/test           # ADR-0020 mechanism+orchestration
ROS_DOMAIN_ID=77 python3 -m pytest src/rak_task_executor/test src/rak_arm_simulation/test
PYTHONPATH=src/rak_agent python3 src/rak_agent/test/test_ai.py
PYTHONPATH=src/rak_intent python3 src/rak_intent/test/test_task_intent.py
```

Install package dependencies with rosdep. Do not create a project venv or install generic
CUDA/PyTorch wheels. The in-process TensorRT decider is integrated and verified against
the pinned engine (`decider:=tensorrt`; see `docs/laya_tensorrt.md`).

Use `build/architecture` + `install/architecture`, never the old `build/` / `install/`
artifacts. Decision-layer-only work skips the GPU packages: `--packages-up-to rak_agent`.
ROS integration tests need the install overlay sourced and an isolated `ROS_DOMAIN_ID`;
they connect only to fake action servers and must never touch the real CAN link. Runtime
operator scripts use Cyclone DDS on domain 47 by default; test domains 77/187 stay isolated.
Treat `colcon build` + `src/rak_agent/test/test_ai.py` as the two checks to pass before
claiming done. There is no CI, linter, formatter or pre-commit config in this repo.

Two gates specifically protect the ADR-0020 orchestration layer, and both are cheap enough to
run on every change to `rak_bringup/launch/` or `rak_lifecycle/`:
`colcon test --packages-select rak_bringup` (the shipped `system_manifest.yaml` must satisfy the
fault-domain invariants **and** every `owner: system_manager` component must actually be launched
as a `LifecycleNode` in the launch file it names — a missed conversion is otherwise a silently
inert node), and `python3 scripts/check_workspace.py --installed` (23 packages now, including
`rak_lifecycle`, which must stay a pure foundation package).

The ROS node runs directly under the ROS system Python. `package.xml` declares
`python3-pydantic`; JetPack provides the TensorRT Python binding.

## Entry points

```bash
ros2 launch rak_bringup lidar.launch.py
ros2 launch rak_bringup astra.launch.py
ros2 launch rak_bringup stereo.launch.py device:=/dev/video0
ros2 launch rak_bringup chassis_bringup.launch.py    # ros2_control chassis chain (allow_motion default false)
ros2 launch rak_bringup slam.launch.py [odom_source:=rf2o|ekf|none]
ros2 launch rak_bringup nav2.launch.py               # includes lidar + SLAM + motion stub
ros2 launch rak_bringup robot.launch.py stage:=sensors|teleop|nav|full   # one command, stage-selected
ros2 launch rak_bringup supervisor.launch.py        # fault-domain orchestrator over the real manifest
ros2 launch rak_bringup supervisor.launch.py profile:=fixture   # software fixtures only, touches no hardware
ros2 run rak_lifecycle rak_system status             # per-domain state + reasons + data ages
ros2 launch rak_task_executor task_simulation.launch.py   # fake Nav2 + estimated telemetry
ros2 run rak_task_executor task_cli                  # "go to <waypoint alias>", pause/resume/cancel
ros2 launch rak_bringup task_navigation.launch.py motion_backend:=sim   # default sim
ros2 launch rak_moveit_config demo.launch.py         # legacy five-axis demo
ros2 launch rak_commissioning chassis_test.launch.py # 键盘速度源（默认 /cmd_vel_unstamped）
ros2 launch rak_moveit_config six_axis_sim.launch.py # six-axis fixture
ros2 launch rak_description view_urdf.launch.py
ros2 launch rak_bringup agent.launch.py decider:=stub sim:=true
ros2 run rak_agent agent_node --ros-args -p sim:=true -p decider:=stub
ros2 run rak_agent agent_node --ros-args -p sim:=true -p decider:=tensorrt  # engine under data/models/
PYTHONPATH=src/rak_agent python3 src/rak_agent/test/test_tensorrt_decider.py  # needs the engine; skips if absent
ros2 run rak_stereo_driver stereo_node
ros2 launch rak_bringup speech.launch.py    # array: capture->ASR, board wake->/speech/wake, /speech/say->speaker
sudo scripts/rak-mode.sh status | headless | desktop   # board memory/DMA mode
```

These are independent entries — do not start them all. `nav2.launch.py` already contains
lidar + SLAM; the arm demo's static `world → base_link` cannot be merged with the mobile
base TF. Never run two Nav2 instances or two `/cmd_vel` end adapters. `task_navigation.launch.py`
offers only `motion_backend:=sim|stub`; the real chassis comes up through
`chassis_bringup.launch.py` → `mecanum_drive_controller` → CAN (ADR-0016), and a parsed
launch is not firmware compatibility. Operator
launchers live in `src/rak_bringup/scripts/run_*.sh` and share `environment.sh` (they resolve
the repo from their own path, default DDS Cyclone, domain 47, logs to `/tmp/*_run.log`); see
that directory's `README.md`.

## Architecture

**`rak_gpu`** (`src/rak_gpu/include/rak_gpu/*.hpp`, namespace `rak::gpu`) is the shared
foundation and the idiomatic pattern for new C++ code: `CUDA_CHECK` / `CU_CHECK` /
`VPI_CHECK` macros that throw with file:line (never leak raw status codes), and RAII
wrappers — `CudaStream`/`StreamPool`/`GpuTimer` (`cuda.hpp`), `DeviceBuffer`/`PinnedBuffer`
(move-only, templated), `TrtEngine` (`trt.hpp`: load `.plan`, `build_from_onnx`, `infer`),
`VpiStream`/`VpiImage`/`VpiRectifier` (`vpi_res.hpp`).

**`rak_stereo_driver`** (`src/rak_stereo_driver/src/capture_node.cpp`) is the reference
implementation and the subtle part:
- V4L2 MJPEG capture of a side-by-side frame (1280×720 @ 30 Hz), decoded with
  `NVJPEG_BACKEND_GPU_HYBRID` straight into a GPU BGR buffer.
- **Two threads, two CUDA streams, ping-pong double buffering.** `nvjpegDecode` blocks the
  CPU ~10 ms, so it runs on a dedicated decode thread; the main thread does grab (~33 ms) +
  copy + publish. Grab > decode, so frame N decodes while N+1 is being grabbed.
- The main thread copies each eye half to host with `cudaMemcpy2DAsync` into **pinned**
  buffers, then publishes `bgr8` on `eye/left/image_raw` / `eye/right/image_raw` (topic
  names, `device`, `sbs_width/height`, `fps`, `frame_id` are node parameters). Matching
  `camera_info` publishers exist but nothing publishes to them yet — the topic names
  predate the rename and are intentionally unchanged.
- Disables `V4L2_CID_EXPOSURE_DYNAMIC_FRAMERATE` (0x009a0903) at init: some USB cameras
  silently drop to ~20 fps in low light with it on. Keep that.
- Images currently cross to host memory as `sensor_msgs/Image`; the cross-node GPU
  zero-copy pipeline is a goal, not a current fact.

**`rak_agent`** (`src/rak_agent/rak_agent/`, ex `brain/`) is the decision layer, built as the
`ament_python` ROS
package `rak_agent` and run with a standard console entry point. All entry points share one policy,
`system1.resolve()`:
- `system1.py` — fast path. `guard()` is the deterministic geometric veto (real authority);
  `gate()` only picks the escalation tier. `StubDecider` is the supported ROS backend;
  the legacy PyTorch `LayaDecider` is not used by `agent_node`.
- `system2.py` — slow path behind a `Consultant` protocol: `StubConsultant`,
  `RaiConsultant` (drives RAI's LLM plumbing directly, LangGraph underneath).
- `node.py` — ROS edge. Subscribes `/perception/tracked_objects` (BEST_EFFORT, keep-last-1:
  newest frame only, a backlog is worse than a gap), publishes `/brain/decision` (RELIABLE,
  depth 10: the consumer is an actuator). Params include `sim`, `decider`, `consultant`,
  `scenario`, `rate`, `force_s2`, `max_state_age`, `max_s2_in_flight`,
  `scene`, `flags`, `frame_id`, `out`.
- `task_intent.py` — moved out in ADR-0013 to the shared `rak_intent` package (consumed by
  `rak_task_executor` and `rak_speech`); a **separate** path from `resolve()`.
- `loop.py` — no-ROS tick loop (`sense → guard → decide → escalate → act → log`), writes
  JSONL episodes to `<repo>/data/episodes/` (ADR-0013). `body.py` — `Body` protocol + `MockBody`
  scenarios. `types.py` — the frozen S1/S2 contract.
- `agent_node` is a standard `console_scripts` entry point. Runtime dependencies are
  installed as ROS/system packages; the node does not switch interpreters.

**`rak_harness`** (ADR-0010) is the **planned** slow-path task system — a standard ROS 2 package
`src/rak_harness` with node `harness_node`, started beside `agent_node`. It owns task context,
tool whitelist, budget, timeouts, event summaries and failure recovery, and talks to a cloud LLM
through a **separate `anthropic` client** pointed at `https://api.deepseek.com/anthropic`
(key from `ANTHROPIC_API_KEY`, never `src/rak_agent/config.toml`). It emits structured proposals only
(waypoint alias / plan revision / clarification / no-op) and never velocity or joint commands.
**It does not exist yet** — ADR-0010 defines the interface and the stub contract, and the package
is created at implementation time, not before.

**`rak_task_executor`** owns the single navigation-task lifecycle: `/task/execute` (Action),
`/task/control` (Service), `/task/status` (status), driving Nav2 `NavigateToPose`. Pause is
only reported after Nav2 cancellation is confirmed **and** fresh repeated stationary telemetry
arrives; resume re-submits the semantic map goal (no replaying old velocities). Its default
parser is local; optional RAI planning may only choose a configured waypoint alias and never
emits velocity or joint commands. Production waypoints live in `config/waypoints.yaml`, which
ships **empty** — the file must declare `schema_version: 1` and `frame_id: map` or the node
refuses to start. The fake-Nav2 fixture's telemetry is a command-motion estimate, not measured
odometry.

**The base driver is `rak_chassis`** (`controller_manager` → `mecanum_drive_controller` →
`CanSession`, started by `chassis_bringup.launch.py` with `allow_motion` default false);
the former UART host node (`rak_base_driver`) was deleted by ADR-0016.
**`rak_arm_simulation`** provides software-only
six-axis trajectory/gripper Action servers for the MoveIt fixture — no physics, no hardware.

**`rak_navigation` was absorbed into `rak_bringup`** (ADR-0013) — its launch files chain the
de-facto bringup:
`lidar → laser_filters → rf2o → slam_toolbox (/map, map→odom) → nav2 → /cmd_vel → cmd_vel_stub → /cmd_vel_raw`.
The stub forwards Nav2's unicycle `Twist` verbatim. `odom_source` is the seam for odometry
sources: `rf2o` today; `ekf` is reserved and does **not** auto-start an EKF (you must supply
`odom→base_footprint` yourself, and set rf2o `publish_tf: false` when you do, or both publish it).

## Constraints that are easy to break

- **Layering rules (ADR-0013) — the workspace contract.** Modules talk only through frozen
  message contracts (`rak_interfaces` + standard ROS msgs); changing a message is a breaking
  change requiring synchronized consumers. Upper layers never branch on the transport backend
  (CAN v4 is the only one since ADR-0016) or on a sensor SDK — same-need sensors publish the same
  message set and switch via launch args. rclpy nodes are always `setup.py` entry points.
  `rak_agent` (decision layer) must not gain dependents beyond `rak_bringup`; task-intent
  parsing lives in the shared `rak_intent` package, never re-implemented per consumer.
  `check_workspace.py` enforces all of this — fix the structure, not the assert.
- **One lifecycle owner per component (ADR-0020 §3).** Nav2's six servers belong to
  `nav2_lifecycle_manager`, `slam_toolbox` belongs to the event dance in `slam.launch.py`,
  controllers belong to their spawners, and only `rak_*` nodes converted to `ManagedNode` belong to
  `rak_lifecycle/system_manager`. `system_manager` **never** sends a transition to a component it
  does not own — it only reads state. A second owner produces exactly the failure this repo has
  already documented: a node that is running, configured-looking, and completely inert.
- **Activation is not a motion permission (ADR-0020 §6).** `allow_motion` (default false) stays the
  only way anything reaches the actuators; bringing a domain up must never change that. When v5
  lands (ADR-0021), NMT `Pre-operational → Operational` is the link layer's own gate and the
  `cmd_seq`/`cmd_flags.bit0` 100 ms deadman stays in the MCU — neither is a substitute for
  `allow_motion`, and neither may be driven by the orchestrator.
- **System 1 (Laya) is a direct function call.** Never route it through an orchestration
  framework or a cross-process boundary. The RAI harness belongs only on the System 2 slow path.
- **Jev does not reach the task executor directly (ADR-0010).** The only new consumer of
  `/brain/decision` is the Harness node, and it is context only — never authority. Jev's path to
  execution is transitive: `/brain/decision` → Harness → `/harness/proposal` → `rak_task_executor`,
  which accepts a proposal only if `task_id`/`revision` match, `expires_at` is in the future, and
  the waypoint is in the configured allow-list. `rak_task_executor` must not subscribe to
  `/brain/decision`. Harness emits structured goals only — never velocity or joint commands.
- **`guard()` is the only authority** (deterministic geometric veto, overrides everything).
  `gate()` only decides escalation tier. System 2 may narrow authority, never widen it.
  Laya's confidence is out-of-domain and untrusted — a malformed model answer is a normal
  path (→ LOW → escalate), not an exception.
- **`src/rak_agent/rak_agent/types.py` is frozen once episodes exist.** `state_text` is stored verbatim
  in `data/episodes/*.jsonl` (repo root, ADR-0013); regenerating it silently corrupts the finetune corpus.
  There is deliberately no training pipeline here. `SceneState` uses camera x-right/z-forward;
  ROS body convention is x-forward/y-left — transform explicitly in the adapter, never by
  rewriting the frozen semantics. New task protocols get their own version.
- **`rak_task_executor` is the only task/goal lifecycle owner**; do not add a second Nav2 or
  a competing `/cmd_vel` writer.
- **The chassis hardware interface stays motion-gated by default.** `chassis_bringup.launch.py`
  keeps `allow_motion:=false` (the hardware only handshakes, reports telemetry and sends zero
  targets); command-interface limits are enforced by `controller_manager`
  (`enforce_command_limits: true`), and the wheel geometry / limits in
  `chassis_controllers.yaml` are unsigned placeholders. Never relax a default to allow motion
  as a shortcut. A launch argument is not firmware compatibility.
- `--decider stub` is the default and must stay runnable without a model. The target System 1
  runtime is in-process TensorRT from JetPack; do not add a model service or generic
  CUDA/PyTorch wheel dependency. The TensorRT backend is integrated: one factory
  (`make_decider` in `src/rak_agent/rak_agent/system1.py`) serves both the CLI loop
  (`--decider tensorrt`) and the ROS node (`decider:=tensorrt`); the engine and tokenizer
  live under the gitignored repo-root `data/models/laya-typed-decisions/` (override with
  `RAK_LAYA_MODEL_DIR` or `-p decider_model_dir:=`).
- **Two LLM keys, two providers.** RAI's slow path reads `OPENAI_API_KEY` (SiliconFlow,
  `src/rak_agent/config.toml [openai]`); the ADR-0010 Harness client reads `ANTHROPIC_API_KEY`
  (DeepSeek's Anthropic-compatible endpoint). Do not share one variable across providers, and
  `src/rak_agent/config.toml` must never contain either key.
- ROS message type identity includes the package name: `rak_interfaces/msg/Decision` does not
  interoperate with the old `core/msg/Decision`. External nodes must be rebuilt; old rosbags
  need explicit type migration.
- The target arm is **standard six axes plus a servo gripper** (a 7th PWM servo) and an
  eye-in-hand camera (ADR-0008). The **real** six-axis model is measured (2026-10-08):
  `rak_arm.urdf.xacro` + `properties_arm.xacro` + `rak_arm.srdf` (limits, axes, zero mapping;
  link masses still `TODO(measured)` placeholders). The five-axis URDF is a demo and the
  six-axis *fixture* (`six_axis_sim.urdf.xacro`) is test-only — do not present either as the
  real hardware, and do not invent calibration for the parts not measured yet (masses, gripper
  mapping, eye-in-hand extrinsics). Do not configure a real `ros2_control` arm hardware plugin
  before the Arm Unlock Gates in `docs/hardware_acceptance.md` pass.
- `src/vendor/**` is third-party. Read-only; do not reformat or "fix" it.
- **CAN protocol ownership.** The wire format's source of truth is the leaf-node repo
  (`../vehical_rak_chassis/docs/PROTOCOL.md`, **CAN v4** since 2026-10-09; v3 retired), not
  this one. The bus is classic CAN 2.0A @ 1 Mbps — never pass `fd on` (the F407 bxCAN would
  error-frame an FD frame). The host-side v4 implementation is `CanSession` in
  `src/rak_chassis` (the production path); the retired `rak_serial_link` lives only
  in git history since the 2026-10-10 `attic/` deletion, not live.

## Gotchas

- **rclpy lifecycle footgun that costs a whole subsystem (found by test, 2026-10-29):** an
  `on_activate` override that forgets `super().on_activate(state)` leaves every
  `LifecyclePublisher` permanently disabled, and `publish()` on a disabled one is **silently
  dropped** — the node looks healthy, spins, logs, and emits nothing. `ManagedNode` centralizes the
  forwarding; any new lifecycle node must call `super()` in every transition hook.
- Jazzy message details that differ from the usual documentation: `DiagnosticArray`'s field is
  `status` (not `statuses`), `DiagnosticStatus` only has `OK/WARN/ERROR/STALE` (no `STOP`/`STAT`),
  and the `TRANSITION_*` id constants live on `lifecycle_msgs.msg.Transition`, not on
  `TransitionDescription` (which is a message type). All three were hit for real here.
- `diagnostic_updater` is installed but its **Python** side has no `TopicMonitor`; use
  `TopicDiagnostic`/`FrequencyStatus`/`TimeStampStatus`. `diagnostic_aggregator` is **not**
  installed, so `/rak/system_state` is aggregated by `rak_lifecycle` itself (accepted cost,
  ADR-0020 §后果).
- `ros2 lifecycle set` on a node nobody activates is not a bug in the CLI: `LifecycleNode`s start
  `unconfigured` and do nothing until configured. Check `ros2 lifecycle get <node>` before
  concluding a node is broken.
- **CMA memory contention.** The 1024×600 touchscreen and the camera compete for the same
  pool. A logged-in GNOME session drives `CmaFree` to ~0 kB of the 256 MiB CMA area, and the
  stereo node asks that pool for 4 `V4L2_MEMORY_MMAP` buffers — so capture fails in a way
  that looks like a camera bug but is a memory decision. `scripts/rak-mode.sh headless`
  stops and disables gdm. GUI launchers (RViz/MoveIt) require `desktop`; the migrated
  sensor/navigation scripts do not. Autologin is off, so returning to the desktop means
  typing the password at the touchscreen once. (VNC via `x11vnc` attaches to whatever display
  GDM runs; see `README.md` for the three routes. **`vino` — what L4T's `README-vnc.txt`
  recommends — is installed but stopped**, autostart renamed to `vino-server.desktop.disabled`:
  re-enabling it makes it fight `x11vnc` for port 5900, and it burns a core anyway — it ignores
  X DAMAGE in the requested region, so an idle screen still ships 99.7 % of the framebuffer.)
- **The host memory picture changed on 2026-10-05** (`docs/CONSTRUCTION.md` §2.2, not in git):
  zram (zstd ~1.7 G) + a 4 G swapfile now exist, so "out of memory" is less immediately fatal
  than this file previously implied. **swap/zram are not real-time guarantees** — the ~158 ms
  S1 and 12–30 s S2 figures are unaffected. Whether zram helps or hurts on a 4 GB board is an
  **open, contested question** (`TODO.md` P2-1): measure `si/so`, compression ratio and p99
  latency under real load before changing it, don't revert from documentation alone.
- **Power mode is pinned at boot now**: `nvpmodel -m 0` (10 W) + `jetson_clocks`. The historical
  ~158 ms/predict and 30 fps figures were measured before/around this lock — **re-measure, do
  not compare benchmarks across power/clocks states.**
- **nvJPEG / stereo fps and the S1 latency are 10 W-mode numbers**; the EMC bandwidth cap
  applies. Do not assume a benchmark is comparable after any power change.
- **The retired PyTorch/Laya path cost ~1.4 GB of this box's 3.4 GB unified memory** on the
  GPU: ~804 MiB of
  bf16 weights (421M params) plus ~600 MiB of CUDA context and cuDNN/cuBLAS workspace. The
  checkpoint is F16 on disk, but `laya.load()` builds in torch's *default* dtype (fp32) and
  `load_state_dict` copies up, so weights used to sit resident at 1606 MiB; on Tegra CUDA's
  "free" is just pages nobody holds, so the fp32 build peaked on the CPU side and starved
  the GPU side at the same moment — `laya.load(model)` OOMed and fell back to CPU before
  anything could be cast. `LayaDecider` therefore loads on CPU with the default dtype already
  set to the checkpoint's `amp_dtype` (bf16) and places afterwards, so fp32 never exists.
  `s1_weights=auto|fp16|bf16|fp32` selected it. This is historical PyTorch backend
  behavior only; do not install or select it for current Jetson runs.
- Half weights only run where autocast is on, and laya only turns it on for CUDA (CPU bf16 is
  the documented `LAYA_CPU_AMP=bf16`). Without it the forward dies in `F.linear` with
  "mat1 and mat2 must have the same dtype". The legacy `LayaDecider` degraded to fp32 when
  there was no CUDA; do not use it for current Jetson runs.
- If it did not fit, `laya` caught CUDA OOM and dropped to CPU at ~1.6 s/predict — slower
  than the sensor rate. This is historical only; the ROS node rejects `decider:=laya` and
  `decider:=tensorrt` is the integrated backend (engine under repo-root `data/models/`).
- The legacy PyTorch/Laya parity test measured the executed policy over 90 states. It does
  not qualify TensorRT; the engine's own acceptance (token layout exact, numerics within
  bf16, same `Decision` contract) is recorded in `docs/laya_tensorrt.md` §Acceptance and
  pinned by `src/rak_agent/test/test_tensorrt_decider.py`. A rebuilt engine needs it again.
- **System 2 latency is 12–30 s per call** (measured), so S2 is fire-and-forget in a thread
  and replies are applied on a later tick. `_drain()` currently only *records* replies; it
  does not execute them. The S1 callback also precedes safety resolution, so this prototype
  is **not** an independent real-time safety controller.
- C++ is C++17 with `-Wall -Wextra -Wpedantic` in the GPU/stereo packages; `rak_stereo_driver`
  runs `ament_lint_auto` under `BUILD_TESTING`, `rak_gpu` has no functional tests.
- `data/`, `build/`, `install/`, `log/` are gitignored. Stray
  `frames_*.gv` / `frames_*.pdf` at the root are debugging artifacts, not deliverables.
- **`colcon build` only ever runs from the repo root** (`--base-paths src`). Run it from `~`
  and it drops orphan `~/build` / `~/install` / `~/log` in the home directory *and* drags
  `~/ws_thirdparty` and `~/vehical_rak_chassis` into package discovery — that is exactly how
  the 2026-10-10 stray artifacts happened (since deleted; their `events.log` named
  `OrbbecSDK-Samples` and `lslidar_driver` from foreign trees). Home must never hold colcon
  output; if `~` grows a `build/`, it is a mistake, not a workspace.
- **`tools/camera` is a load-bearing path, not junk.** `rak_astra_driver`'s CMake cache and
  `astra_node`'s `RUNPATH` are baked to `/home/xrak/rak/tools/camera/.../SDK` — `ldd
  install/rak_astra_driver/lib/rak_astra_driver/astra_node` shows it resolving there — so
  deleting or re-moving `tools/` breaks configure *and* runtime (`libOrbbecSDK.so.1.10.18`
  not found). Without `-DOrbbecSDK_DIR` CMake falls back to its default
  `/opt/OrbbecSDK_v1.10.18/SDK`; both copies coexist, and changing the path requires
  rebuilding `rak_astra_driver` in **both** `build/` and `build/camera-debug/` (the RUNPATH
  lives in the ELF, so text-editing the cache is not enough).
- The CAN link smoke test is `python3 scripts/can_link_smoke.py can0` (10/10 pass = qualified;
  battery ≈12500 mV, FAULT=0, BUSOFF=0). Physical/driver layers: `ip -details -statistics link
  show can0` and `dmesg | grep -i can`. The old serial scripts were deleted with the USB link
  (ADR-0016).
- `src/rak_moveit_config/test/validate_six_axis_sim.py` asserts the six-axis URDF contains
  **no** `<ros2_control>` tag. Adding one as part of the ADR-0007 migration will break that
  test — update it deliberately, don't just make the assertion pass.
- The lidar `lidar_name` must be `N10_P` (108 B @ 460800); `N10` fails CRC. Use the udev alias
  `/dev/wheeltec_lidar`, never `/dev/ttyUSB0` (other serial adapters may also claim ttyUSB).
- The former USB-serial `latency_timer` caveat went away with the link itself: since ADR-0016
  the base runs over SocketCAN, whose latency is not governed by a USB-serial timer.
