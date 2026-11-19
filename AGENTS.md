# AGENTS.md

> **协议只剩一套：v5 = CANopen（2026-11-14 主机侧 v4 已整体删除）** —— 下位机 2026-10-29 就切了，
> 上位机这次跟上：`src/rak_chassis` 的 v4 `CanSession`/`can_decode`/帧表/`can_link_smoke.py` 全部删除，
> 现在是**薄桥**（`CanopenChassis` / `CanopenArm`：只做 OD 对象 ↔ ros2_control 接口、量化、`allow_motion`
> 安全门），协议栈 = apt 的 `ros2_canopen` 0.3.4（NMT/SYNC/PDO/SDO/心跳归它）。
> 线缆与对象事实源仍是两个下位仓：`../vehical_rak_motion/docs/CANOPEN-V5.md` + `docs/canopen/chassis.json`
> （底盘板）、`../vehical_rak_arm/docs/CANOPEN-V5.md` + `docs/canopen/arm.json`（机械臂板）。
> 无下位机时的验证 = `python3 scripts/verify_canopen_vcan.py`（vcan0 + fake slave，实测记录见
> `docs/status/canopen_vcan_2026-11-14.md`）。**真机 can0 已验收（2026-11-17：G0–G3 全过，
> 四轮真实前移 10 s，轮序/极性 v5 首测闭合）**；记录见
> `docs/how-to/chassis_canopen_bringup.md` §7、上位机接入见 `docs/how-to/remote_host_chassis_access.md`；
> 同一总线永远只能有一个主站。


Layered embodied robot system for Jetson Orin Nano (JetPack 7.2). C++/ROS 2 (Jazzy) + a
plain-Python decision layer. `README.md` is the engineering entry point and this file is the
single agent-instruction source — there is no second instruction file; read `## Orientation`
first, then the topology section, before changing behavior.

## Orientation

Layered embodied-robot workspace for Jetson Orin Nano / ROS 2 Jazzy: perception + decision on
the Orin, real-time control on **two** STM32F407 leaf nodes (chassis + arm; see the topology
changes below — the "one F407" phase is history again).
Read these before changing behavior — they record decisions the code does not show:

- `README.md` — engineering entry point (Chinese): layout, build, launch entries, hardware.
- `docs/explanation/architecture.md` — package boundaries, target runtime architecture, acceptance stages,
  and an explicit list of gaps that are **not** closed.
- `docs/status/project_status.md` — what is actually done vs. only claimed (dated; read the date).
- `docs/tutorials/construction.md` — assembly order and deployment checklist; host-side board changes.
- `docs/adr/` — **immutable** architecture decisions (why, and what was rejected). Index in
  `docs/adr/README.md`. `ADR-0007` is the current control architecture and **supersedes ADR-0001**.
  `ADR-0016` (2026-10-14) removed the USB v2 link — CAN is the only transport (`CanSession`),
  and tools talk to drivers only through topic/action/service. `ADR-0010` defines the
  Jev↔Harness arbitration seam and the reserved "呼吸感" expression interface; the seam is
  **design-only** — `src/rak_harness` exists (2026-10-15 it also hosts the P1 agentic tool
  loop), but the arbitration path itself still needs its ADR.
- `docs/adr/0022-jetson-canopen-contract-layer.md` — how the **Jetson** side binds to CANopen v5: protocol stack = lely-core (ADR-0021), this repo only *derives* COB-IDs / PDO layout / rates / quantization from the MCU SSOT (`src/rak_canopen/`) and gate-checks that v4 and v5 never share a bus.
- `docs/reference/hardware_acceptance.md` — gates that must pass before any physical base/arm test.
- `docs/design/brain_arm_tools.md` — **DESIGN DRAFT** (2026-10-15, not accepted): tool-proposal
  contract, pinocchio-IK arm chain, tool menu, agentic-loop consumption (no CLI; the
  cross-process MCP surface is separately accepted by ADR-0028, same proposal seam), phasing P0–P5.
  Needs an ADR before implementation.
- `docs/adr/0028-mcp-tool-surface.md` — **accepted**: `src/rak_mcp` exposes the tool registry over
  standard MCP (stdio JSON-RPC); it is a third *client* of `/tool_proposals`, never a second
  executor, and holds no speed limit / safety constant of its own.
- `docs/explanation/migration_2026-10.md` — the 2026-10-04 unified `rak_` package rename (old name → new name table).
- `TODO.md` — prioritized roadmap (P0–P3). Tasks only; rationale lives in the ADRs.
- `docs/README.md` — docs index + writing rules (Diátaxis layout, status line, numbering).

**Trust hierarchy when docs disagree.** This file is the current lead on topology (updated
2026-11-14); the sections below keep the historical reasoning. The 2026-10-04 docs
(`README.md`, `docs/explanation/architecture.md`, `docs/status/project_status.md`) describe the code as it stood
then; the 2026-10-05 docs (`TODO.md`, `docs/tutorials/construction.md`, `docs/adr/0007`) describe the
**accepted target** after the first hardware change. Do not treat an old status line as current.

## Topology: dual-board again since 2026-10-28 — read this first

**2026-10-28 — the second board is back.** The earlier "arm board burned out / one F407
driving 4 wheels + 7 servos" topology (2026-10-05 … 2026-10-27) is **superseded**. There are
now **two** C30D F407 boards on one CAN bus with the Jetson (three nodes total):

- **chassis board** (`vehical_rak_motion`, firmware repo HEAD `R76`, 在板 `R73` — 2026-11-15 对账):
  4 wheels + onboard IMU.
- **arm board** (`vehical_rak_arm`, git dir name `vehical_rak_arm`, firmware repo HEAD `A36`, 在板 `A14`):
  **J1/J2 = MG513P30 incremental-encoder motors** (AT8236 + TIM6 25 kHz position loop,
  software zeroing) + **J3..J6 = servos**. A2 (2026-10-28) motorized J1/J2; A1 was pure-servo.
  在板版本一律以 SDO `0x2F00` build_tag 实读为准——repo HEAD 领先在板版本是常态。

**Jetson sees no wire change from this**: the `ARM_CMD` (0x110/0x111) and `SERVO_STATE`
(0x210/0x211) frame IDs and byte layouts are unchanged — only the *board* that answers them
moved. The arm board adds two *additive* uplink frames, `0x031 ARM_NODE_INFO` (1 Hz) and
`0x221 ARM_DIAG` (10 Hz); `rak_chassis` subscribes to both as of 2026-10-29. CAN protocol
version stays **v4** (`NODE_INFO.PROTO==4`). `servo_valid_mask` now means: bit0/bit1 =
J1/J2 **true encoder measurement**; bit2..5 = J3..J6 **servo-pulse echo** (not physical);
bit6 grip = no feedback.

**Still unverified on hardware**: repo-HEAD firmware (`R76` / `A36`) has never been
flashed/run (在板的是 R73 / A14). J1/J2
encoder polarity, gains and limits are `[AI]` placeholders — see
`vehical_rak_arm/README.md` §6 before sending any `ARM_CMD`.

**2026-10-09 (ADR-0012): CAN v4 is the primary link** (SN65HVD230 transceiver, protocol v4 in
the leaf repo). **ADR-0016 (2026-10-14) then deleted the USB v2 transport entirely**: CAN v4
is now the *only* transport, and `Session` (`src/rak_chassis`) has a single implementation,
`CanSession`. Upper layers never see the link. **2026-10-09 (ADR-0013): the workspace was
regrouped by subsystem** — `brain/` moved to `src/rak_agent`, CAN v3 stack retired (local
`attic/` archive deleted 2026-10-10 — the code lives only in git history). On 2026-10-29 the
two remaining v3 leftovers were also removed: the `rak_can_driver` package (deleted) and a
never-committed `rak_base_driver` dir; **ADR-0016 had already deleted `rak_base_driver`
itself** together with `SerialSession`. `scripts/check_workspace.py`'s `EXPECTED` is now **27**
owned packages.

- `docs/adr/*` are immutable (append/supersede only), indexed by `docs/adr/README.md`.
  **ADR-0007 supersedes ADR-0001**; the valid set is 0002 + 0003 (v4 layer) + 0004–0015.
  **ADR-0011 (WBC/HQP motion arbitration) was accepted 2026-10-08**: `src/rak_wbc`
  (ament_python skeleton + desktop numeric tests) exists; arm interfaces and measured
  masses/geometry remain hard gates — read its closed items before wiring hardware.
- ADR-0010's target — Harness as a standard ROS 2 package `src/rak_harness` + node
  `harness_node`, the Jev↔executor arbitration seam, a separate `anthropic` cloud client and a
  gated "呼吸感" expression interface — is **accepted but design-only**: the expression layer is
  a no-op stub behind closed hardware gates, and as of 2026-10-15 `src/rak_harness` also hosts the
  **P1 agentic tool loop** (Claude Agent SDK in a subprocess, `voice_path:=agent|dialogue|auto`)
  from the design draft below — that loop itself still **needs its ADR** before it is considered
  accepted.
  The **DESIGN DRAFT** for the brain↔arm/tools operation layer (versioned tool-proposal
  contract, pinocchio-IK arm chain, tool menu, agentic-loop consumption — no CLI external
  control surface; the MCP one is ADR-0028, phasing P0–P5) is
  `docs/design/brain_arm_tools.md` (2026-10-15, **not accepted** — needs an ADR; do not
  implement it as if it were one).
- ADR-0007's target — whole robot on `ros2_control`, host-side mecanum IK — is **partially
  implemented as of 2026-10-14**: `src/rak_chassis` (ex `rak_hardware`, renamed by ADR-0013)
  exports 4 wheel velocity interfaces (F407 chassis smoke test, firmware R37) **plus 7 arm
  position command interfaces** (6 joints + gripper, command domain = servo command angle,
  contract `docs/reference/arm_moveit_contract.md` §1), with `arm_controller`
  (JointTrajectoryController) and `gripper_controller` spawned by
  `chassis_bringup.launch.py`. Verified 2026-10-14 on the live F407: first with
  `allow_motion:=false` (arm feedback syncs from the firmware echo, all 4 controllers
  activate, `/joint_states` carries the 6 arm joints, action path + `arm_tui` end-to-end,
  suppressed goals abort with a legible hint), then **with `allow_motion:=true` for a real
  jog**: j1 +5° and Home both SUCCEEDED, the wire carried 35 `ARM_CMD_A` frames ramping
  0.1°→5.0°→0.0° at 0.1° quantization (event-driven, no periodic replay), readback followed,
  no faults. Still unverified: J2–J6 physical motion, limit/estop behavior under motion,
  and the gripper (no hardware channel — readback is always 0). The 7 arm joints' real
  geometry is defined **once** in `src/rak_description/urdf/macros/arm_chain.xacro`
  (`<xacro:rak_arm_chain/>`, 2026-10-16) and shared by `rak_arm.urdf.xacro` and
  `rak_chassis.urdf.xacro`; the former zero-length control stub chain is gone. The gripper
  has no geometry — a bare `gripper_link` frame after `tool0` only keeps Jazzy's
  "every `<ros2_control>` joint must exist in the kinematic tree" satisfied.
  `src/rak_chassis/test/validate_chassis_urdf.py` pins that contract under `colcon test`.
  `src/rak_moveit_config/test/validate_six_axis_sim.py:36` still asserts the
  six-axis **sim fixture** URDF contains no `<ros2_control>` tag — that fixture is not the
  real arm.
- The **CAN v3 host stack** (`rak_can_driver`, `rak_serial_link`) is retired (ADR-0012/0013);
  the local `attic/` archive was **deleted on 2026-10-10**, and the last straggler package
  `src/rak_can_driver` was **deleted on 2026-10-29** — the code lives only in git history
  (retrievable at commit `4f34795` and earlier). The live CAN path is the `CanSession`
  backend inside `src/rak_chassis`.
- Docs disagree by date, not correctness. The 2026-10-04 set (`README.md`,
  `docs/explanation/architecture.md`, `docs/status/project_status.md`) describes the code as it stands;
  `TODO.md`, `docs/tutorials/construction.md` and `docs/adr/0007` describe the accepted target.
  Do not read a 10-04 status line as current, and do not implement the target as if the
  code already supported it.

## Layout

The 2026-10-09 subsystem regroup (ADR-0013) supersedes the 2026-10-04 package layout.
Read `docs/explanation/architecture.md` and `docs/explanation/migration_2026-10.md` (dated mapping tables) before moving
package boundaries. `scripts/check_workspace.py`'s `EXPECTED` holds **27** owned packages;
every one has a real responsibility:

- `src/rak_interfaces`: messages/actions/services only, no GPU/model dependencies.
- `src/rak_gpu`: JetPack library, exported target `rak_gpu::gpu`, namespace `rak::gpu`.
- `src/rak_stereo_driver`: plugin `rak::stereo_driver::CaptureNode`, executable `stereo_node`.
- `src/rak_astra_driver`: tested depth camera adapter, executable `astra_node`.
- `src/rak_pointcloud`: GPU point cloud node (`gpu_pointcloud_node`).
- `src/rak_imu_driver`: Jetson-direct LSM6DSRTR IMU over SPI1 → `sensor_msgs/Imu` on
  `/imu/data_raw` (heading authority, ADR-0002).
- `src/rak_road_perception`: TorchScript road segmentation (+ optional Nav2 point-cloud
  observations).
- `src/rak_grasp_perception`: RGB-D snapshots, offline grasp candidates, planning visualization.
- `src/rak_description`: geometry and visualization — legacy five-axis demo plus the
  isolated six-axis fixture `urdf/six_axis_sim.urdf.xacro`.
- `src/rak_moveit_config`: SRDF, IK and planning configuration; six-axis fixture is simulation-only.
- `src/rak_arm_pinocchio_ik`: pinocchio-based IK arm chain for the brain↔arm tools layer.
- `src/rak_chassis`: (ex `rak_hardware`) **CANopen v5 薄桥** — `ros2_control` 硬件插件
  `CanopenChassis`（4 轮 velocity）/ `CanopenArm`（7 轴 position），只做 OD 对象 ↔ 接口、
  量化与 `allow_motion` 安全门；协议栈是 `ros2_canopen`（订阅 `<driver>/rpdo`、发布
  `<driver>/tpdo`），本包**不碰 CAN 帧**。旧的 v4 `CanSession`/`can_decode`/Session 抽象已删除。
- `src/rak_canopen`: CANopen v5 **Jetson-side contract layer** (ADR-0022) — derives COB-IDs,
  PDO layout, rates, quantization and v5-internal COB-ID uniqueness from the MCU SSOT manifest;
  owns `bus.yaml` + the `dcfgen`-generated `master.dcf`（官方生成器，漂移门禁
  `test/test_dcf_sync.py`）。Not a protocol stack; production master is `ros2_canopen` (ADR-0024).
- `src/rak_commissioning`: (ex `rak_drive_tui` + `rak_arm_tui` + `chassis_key_test`)
  commissioning entries: `chassis_tui`, `arm_tui`, `chassis_key_test`,
  `launch/chassis_test.launch.py`. Both TUIs are pure ROS 2 tools (actions/topics only, never
  touch CAN directly — `check_workspace.py` forbids this package depending on `rak_chassis`).
  Not production motion interfaces.
- `src/rak_arm_simulation`: software-only six-axis trajectory/gripper action fixture.
- `src/rak_lifecycle`: managed-lifecycle infrastructure (ADR-0020): fault-domain manifest,
  `ManagedNode` base, `system_manager`, `rak_system` CLI.
- `src/rak_orchestration`: assembly **contract** layer — manifest, launch contracts,
  supervisor entry; buildable/testable on a plain x86 desktop.
- `src/rak_metrics`: latency/age/drop aggregator (p50/p95/p99) as `diagnostic_msgs` + a small
  HTTP endpoint; also owns the rosbag record profile.
- `src/rak_harness`: Jev↔executor arbitration seam + voice dialogue loop (ADR-0010) —
  advisory proposals only, never velocity or joint commands.
- `src/rak_tool_executor`: the single deterministic executor for the versioned tool-proposal
  contract (`docs/design/brain_arm_tools.md`, still a DESIGN DRAFT). Owns the only motion gates
  (`allow_motion`, chassis single-writer interlock) and the audit trail.
- `src/rak_mcp`: standard MCP (stdio JSON-RPC 2.0) surface over the **same** registry — tools,
  schema validation and timeouts are imported from `rak_tool_executor`, never copied (ADR-0028).
- `src/rak_task_executor`: sole owner of text-navigation task and Nav2 goal lifecycle;
  depends on `rak_intent`, never on `rak_agent`.
- `src/rak_intent`: shared versioned text→intent contract (stdlib-only at import time);
  consumed by task executor and speech (ADR-0013).
- `src/rak_speech`: offline ASR (`asr_node`) + intent publishing over `rak_intent`; the R818
  array board bridge (`vtn_node`: adb-tunneled vtn socket → `/speech/wake` WakeEvent +
  latched `/speech/vtn_ready`, frame codec `vtn_protocol.py` is unit-tested) and offline TTS
  (`tts_node`: `/speech/say` Say service → sherpa-onnx → `/audio_out` → `audio_play`).
  Audio in is `audio_common_msgs/AudioData` on `/audio` (`source:=topic`, published by
  `audio_capture` in `rak_bringup speech.launch.py`) or a direct `arecord` device; the
  six-mic array's measured facts, the board protocol and the decode/synthesis speed budget
  live in `docs/reference/microphone_array.md`.
- `src/rak_wbc`: velocity-level whole-body arbitration (ADR-0011): ament_python standalone
  node `wbc_node` — single regular motion writer for base + arm; skeleton + desktop numeric
  tests exist, hardware wiring gated by the ADR's closed items. Fixtures/smoke:
  `fake_arm_stream` (command-echo arm fixture) and `scripts/wbc_shadow_smoke.sh`
  (shadow contract smoke; remaps the base output — no actuator is commanded).
- `src/rak_motion_stub`: test relay/estimate, not a hardware driver.
- `src/rak_bringup`: the single composition layer — all launch files (`agent`, `robot`,
  `sensors`, `task`, `task_navigation`, `harness_agent`, `chassis_bringup`,
  `slam`, `nav2`, `lidar`, `astra`, `depth_camera`, `gemini335l`,
  `grasp_camera`, `stereo`, `speech`), params,
  udev and operator scripts (absorbed `rak_sensor_bringup`, `rak_navigation`,
  `rak_chassis_bringup`).
- `src/rak_agent`: (ex `brain/`) the decision layer, ROS package `rak_agent`, module
  `rak_agent`. Episode corpus at repo-root `data/episodes/` (gitignored). Only `rak_bringup`
  may depend on it (it launches `agent_node`); check_workspace enforces this.
- `src/vendor`: third-party code, read-only.

Target arm: six positioning joints plus an independent gripper. The real six-axis geometry,
limits, axes and zero mapping were **measured on 2026-10-08**
(`src/rak_description/urdf/rak_arm.urdf.xacro` + `properties_arm.xacro`;
`src/rak_moveit_config/config/rak_arm.srdf`; link masses are still `TODO(measured)`
placeholders). Never invent the missing axis or calibration from the legacy URDF, and do not
confuse the six-axis **sim fixture** with the real arm. Models, topics and episode schemas
were not redesigned during the package migration.

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
- `~/vehical_rak_motion` — STM32 F407 **chassis board** firmware project (4 wheels + IMU),
  **not a ROS ws**. Carries the v5 wire truth: `docs/CANOPEN-V5.md` +
  `docs/canopen/chassis.json`, plus `tools/canopen_bringup.py` (raw SocketCAN) for on-board
  verification. Do not move or absorb it. The v4 `PROTOCOL.md` there is now history.
- `~/vehical_rak_arm` — STM32 F407 **arm board** firmware project (J1..J6 + gripper), not a
  ROS ws. Renamed 2026-11-14 from `vehical_rak_edge_ai`; same v5 layout
  (`docs/CANOPEN-V5.md` + `docs/canopen/arm.json` + `tools/canopen_bringup.py`).
  `vehical_rak_chassis` is a **dead name** — that repo no longer exists.

Two gitignored local directories live *inside* the repo (moved out of `~` on 2026-10-14;
`.gitignore` carries `/tools/` and `/debug/`, root-anchored so they cannot swallow a nested
`tools/` in `src/`):

- `tools/` — local tooling and third-party copies: `tools/camera/` (OrbbecSDK 1.10.18),
  `tools/harness-sdk/` (vendored `claude-agent-sdk` overlay for the P1 agent loop's
  subprocess — its pydantic v2 tree must never enter the ROS system Python), `tools/cosyvoice/`
  (vendored CosyVoice3 + torchaudio shim + matcha for the GPU TTS backend; J7.2 adaptations
  in-file), `tools/qwentts.cpp/` (vendored Qwen3-TTS GGML runtime — the **default** TTS
  content layer per ADR-0018: `build/tts-server` + `models/*.gguf`, local uncommitted
  `QT_MAX_SEQ` patch in `src/pipeline-tts.cpp`; rebuild to keep the binaries in step),
  `tools/graspnet/`
  (capture/verification artifacts), `tools/remote_check.sh`,
  `tools/iwlwifi-patches/`.
- `debug/` — hardware debug scraps (`spi_loopback.c`, `i2c_meter.sh`, `spi_meter.sh`).

## Commands

No CI, no linter/formatter config, no pre-commit. Verification commands:

```bash
# C++ / ROS 2 — repo root, only /opt/ros/jazzy sourced
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --executor sequential --build-base build/architecture --install-base install/architecture --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/architecture/local_setup.bash
python3 scripts/check_workspace.py --installed
python3 scripts/check_docs.py                 # docs link/index/status-line gate
colcon test --build-base build/architecture --install-base install/architecture --packages-select rak_description rak_moveit_config
colcon test-result --test-result-base build/architecture --verbose
ROS_DOMAIN_ID=77 python3 -m pytest src/rak_task_executor/test src/rak_arm_simulation/test
ROS_DOMAIN_ID=77 python3 -m pytest src/rak_lifecycle/test
ros2 run rak_stereo_driver stereo_node
ros2 launch rak_bringup lidar.launch.py
ros2 launch rak_bringup speech.launch.py  # array: capture -> ASR, board wake -> /speech/wake, /speech/say -> speaker
python3 scripts/make_quick_phrases.py     # 重新生成即时反应层素材(serena);跑 TTS 前先跑一次
ros2 topic pub --once /speech/quick std_msgs/String '{data: ack_wake}'  # 即时层:直出预合成 wav

# Memory/DMA mode of the board itself: status | headless | desktop (needs sudo; see Gotchas)
sudo scripts/rak-mode.sh headless

# The brain as a standard ROS node (system Python; decider:=stub | tensorrt)
ros2 launch rak_bringup agent.launch.py decider:=stub sim:=true
ros2 run rak_agent agent_node --ros-args -p sim:=true -p decider:=stub
ros2 run rak_agent agent_node --ros-args -p sim:=true -p decider:=tensorrt  # engine under data/models/
PYTHONPATH=src/rak_agent python3 src/rak_agent/test/test_tensorrt_decider.py  # needs the engine; skips if absent

# Python decision/intent tests, no ROS
PYTHONPATH=src/rak_agent python3 -m rak_agent.loop --scenario approach --decider stub --ticks 50
PYTHONPATH=src/rak_agent python3 src/rak_agent/test/test_ai.py
PYTHONPATH=src/rak_intent python3 src/rak_intent/test/test_task_intent.py
```

Install ROS package dependencies through rosdep (`rosdep install --from-paths src
--ignore-src -r -y`). Do not create a project venv or install generic CUDA/PyTorch wheels.

Each command above has a known, unrelated way to fail:

- **A full build needs three host deps** that may be absent: `/opt/OrbbecSDK_v1.10.18`
  (`rak_astra_driver`), `libpcap-dev` (vendor `lslidar_driver`), and
  `ros-jazzy-camera-info-manager` (`rak_stereo_driver`). Until they exist, build with
  `--packages-skip rak_astra_driver lslidar_driver rak_stereo_driver` — the remaining
  packages still build. Their absence also makes `stereo_node` / `lidar.launch.py` fail
  with "package not found", and leaves a partial install entry, so sourcing prints
  `not found: .../local_setup.bash` (harmless).
  **On a clean tree (empty `build/architecture`+`install/architecture`) `--packages-skip` is
  not enough**: a skipped dep that an included package declares (`rak_bringup` →
  `rak_astra_driver`) makes colcon fail sourcing its `package.sh`. Use
  `--packages-ignore rak_astra_driver` instead — the dep drops out of the graph and the rest
  builds (2026-10-11: 23 packages, exit 0, with only the Orbbec SDK absent).
- **`check_workspace.py` is structure law (ADR-0013)**: at HEAD it exits 0. If it fails,
  fix the structure (package.xml deps, launch references, frozen boundaries), never the
  assertion. Historical note: before the 2026-10-09 regroup it failed on its own stale
  EXPECTED set; that debt is gone.
- **`PYTHONPATH=src/rak_agent` replaces ROS's own PYTHONPATH** →
  `ModuleNotFoundError: No module named 'rclpy'`. Append `:$PYTHONPATH`; putting
  `src/...` first also makes pytest test your edits without a rebuild.
- **`test_protocol.py` duplicate basenames are a pytest hazard** (two files with the same
  basename in one run → `ImportPathMismatchError`, even with `--import-mode=importlib`).
  Run one package per invocation, or `colcon test --packages-select`, which isolates them.
- **ROS pytest needs the install overlay sourced** — `rak_interfaces` exists only there.
  `ROS_DOMAIN_ID=77` keeps it off runtime domain 47, which `~/.bashrc` pins for the operator.
  `colcon test` on `rak_moveit_config` fails `legacy_srdf` until `python3-defusedxml`
  (a declared `test_depend`) is installed.
- `rak_agent` requires the Ubuntu `python3-pydantic` system package. The TensorRT binding is
  supplied by JetPack; the engine + tokenizer are integrated and serve both the CLI loop and
  the node (`decider:=tensorrt`).
- Docs are gated too: `python3 scripts/check_docs.py` fails on a broken relative link, a doc
  missing from `docs/README.md`, a missing `> 状态：` line, or an ADR missing from
  `docs/adr/README.md`. It link-checks this file, `README.md` and `TODO.md` as well.

`rak_agent` runs under the sourced ROS system Python. ROS integration tests must keep the
install overlay on `PYTHONPATH` and use an isolated `ROS_DOMAIN_ID`; plain decision tests
can use `PYTHONPATH=src/rak_agent`.

Text navigation smoke test: source `install/architecture/local_setup.bash`, then run
`ros2 launch rak_task_executor task_simulation.launch.py` and in another terminal run
`ros2 run rak_task_executor task_cli`. This fixture uses fake Nav2 and explicitly estimated
base telemetry; it does not use sensors or represent physical feedback. For sensor-backed
Nav2 composition use `ros2 launch rak_bringup task_navigation.launch.py motion_backend:=sim`.
Current progress and acceptance gaps are tracked in `docs/status/project_status.md`.

Build `rak_interfaces` before `rak_agent`, and `rak_gpu` before `rak_stereo_driver`;
colcon knows the order from package manifests. Treat `colcon build` +
`tests/test_ai.py` as the two checks to pass before claiming done — subject to the
caveats above.

## 运行期编排与生命周期（ADR-0020，2026-10-29）

- 机制层用标准 `lifecycle_msgs` 转移，策略层是 `src/rak_bringup/config/system_manifest.yaml`
  （故障域 / 组件 / 所有权 / 判据的唯一来源），观测层只出标准 `diagnostic_msgs`
  （`/diagnostics` + `/rak/system_state`）。**没有新增 `.msg`**——`rak_interfaces` 是冻结契约。
- 入口：`ros2 launch rak_bringup robot.launch.py stage:=sensors|teleop|nav|full`（stage 决定期望域，
  传感器开关同步剔除对应域）；`ros2 launch rak_bringup supervisor.launch.py [profile:=fixture]`
  只起编排者（fixture 档不碰硬件，用来验证编排本身）；`ros2 run rak_lifecycle rak_system status`
  看每个域的状态与原因。`supervise:=false` 是排障逃生门（回到无编排行为）。
- **两条不可协商的规则**：① 一个组件只有一个生命周期拥有者——Nav2 归 `nav2_lifecycle_manager`、
  slam_toolbox 归 `slam.launch.py` 的事件、控制器归 spawner，`system_manager` 对别人的组件
  **只读不写**；② **激活 ≠ 运动许可**，`allow_motion`（默认 false）仍是唯一许可，
  任何域被拉起都不得产生执行器命令。
- 门禁：`colcon test --packages-select rak_bringup`（清单不变量 + **受管组件在其声明的 launch 里
  必须是 `LifecycleNode`**——漏改一个就是一个静默空转节点）；
  `ROS_DOMAIN_ID=77 python3 -m pytest src/rak_lifecycle/test`（27 项机制与编排测试）。
- 清单里的 `link` 域已是 **CANopen v5**：`/device_container`（ros2_canopen NMT 主站）是该域的
  拥有者（observed/external），本 manager 只读状态，绝不发 NMT 转移。

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

**`rak_harness`** (ADR-0010) is the Jev↔executor arbitration seam plus the voice dialogue loop;
the package `src/rak_harness` exists and as of 2026-10-15 also hosts the **P1 agentic tool loop**
(Claude Agent SDK in a subprocess, `voice_path:=agent|dialogue|auto`). Its target role: task
context, tool whitelist, budget, timeouts, event summaries and failure recovery, talking to a
cloud LLM through a **separate `anthropic` client** pointed at
`https://api.deepseek.com/anthropic` (key from `ANTHROPIC_API_KEY`, never
`src/rak_agent/config.toml`). It emits structured proposals only (waypoint alias / plan
revision / clarification / no-op) and never velocity or joint commands. The arbitration path
itself is still **design-only / needs its ADR** — do not treat it as accepted behaviour.

**`rak_task_executor`** owns the single navigation-task lifecycle: `/task/execute` (Action),
`/task/control` (Service), `/task/status` (status), driving Nav2 `NavigateToPose`. Pause is
only reported after Nav2 cancellation is confirmed **and** fresh repeated stationary telemetry
arrives; resume re-submits the semantic map goal (no replaying old velocities). Its default
parser is local; optional RAI planning may only choose a configured waypoint alias and never
emits velocity or joint commands. Production waypoints live in `config/waypoints.yaml`, which
ships **empty** — the file must declare `schema_version: 1` and `frame_id: map` or the node
refuses to start. The fake-Nav2 fixture's telemetry is a command-motion estimate, not measured
odometry.

**The base path is `rak_chassis` (CANopen 薄桥)** — `controller_manager` →
`mecanum_drive_controller`/`arm_controller` → `CanopenChassis`/`CanopenArm`（OD 对象量化 +
`allow_motion` 门）→ `ros2_canopen` proxy driver → 主站 `device_container`，由
`chassis_bringup.launch.py` 起（`allow_motion` 默认 false）。协议栈不在本仓；旧 UART 节点
(`rak_base_driver`) 与 v4 `CanSession` 都已删除。
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
  (CAN is the only one since ADR-0016) or on a sensor SDK — same-need sensors publish the same
  message set and switch via launch args. rclpy nodes are always `setup.py` entry points,
  never bare scripts. `rak_agent` (the decision layer) must not gain dependents beyond
  `rak_bringup`; task-intent parsing lives in the shared `rak_intent` package, never
  re-implemented per consumer. `check_workspace.py` enforces all of this — fix the structure,
  not the assert.
- **System 1 (Laya) is a direct function call.** Never route it through an orchestration
  framework or a cross-process boundary. The RAI harness belongs only on the System 2
  slow path.
- **`guard()` is the only authority** (deterministic geometric veto, overrides everything).
  `gate()` only decides escalation tier. System 2 may narrow authority, never widen it.
  Laya's confidence is out-of-domain and untrusted — a malformed model answer is a normal
  path (→ LOW → escalate), not an exception.
- **`src/rak_agent/rak_agent/types.py` is frozen once episodes exist.** `state_text` is stored
  verbatim in `data/episodes/*.jsonl` (repo root, ADR-0013); regenerating it silently corrupts
  the finetune corpus. There is deliberately no training pipeline here. `SceneState` uses
  camera x-right/z-forward; ROS body convention is x-forward/y-left — transform explicitly in
  the adapter, never by rewriting the frozen semantics. New task protocols get their own
  version.
- `--decider stub` is the default and stays runnable without a model. The target System 1
  runtime is in-process TensorRT from JetPack; do not add a model service or generic
  CUDA/PyTorch wheel dependency. The TensorRT backend is integrated (`decider:=tensorrt`
  in the node, `--decider tensorrt` in the CLI loop): one factory in
  `src/rak_agent/rak_agent/system1.py` (`make_decider`), engine + tokenizer under the
  gitignored repo-root `data/models/laya-typed-decisions/` (override with
  `RAK_LAYA_MODEL_DIR` or `-p decider_model_dir:=`), runbook `docs/how-to/laya_tensorrt_runbook.md`.
- `rak_task_executor` is the only task/goal lifecycle owner. Pause is not reported until
  Nav2 cancellation is confirmed and fresh repeated stationary telemetry arrives; resume
  submits the semantic map goal again. Its default parser is local; optional RAI planning
  may only choose a configured waypoint alias and never issues velocity/joint commands.
  It consumes the shared `rak_intent` contract — never parse logic private to the executor.
- **The chassis hardware interface stays motion-gated by default.** `chassis_bringup.launch.py`
  keeps `allow_motion:=false` (the hardware only handshakes, reports telemetry and sends zero
  targets); command-interface limits are enforced by `controller_manager`
  (`enforce_command_limits: true`), and the wheel geometry / limits in
  `chassis_controllers.yaml` are unsigned placeholders. Never relax a default to allow motion
  as a shortcut. A launch argument is not firmware compatibility.
- **One lifecycle owner per component (ADR-0020 §3).** Nav2's six servers belong to
  `nav2_lifecycle_manager`, `slam_toolbox` belongs to the event dance in `slam.launch.py`,
  controllers belong to their spawners, and only `rak_*` nodes converted to `ManagedNode`
  belong to `rak_lifecycle/system_manager`. `system_manager` never sends a transition to a
  component it does not own — it only reads state. A second owner produces a node that is
  running, configured-looking and completely inert.
- **Activation is not a motion permission (ADR-0020 §6).** `allow_motion` (default false) stays
  the only way anything reaches the actuators; bringing a domain up must never change that.
  When v5 lands (ADR-0021), NMT `Pre-operational → Operational` is the link layer's own gate
  and the `cmd_seq`/`cmd_flags.bit0` 100 ms deadman stays in the MCU — neither is a substitute
  for `allow_motion`, and neither may be driven by the orchestrator.
- The retired **CAN v3 `serial_link_node`** (git history only since 2026-10-10, was
  `attic/rak_serial_link`) opens hardware unconditionally: its `port` parameter defaults to
  `/dev/wheel_motor` at 921600 and is opened in `__init__` with no `allow_hardware` gate.
  Do not resurrect it from git history onto a machine where that device exists unless you
  mean to own the port.
- The six-axis MoveIt fixture and `rak_arm_simulation` action servers are software-only.
  They contain test geometry/limits and must not be connected to hardware.
- **Two LLM keys, two providers.** RAI's slow path reads `OPENAI_API_KEY` (SiliconFlow,
  `src/rak_agent/config.toml [openai]`); the ADR-0010 Harness client reads `ANTHROPIC_API_KEY`
  (DeepSeek's Anthropic-compatible endpoint). Do not share one variable across providers, and
  `src/rak_agent/config.toml` must never contain either key.
- **Jev does not reach the task executor directly (ADR-0010).** The only new consumer of
  `/brain/decision` is the Harness node, and it is context only — never authority. Jev's path to
  execution is transitive (`/brain/decision` → Harness → `/harness/proposal` →
  `rak_task_executor`), which accepts a proposal only if `task_id`/`revision` match,
  `expires_at` is in the future, and the waypoint is in the configured allow-list. The executor
  must not subscribe to `/brain/decision`, and Harness emits structured goals only — never
  velocity or joint commands.
- **ROS message type identity includes the package name**: `rak_interfaces/msg/Decision` does
  not interoperate with the old `core/msg/Decision`. External nodes must be rebuilt; old
  rosbags need explicit type migration.
- **The target arm is standard six axes plus a servo gripper** and an eye-in-hand camera
  (ADR-0008). The real six-axis model is measured (2026-10-08); the five-axis URDF is a demo
  and `six_axis_sim.urdf.xacro` is test-only — do not present either as the real hardware, and
  do not invent calibration for the unmeasured parts (masses, gripper mapping, eye-in-hand
  extrinsics). Do not configure a real `ros2_control` arm hardware plugin before the Arm Unlock
  Gates in `docs/reference/hardware_acceptance.md` pass. `src/vendor/**` is third-party:
  read-only, do not reformat or "fix" it.
- **CAN protocol ownership.** The wire format's source of truth is the leaf-node repos — since
  2026-10-29 that is **v5 = CANopen**: `../vehical_rak_motion/docs/CANOPEN-V5.md` +
  `docs/canopen/chassis.json`, `../vehical_rak_arm/docs/CANOPEN-V5.md` +
  `docs/canopen/arm.json` — never this repo. The bus is classic CAN 2.0A @ 1 Mbps; never pass
  `fd on` (the F407 bxCAN would error-frame an FD frame). The host side is now **v5 only**:
  `chassis_bringup.launch.py` starts `canopen_core/device_container_node` (NMT master node 127,
  底盘 1 / 臂 2) with `rak_canopen` 的 `bus.yaml` + `master.dcf`，`rak_chassis` 薄桥经 proxy
  driver 的 rpdo/tpdo 话题交换 OD 对象。v4 `PROTOCOL.md`、`CanSession`、`can_link_smoke.py`
  与 `canopen_bringup.launch.py` 都是历史（2026-11-14 删除）。**无下位机时的验收**是
  `scripts/verify_canopen_vcan.py`（vcan0 + fake slave）；真机 can0 已验收（2026-11-17，
  G0–G3，见 `docs/how-to/chassis_canopen_bringup.md` §7）；同一总线永远
  只能有一个主站（双主站会把从站踢出 Operational）。

## Gotchas

- **rclpy 生命周期最容易踩的坑（本轮被测试抓到）**：覆盖 `on_activate` 却忘了
  `super().on_activate(state)` → `LifecyclePublisher` 永远不被启用，而未启用的发布器
  **静默丢弃 publish 且不报错**：节点活着、日志正常、一帧不发。新写 lifecycle 节点时
  每个转移钩子都要转发 super。
- Jazzy 里几个与常见文档不同的名字：`DiagnosticArray` 的字段是 `status`（不是 `statuses`）、
  `DiagnosticStatus` 只有 `OK/WARN/ERROR/STALE`（没有 `STOP`/`STAT`）、`TRANSITION_*` 常量挂在
  `lifecycle_msgs.msg.Transition` 上（`TransitionDescription` 是个消息类型）。这三处本仓都实际踩过。
- `diagnostic_updater` is installed but its **Python** side has no `TopicMonitor`; use
  `TopicDiagnostic`/`FrequencyStatus`/`TimeStampStatus`. `diagnostic_aggregator` is **not**
  installed, so `/rak/system_state` is aggregated by `rak_lifecycle` itself (accepted cost,
  ADR-0020 §后果).
- `ros2 lifecycle set` on a node nobody activates is not a CLI bug: `LifecycleNode`s start
  `unconfigured` and do nothing until configured. Check `ros2 lifecycle get <node>` before
  concluding a node is broken.

- Build files hardcode JetPack paths: VPI at `/opt/nvidia/vpi4`, pkg-config module
  `nvjpeg-13.2`, CUDA headers at `/usr/local/cuda-13.2/targets/sbsa-linux/include`.
  This workspace only builds on the Jetson (aarch64); do not "portable-ize" casually
  without an x86 alternative to verify against. **CUDA 13.2 + CMake 3.28**：`FindCUDAToolkit`
  默认找不到 CUDART，`rak_gpu` 需要额外传 `-DCUDAToolkit_ROOT=/usr/local/cuda-13.2`；
  `rak_pointcloud`（`enable_language(CUDA)`）还需要 `-DCUDAToolkit_LIBRARY_ROOT=/usr/local/cuda-13.2`
  并清掉该包 build 目录才生效（2026-11-14 实测）。
- C++ is C++17 with `-Wall -Wextra -Wpedantic` in GPU/stereo packages. Stereo runs
  `ament_lint_auto` under `BUILD_TESTING`; GPU has no functional tests.
- System 2 latency is 12–30 s per call (measured), so S2 is fire-and-forget in a thread
  and replies are checked on a later tick. Currently `_drain()` only records replies;
  it does not execute them. The S1 callback also precedes safety resolution, so the
  prototype is not an independent real-time safety path. See
  `docs/explanation/architecture.md`.
- `data/`, `build/`, `install/`, `log/` are gitignored. Stray
  `frames_*.gv` / `frames_*.pdf` at the root are debugging artifacts, not deliverables.
- **`colcon build` only ever runs from the repo root** (`--base-paths src`). Run it from `~`
  and it drops orphan `~/build` / `~/install` / `~/log` in the home directory *and* drags
  `~/ws_thirdparty`, `~/vehical_rak_motion` and `~/vehical_rak_arm` into package discovery —
  that is exactly how
  the 2026-10-10 stray artifacts happened (they were deleted; `events.log` there named
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
- `rak_agent` installs `agent_node` as a standard `console_scripts` entry point and runs
  with ROS system Python. Keep Python runtime dependencies in `package.xml`/rosdep rather
  than reintroducing an interpreter bootstrap or project venv.
- The 1024x600 touchscreen and the camera compete for the same pool. A logged-in
  GNOME session drives `CmaFree` to **0 kB** of the 256 MiB CMA area, and the stereo
  capture (`stereo_node`, node `eye_capture`, topics `eye/*` — "eye" is the historical
  name) asks that pool for 4 `V4L2_MEMORY_MMAP` buffers, so capture fails in a way that
  looks like a camera bug but is a memory decision. `scripts/rak-mode.sh headless`
  stops and disables gdm (~1.8 -> ~2.3 GiB available, 75-160 MiB CMA back).
  GUI launchers require `desktop`; migrated sensor/navigation wrappers do not
  require DISPLAY. Autologin is off, so returning to the desktop requires login.
- **`master_config` 必须是 `dcfgen` 生成的主站 DCF**（带 1F80/1F81/1F85 网络表）。拿从站 EDS
  冒充它，`device_container` 能加载但**不发 NMT reset**，proxy driver 1 s Boot Timeout 后
  上游 retry 路径抛未捕获 `std::system_error: Command: Operation not permitted` 直接 abort
  （2026-11-14 实测）。重生成：`scripts/gen_canopen_master_dcf.py`；漂移门禁
  `src/rak_canopen/test/test_dcf_sync.py`。`device_container` **不是** lifecycle 节点，
  `ros2 lifecycle set` 对它无效。从站 `dcf` 必须是相对名（绝对路径会被拼成双路径）。
  物理层自检：`ip -details -statistics link show can0`、`dmesg | grep -i can`。
- **驱动激活（2026-11-15 起）**：bus.yaml 已切**非 lifecycle** 驱动
  （`MasterDriver` / `ProxyDriver`）——`device_container` 在 `init()` 里自己
  configure+activate，**不需要** `ros2 lifecycle set /lifecycle_manager …`（lifecycle
  变体的 6 s/9 s 定时转移已删；那是启动时序赌博）。controller_manager 延到 6 s 起，
  硬件插件另有 startup_grace_ms=5000 宽限兜底。
  **SYNC / TPDO / `~/rpdo` 全链已闭合（2026-11-15 真机，机械臂板 A36）**：
  - SYNC 由主站发，前提是 bus.yaml 写 `sync_period: 2000`（dcfgen → master.dcf 0x1006），
    实测 `080#` 稳定 500 Hz。
  - **`dcfgen` 必须加 `-r`（remote PDO）**：不加时主站 OD 里没有 0x5800/0x5A00，
    lely 不把从站 TPDO 当 RPDO 写入，`~/rpdo` 一帧没有（现象极具误导性：帧在线上
    50 Hz、SDO 正常、驱动 `Driver booted and ready`，但话题空）。
    `scripts/gen_canopen_master_dcf.py` / `test_dcf_sync.py` / `verify_canopen_vcan.py`
    都已加 `-r`，别再省。
  - **`device_container_node` 需要 `CAP_NET_ADMIN`**（lely 走 netlink 改链路属性）：
    普通用户跑直接 `CanController: Operation not permitted`。用 root 跑；
    **不要**给二进制加 file capability —— secure-exec 会丢掉 `LD_LIBRARY_PATH`，
    连 `libdevice_container.so` 都找不到（exit 127）。
  - **单板联调必须同时裁两处**：`slaves:=chassis|arm|both` 同时裁 bus.yaml、
    URDF 的 `<ros2_control>` 段与对应 spawner。只裁 bus.yaml 的话另一台的 hardware
    仍会被激活，永远收不到反馈 → `read()` ERROR → 连 joint_state_broadcaster 一起停。
  - 实测：`/arm/rpdo` 505 Hz，`/joint_states` 真实读数。
    **底盘板（node 1）2026-11-17 已上线并全链验收**（端子重压后 G0–G3 全过，
    见 `docs/how-to/chassis_canopen_bringup.md` §7；此前 11-15/11-16 的"不在总线"
    是物理层接触不良，SWD 曾证明固件本身在跑）。
- The lidar `lidar_name` must be `N10_P` (108 B @ 460800); `N10` fails CRC. Use the udev alias
  `/dev/wheeltec_lidar`, never `/dev/ttyUSB0` (other serial adapters may also claim ttyUSB).
- **2026-11-17 装车实测：轮子"跑久就卡/走很乱"的根因在固件极性表，不在主机侧。**
  `encoder_sign` 由固件"kick 自检"学习后存进 Flash（sector 11 / `0x08060000`，
  append-only 记录），而 R38 的矛盾自检只要 `duty>40%` + 1 ms 窗增量反向就**立刻翻**
  —— 车顶墙/受憋时窗增量是噪声，于是撞一次墙就把极性翻坏并落盘，重启也带着错的：
  表象是四轮有反馈但车不走、或左右互顶打滑。2026-11-17 已打两处补丁（`openocd
  program` 实测 **Verified OK**）：**R77** —— 矛盾必须**连续 5 拍(50 ms)且滤波转速
  ≥10 counts/ms** 才翻；再加"有电不转 2 s 就把该轮极性清 0 重学"的兜底。
  **R78** —— `app_sign_store.h` 的 `SIGN_STORE_SECTOR` 原来是 `FLASH_SECTOR_11`，
  但本板是 **512 KB F407，`0x08060000` 落在 sector 7**：擦除永远失败 → 记录只能写进
  最初几十个空槽，之后再也存不进去，每次开机都载入**最早那条**（往往是错的），
  而且"已有记录"让 kick 自检不再重学。改 `FLASH_SECTOR_7` 后实测记录写入成功
  （`magic=0x53474E31 seq=1 signs=(-1,1,1,-1)`）且**跨复位保持**。
  实测正确极性 = **`[-1,+1,+1,-1]`（FL/RR 反、FR/RL 正）**，主机侧对应 `state_sign`
  （`rak_chassis.urdf.xacro`），命令侧 `wheel_sign` 仍全 `+1`。**SWD 读符号前必须
  重新 `nm` 解析地址**（重编译后变量会移位，本轮因用旧地址读出过"estop=200"这类鬼值）。取证套路：`openocd … -c "mdw 0x20009bc8 1"` 读 `encoder_sign`、
  `0x20009c74` `wheel_speed`、`0x20009bd0` `duty_q8`；**`st-flash read` 会 halt 核**，
  读完必须 `reset run`（本轮踩过，OLED/CAN 全停但看着像固件坏了）。
- **两块 IMU 的轴/朝向都实测过（2026-11-17）**：
  板载（chassis F407）`s_acc_mg=[35,3.6,956] mg` → 重力沿**芯片 +Z**，模块是**平装**；
  R76 的倾角算法按"竖装、重力 +Y"写，实测 pitch=−89.8°/roll=+84.2°（垃圾）。改成
  `pitch=atan2(-ax,az)`、`roll=atan2(-ay,az)` 后实测 **−2.14°/−0.21°**（R78）。
  Jetson 直连 LSM6DSR 正常：`axis_map:="-z,-x,y"` 下 `acc=(0.02,0.15,9.84)`、
  `|a|=9.84`、gyro≈0 —— **但 `i2c_imu` 是 lifecycle 节点**，`ros2 run` 起来是
  unconfigured，必须 `ros2 lifecycle set /i2c_imu configure` + `activate` 才发
  `/imu/data_raw`（正常由 `robot.launch.py` 的 system_manager 拉起）。
- **2026-11-18 装车实测：三处会让底盘"轮子转但车不走/飞转"的根因**
  （都已修，别回退）：
  ① **`bus.yaml` 的 `chassis.polling` 必须是 `false`**。ros2_canopen 的 `ProxyDriver`
  在 `polling: true` 时用 SDO 周期读把 OD 写回它自己的缓存，**覆盖掉 TPDO 收到的真值**。
  现象极具误导性：`candump can0,281:7FF` 位置字节一路增长，而 ROS `/chassis/rpdo` 的
  `0x2020` 恒定不变（实测 3739004 卡死），`~0x2010` 恒为 −1；底盘因此永远里程计为零。
  TPDO 在 500 Hz SYNC 下已稳定，不需要轮询。
  ② **URDF 轮关节 `state_sign` 必须是 `1`**。固件 `car_wheel_positions()` 里已经
  `out[i] = position_counts[i] * encoder_sign[i]`，OD `0x2020..` 出来就是"正指令 → 正
  counts"（悬空实测四轴增量 FL=+582816 FR=+610134 RL=+593716 RR=+583507）。主机再用
  `-1` 翻一次等于抵消，直走被算成自转。改了 URDF 必须重编 `rak_chassis` 插件 ——
  **旧 `.so` 不认 `state_sign`，会继续按老逻辑翻转**（本轮踩过：URDF 全 `1` 了但
  `/joint_states` 仍是 `-,+,+,-`）。
  ③ **固件不要再"自动改极性"**。R77 运行期翻转、R78「有电不转就清 `encoder_sign`」两条
  都删了：架空/打滑/被顶住时它们同样触发，清 0 会让
  `car_motor_output_gate` 走 `encoder_uninitialized` 分支 → **75% 开环直驱**，轮子飞转且
  极性学不回来（实测 Flash 里攒出 seq=1..8 七条互相矛盾的记录，开机载入最后一条=错的）。
  极性只保留两个来源：开机从 Flash 读（人工标定一次）、`encoder_sign==0` 时 kick 自学。
  恢复办法：`openocd … -c "flash erase_sector 0 7 7"`，再
  `mww <encoder_sign> 0xFF0101FF`（= `[-1,+1,+1,-1]`）后 `resume`。
- **2026-11-18 底盘速度环标定：PI 其实跑在 1 kHz，不是 25 kHz。**
  `car_control_isr()` 里整段 PI 在 `if (++win_cnt >= 25U)` 内 —— 每 25 个 40 µs 节拍
  才跑一次，即 **1 ms 一次**。旧增益 `e/2 + e/4`（0.75 Q8/ms）换算成物理量只有
  **0.85 %/s**：误差 30 counts/ms 时占空比 1 秒才涨 0.9%，爬满 85% 要 100 秒。
  表象是"指令 0.12 m/s 只走出 0.04 m/s（30% 跟踪）"，且误差越大约明显。
  修法（R81）：改 `SPEED_KI_NUM/DEN = 1/2`（e_q8/2 每毫秒，满误差 0.5 s 到限幅），
  并把 `pi_scale` 改成"距上次 PI 的毫秒数"（ISR 被延迟时物理增益不变）。
  实测：0.10 m/s 跟踪 98%（轮系）/88%（雷达）。
- **2026-11-18 `wheels_radius` 实测回填 0.0275 → 0.0309**。0.0275 是轮盘本体半径，
  mecanum 实际滚在**滚子外缘**。用 0.0275 时四轮 19.38 rad 只算 0.533 m，而雷达
  0.599 m、深度 0.601 m 两路独立吻合 → 有效半径 0.0309。回填后再次实测：
  轮系 0.523 / 雷达 0.519 / 深度 0.525 m，**轮系/雷达 = 1.007**，里程计与双传感器
  真值一致（此前差 11%）。四轮转角同号且极差 <6%，说明是直线不是自转。
- **2026-11-18 IMU 功能升级与航向保持（两项收尾）**：
  ① **Jetson 直连 IMU 会"数据冻结"**：LSM6DSR 偶发掉回 power-down，但 I2C 仍 ACK、
  寄存器回读正常（i2c_err=0），驱动只能从**数据**发现 —— 实测连续 30 s 全 0，把
  EKF 的 vyaw 全程喂垃圾。修法（R82）：`i2c_imu.py` 加看门狗，连续 50 帧同一 12 字节
  即重写 CTRL1_XL/CTRL2_G 并报 WARN（实测 2.5 s 内触发一次并自愈，12 s 观测零帧
  比例 0.00%）。**多次 ros2 run 会叠多个 i2c_imu 实例**（本轮叠到 3 个，频率统计全废），
  起节点前必须 `pgrep -af i2c_imu` 清点。
  ② **陀螺零偏 2026-11-18 重标定**：`0.001186,-0.007734,-0.003310` rad/s
  （60 s 静置、35786 帧、az=9.7182 确认未冻结；标准误 ~0.001 °/s）。旧值
  （2026-10-29）Z 轴差 0.155 °/s ≈ 10 min 56°，是"航向不可信"的主要来源。
  ③ **底盘直行航向漂移 −2.0°/m（无稳定器）→ +0.42°/m（开）**：固件已有轮域稳定器
  （R74，`cmd_flags.bit1` 门控 + 板载 IMU yaw-rate 阻尼/航向保持），默认关。
  打开 = launch 加 `stabilizer:=true`（URDF → bridge → flags bit1 → MCU）。
  实测 0.453 m 内漂 +0.19°（改善 ~5 倍）；速度跟踪不受影响（稳态 93–107%）。
  ④ rf2o 激光里程计**位姿与航向不自洽**（位移方向 +37.9° = lidar_yaw 量值，航向差
  177°；航向**变化量**准）：接 EKF 前必须只取位姿、禁 yaw（ekf.yaml 已按此设计）。
- **排查固件控制量的正确姿势（2026-11-18）**：不要用 `st-flash read` 逐次抓
  （每次 halt 核会把 ISR 记账打断，曾据此误判"ISR 只有 1 kHz"，真实是满速 25 kHz）。
  固件里有 `car_dbg_mirror[12]`（seqlock：`[0]` 奇数=写入中）：
  `openocd -c init -c halt -c "mdw <mirror> 12" -c resume -c shutdown`，
  字段 `[v_cmd v_ramp duty(0.01%) speed(counts/ms) e_q8 ff scale isr_tick gap_cyc gap_max phase|estop<<8]`。
  `gap_cyc` 应恒为 **6720**（168 MHz × 40 µs）；`scale` 应恒为 **1**。
- **`0x2010..0x2013`（轮速反馈）会冻结在旧值**，`0x2020..0x2023`（int32 编码器计数）
  一路正确。所以 `CanopenChassis` 现在用 `velocity_from_position:=true` 由位置差分
  出速度，`jr` 的 `aux_state_*` 把位置当第二路状态导出（否则 joint_states 位置是 NaN、
  车轮 TF 被 robot_state_publisher 丢弃、控制器只能积分速度算里程计）。
- **2026-11-18 五个"静默接不上"的坑**（细节与门禁见 `docs/status/project_status.md` 同日增量）：
  cyclonedds 网卡名写死会让每个 ROS 进程 abort（现由 CMake 构建期按本机网卡替换）；
  `robot_localization` 输入槽必须从 `odom0` 连续；协方差列表 int/float 混排让 `ekf_node`
  启动即 abort；`ManagedNode` 不要在 `main()` 里自己做生命周期转移；
  `pgrep -af` 会把 `ros2 run` 包装器与真节点数成两个。
- **两套安装树并存会静默喂旧 URDF**：`install/` 与 `install/architecture/` 同时存在时，
  launch 里的 `get_package_share_directory` 可能取到旧那份（本轮表现为改了 `sign`
  但线上一直是旧值）。改 URDF 后要么两棵树一起 build，要么确认 `AMENT_PREFIX_PATH` 顺序。
- Simulation task telemetry is a command-motion estimate, not measured odometry. A
  successful software pause/resume test is not hardware acceptance; see
  `docs/reference/hardware_acceptance.md`.
- **`nvpmodel` is already at mode 0 = 10 W, this board's *highest* mode** (the only other
  modes are `7W_AI` and `7W_CPU`), so there is nothing to raise. `nvpmodel.service` runs
  at boot, but `jetson-clocks.service` is enabled and has **not** run (inactive, no
  journal entries): the governor is `schedutil` and CPU clock floats 729–1510 MHz. Do not
  assume clocks are locked, and never compare benchmarks across power/clock states — the
  ~158 ms/predict figure and eye's 30 fps are 10 W-mode numbers.
- **Memory is zram + swap now** (`docs/tutorials/construction.md` §2.2, not in git): 1.7 G zram plus
  a 4 G swapfile. `free` looks less alarming than it should, and swap/zram are not
  real-time guarantees; whether zram helps on a 4 GB board is contested (TODO P2-1) —
  measure `si/so` and p99 latency rather than reverting from documentation.
- The historical PyTorch/Laya path cost **~1.4 GB of this box's 3.4 GB unified memory**
  on the GPU: 804 MiB of
  resident bf16 weights (421M params) plus ~600 MiB CUDA context / cuDNN / cuBLAS
  workspace, at 158 ms/predict. It once needed ~1.9 GB for nothing: `laya.load()` built
  the module in torch's *default* dtype (fp32) and `load_state_dict` copied up into those
  parameters, so the weights sat at 1606 MiB — and on Tegra CUDA's "free" is just the
  pages nobody is holding, so the fp32 build peaked CPU-side and starved the GPU at the
  same moment, making `laya.load(model)` OOM and fall back to CPU before any cast could
  run. `LayaDecider` therefore loads on CPU with the default dtype already set to the
  checkpoint's `amp_dtype` (bf16) and places afterwards, so fp32 never exists.
  These are historical measurements, not the target TensorRT runtime; do not install or
  select this legacy backend for current Jetson runs.
- Half weights only run where autocast is on, and laya only turns it on for CUDA
  (CPU bf16 is the documented `LAYA_CPU_AMP=bf16`). Without it the forward dies in
  `F.linear`: "mat1 and mat2 must have the same dtype, but got Float and
  BFloat16" -- fp32 activations against half weights. The legacy `LayaDecider` degraded to
  fp32 without CUDA; do not use it for current Jetson runs.
- If it does not fit, `laya` catches the CUDA OOM itself and drops to CPU at
  ~1.6 s/predict -- slower than the sensor rate, with no exception raised.
  `LayaDecider.device/.weights/.place_error` and the node's `s1_device=`/`s1_weights=`
  startup fields describe that legacy implementation; do not run `decider:=laya`.
- The legacy PyTorch/Laya fp16 parity run compared executed actions and safety tiers over
  90 states. It does not qualify TensorRT: the engine's own acceptance (token layout exact,
  numerics within bf16, argmax identical, same `Decision` contract) is recorded in
  `docs/how-to/laya_tensorrt_runbook.md` §Acceptance and pinned by
  `src/rak_agent/test/test_tensorrt_decider.py`; a *new* engine needs that pass again.
- **Wheel polarity: the firmware owns it — final answer 2026-11-18.** The 2026-10-14
  conclusion ("left pair FL/RL wired inverted, fix it host-side with `wheel_sign=-1 1 -1 1`")
  is **void**. Command side: with all `+1` directives the base drives straight (visual +
  wire re-tests 2026-11-16/17), so the firmware directive table
  (`vehical_rak_motion/Core/Inc/wheel_config.h`, `WHEEL_FWD_DIR_LIST`) stays all `+1`
  and the host must **not** stack a second sign on top. URDF defaults and
  `chassis_bringup.launch.py wheel_sign:=` are therefore `1 1 1 1`; the per-joint
  `sign` mechanism still exists (negates command **and** feedback together) only as an
  override after a re-wiring. Feedback side: the firmware `car_wheel_positions()` already
  multiplies by `encoder_sign` (true values `[-1,+1,+1,-1]`, Flash record `seq=1`,
  survives `reset run`), so OD `0x2020..` arrives as "positive command → positive
  counts" and the host `state_sign` **must stay 1** — the previous host-side `-1`
  flipped it a second time, which is exactly why straight driving was metered as
  spin-in-place (measured in air 2026-11-18: all four wheel deltas positive
  FL=+582816 FR=+610134 RL=+593716 RR=+583507). `canopen_contract.yaml` records
  `[FL,FR,RL,RR] ↔ W1..W4` and points at the firmware for values — do not restate the
  numbers in a third place. Re-calibrate after any wheel, motor or encoder change.
- **历史坑，别再复活**：v4 时代 `read()` 曾把 `fault_bitmap` 里 `cmd_stale`（bit3）也当反馈否决，
  自锁成"控制器没激活 → 死区 → read 报错 → CM 停用硬件"。ADR-0017 定的是"只对真正的故障位否决"，
  这条原则在 v5 桥里延续成：**每块板自己的反馈超时才会 ERROR**，`cmd_seq`/`cmd_flags` 活性由 MCU
  判死（100 ms），主机侧不重复当故障。
- **Never run two `chassis_bringup` instances.** Both put a node named
  `/controller_manager` on the domain, so the second launch's `spawner`s reach the first
  one's manager and die with `Failed to configure controller` / `Failed loading
  controller` (its YAML was parsed long ago and has no `arm_controller` or
  `gripper_controller`). Check with
  `ps -eo pid,etime,cmd | grep chassis_bringup` — it must print exactly one line — before
  launching.
- **语音栈现状（2026-10-17）**：ASR 默认引擎是流式 Zipformer（14M，`acoustic/models/
  sherpa-onnx-streaming-zipformer-zh-14M-*`，sherpa-onnx，`engine:=zipformer`；Vosk 保留为
  `engine:=vosk` 回退）。**TTS 默认三层（ADR-0018，已接受）**：即时层播预合成素材
  （`acoustic/models/quick/*.wav`，由 `scripts/make_quick_phrases.py` 生成，清单
  `src/rak_speech/config/quick_phrases.toml`，`/speech/quick` 按 key 直出，无推理）；
  内容层默认 `backend:=qwentts`——qwentts.cpp 的常驻 `tts-server` + speaker `serena` +
  24 kHz 流式 PCM（首帧 ~0.5–1.0 s，整句 RTF 3.5–5.6，CPU；`qwentts_max_seq` 默认 2048
  = 448MB KV）；单句超预算丢弃并降级为即时素材。备选 `backend:=sherpa`（kokoro CPU）与
  `backend:=cosyvoice`（GPU CosyVoice3-0.5B，情感 instruct 经 Say.srv）仍在，但 cosyvoice
  已证内存不可行。qwentts 的 GPU 路径被上游 sm87 bug 堵着（qwentts.cpp#40），只用 CPU。
  用户 site 已装
  torch 2.12.0+cu132（R39.2 原生可用，sm_87 走 sm_80 回退）与 onnxruntime-gpu 1.28
  （TensorRT EP 可用；numpy 钉 1.26.4，因 ORT-GPU/pyarrow 的 2.x 需求与系统栈互斥）。
  **GPU 堆注意事项**：反复 cudaMalloc OOM 被杀会留下坏堆（>1GB 分配必失败，重启即愈）；
  CosyVoice3 推理需 ~1.7GB 显存，桌面会话在跑时仅 ~1.3GB 可用——跑它先 headless。
  qwentts 内容层常驻约 1.2–1.5GB 内存（Q4_K_M 权重 + 448MB KV + codec），跑之前同样先
  headless。agent loop 的 SDK 必须 `permission_mode="bypassPermissions"`：ark-code-latest 在
  acceptEdits 下会自行脑补"等弹窗放行"而拒绝调用运动工具；权威始终在执行器侧，不变。

## Docs

`docs/` is Diátaxis-shaped since 2026-11-14 — `tutorials/`（照着跑一遍）、`how-to/`（完成一件事）、
`reference/`（字段与门限的事实）、`explanation/`（为什么这样设计）、`design/`（未落地的草案）、
`status/`（带日期的进度）、`archive/`（只读旧快照）、`adr/`（**不可变**，只许修正链接与状态行）。
每份文档前 12 行必须有 `> 状态：…` 行，并且收录进 `docs/README.md` 的索引；新增文档两份一起改，
否则门禁失败。决策类内容先写 ADR，不要塞进 `explanation/`。门禁：
`python3 scripts/check_docs.py`（链接可达 + 索引覆盖 + 状态行 + ADR 索引，同时链接检查
`README.md`、本文件与 `TODO.md`）。

## Project skills

Project-level Agent Skills live in `.agents/skills/` (installed with
`npx skills@latest add <owner/repo@skill> --agent '*' -y`, recorded in `skills-lock.json`,
symlinked into the agent dirs). Installed set as of 2026-11-14: `documentation-and-adrs`,
`technical-writing`, `code-review`, `code-review-and-quality`, `ros2`, `ros2-cli`,
`robotics-testing`, `canopen-integration`, `stm32-freertos-developer`, `stm32-baremetal`,
`stm32-debug`. The skill payload itself is **not in git** (`.agents/skills/` is ignored,
`stm32-debug` alone is 2.9 MB of vendor SVD); restore it from the lockfile with
`npx skills@latest experimental_install --agent '*' -y`, and commit `skills-lock.json`
whenever the set changes. Use `documentation-and-adrs` + `technical-writing` for docs/ADR work,
`canopen-integration` + the `stm32-*` skills for firmware work, and
`code-review` / `code-review-and-quality` before merging. Nothing here auto-runs — an agent
has to load a skill explicitly; installing one changes no behaviour by itself.
