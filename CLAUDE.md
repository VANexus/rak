# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Orientation

Layered embodied-robot workspace for Jetson Orin Nano / ROS 2 Jazzy: perception + decision on
the Orin, real-time control on **one** STM32F407 leaf node (see the topology change below).
Read these before changing behavior — they record decisions the code does not show:

- `README.md` — engineering entry point (Chinese): layout, build, launch entries, hardware.
- `docs/architecture.md` — package boundaries, target runtime architecture, acceptance stages,
  and an explicit list of gaps that are **not** closed.
- `docs/project_status.md` — what is actually done vs. only claimed (dated; read the date).
- `docs/CONSTRUCTION.md` — assembly order and deployment checklist; host-side board changes.
- `docs/adr/` — **immutable** architecture decisions (why, and what was rejected). Index in
  `docs/adr/README.md`. `ADR-0007` is the current control architecture and **supersedes ADR-0001**.
  `ADR-0010` defines the Jev↔Harness arbitration seam and the reserved "呼吸感" expression
  interface; it is **design-only** — `src/rak_harness` / `harness_node` do not exist yet.
- `docs/hardware_acceptance.md` — gates that must pass before any physical base/arm test.
- `docs/migration.md` — the 2026-10-04 unified `rak_` package rename (old name → new name table).
- `TODO.md` — prioritized roadmap (P0–P3). Tasks only; rationale lives in the ADRs.
- `AGENTS.md` — parallel agent-instruction file. It overlaps this one but is **not** identical;
  keep both in sync when you change repository-wide guidance.

**Trust hierarchy when docs disagree.** The 2026-10-04 docs (`README.md`, `docs/architecture.md`,
`docs/project_status.md`) describe the code as it stood then (UART v1 host driver, two-MCU CAN
topology). The 2026-10-05 docs (`TODO.md`, `docs/CONSTRUCTION.md`, `docs/adr/0007`) describe the
**accepted target** after the hardware changed. Neither is wrong, but the target is not yet
implemented. Do not treat a 2026-10-04 status statement as current, and do not implement the
2026-10-05 target as if the code already supports it.

## The 2026-10-05 topology change (most important context)

The arm board burned out. What changed:

| | Before (ADR-0001, 2026-10-04) | Now (ADR-0007, 2026-10-05) |
|---|---|---|
| MCUs | **two** F407 (base + arm), CAN-interconnected | **one** F407, drives 4 wheels **and** 7 servos (6 arm joints + 1 gripper) |
| Jetson↔base | UART 921600 | **USB**, the only link |
| CAN | base↔arm classic 2.0A 1 Mbps | **dead** (peer gone) |
| Mecanum IK | on the F407 | **moves to the host** (`mecanum_drive_controller`) |
| Heading authority | F407 onboard IMU | **Jetson's LSM6DSRTR (SPI)** feeds the EKF; F407 IMU is wheel-level only |

**Accepted target (ADR-0007, not yet built):** the whole robot goes `ros2_control` — a
`hardware_interface::SystemInterface` plugin named `rak_hardware` exposing 4 wheel velocity
commands + 7 servo position commands (6 arm joints + 1 gripper) over USB, with `mecanum_drive_controller` and
`joint_trajectory_controller`. **`src/rak_hardware` does not exist yet.** The current code
predates this: `rak_base_driver` (UART v1, body-velocity `(vx,vy,wz)` protocol),
`rak_can_driver` and `rak_serial_link` (CAN v3 stack). Per `TODO.md` P0-3 and ADR-0007 待闭合项
#8, the CAN packages are **deprecated / reserved** — they exist but their semantics are void.
Do not build new work on the CAN path or assume `rak_base_driver` survives ADR-0007.

Two facts that are easy to get wrong:
- **Wheel-level saturation belongs to the MCU.** `mecanum_drive_controller` only clamps at the
  body-axis level (`linear.x/y`, `angular.z`) and does *no* per-wheel saturation. The MCU must
  scale all four wheel speeds by one factor when any exceeds the limit — never clamp axis-by-axis,
  or diagonal driving becomes straight driving (ADR-0007 D6).
- **`enable_odom_tf: false` and an explicit `reference_timeout`** are required once the controller
  exists, or it fights the EKF for `odom→base_link` (ADR-0002/0007 D4).

## Layout

`colcon build` runs from the repo root and picks up both `src/` and `brain/`.

| Package | Contents |
|---|---|
| `src/rak_interfaces` | Messages/actions/services only (`TrackedObject(s)`, `Action`, `Decision`, `ExecuteTask`, `TaskControl`, `TaskStatus`, `BaseTelemetry`, `SendAtomicAction`). No GPU/model deps |
| `src/rak_gpu` | CUDA / TensorRT / VPI RAII utilities. JetPack-only. Export target `rak_gpu::gpu` |
| `src/rak_stereo_driver` | V4L2 + nvJPEG stereo capture. Component `rak::stereo_driver::CaptureNode`, exec `stereo_node` |
| `src/rak_astra_driver` | Orbbec Astra depth/color bridge (OrbbecSDK 1.10.18). Exec `astra_node` |
| `src/rak_sensor_bringup` | Device launch + config: `lidar`, `astra`, `stereo` |
| `src/rak_description` | URDF/xacro, meshes, RViz. Legacy five-axis demo **plus** an isolated six-axis sim fixture |
| `src/rak_navigation` | SLAM, odometry sources, Nav2: `slam.launch.py`, `nav2.launch.py` |
| `src/rak_task_executor` | **Sole owner** of the navigation-task lifecycle: `/task/execute` Action, `/task/control` Service, `/task/status`; execs `task_executor`, `task_cli`, `fake_navigation` |
| `src/rak_base_driver` | Jetson→F407 UART v1/v1.1 host codec/node (exec `base_uart_node`, node `base_uart_driver`). Hardware-disabled and DISARMED by default. Slated for replacement by `rak_hardware` (ADR-0007) |
| `src/rak_moveit_config` | SRDF, IK, planning config: `demo.launch.py` (legacy five-axis) + `six_axis_sim.launch.py` (fixture). No real trajectory execution path |
| `src/rak_arm_simulation` | Software-only six-axis trajectory/gripper Action servers (exec `fake_joint_controllers`). No physics, no hardware interface |
| `src/rak_motion_stub` | Test-only `/cmd_vel` → `/cmd_vel_raw` relay (`cmd_vel_stub`). Not a hardware driver |
| `src/rak_can_driver` | CAN v3 chassis host stack (python-can, exec `chassis_can_node`). **Docs mark it deprecated/reserved** after the topology change; the code is not annotated yet |
| `src/rak_serial_link` | CAN v3 frames over UART (execs `serial_link_node`, `serial_link_cli`). Same deprecated/reserved status |
| `src/rak_bringup` | Integration entry (`agent.launch.py`, `task_navigation.launch.py`, `base_uart.launch.py`), DDS config, operator scripts |
| `brain/` | Decision layer. `ament_python` ROS package `rak_agent`, Python module `rak_ai`; system Python runtime |
| `src/vendor/*` | Leishen lidar + `rf2o`. Third-party, **read-only** |

`brain/` stays at the repo root to preserve the source and episode-data location. It is
built and installed by colcon as a standard ROS package; it has no dedicated venv or
inference service process. Its `COLCON_IGNORE` is gone deliberately — do not re-add it.
`depth` / `mind` / `world` / `forge` / `nerve` are roadmap names; they do not exist here.

## Build & verify

Requires the on-board JetPack SDK; build files hardcode VPI `/opt/nvidia/vpi4`,
pkg-config `nvjpeg-13.2`, CUDA `/usr/local/cuda-13.2/targets/sbsa-linux/include`.
From the repo root in a shell with only `/opt/ros/jazzy` sourced:

```bash
source /opt/ros/jazzy/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build \
  --base-paths src brain --executor sequential \
  --build-base build/architecture --install-base install/architecture \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/architecture/local_setup.bash

python3 scripts/check_workspace.py --installed        # package/resource/launch/message checks
colcon test --build-base build/architecture --install-base install/architecture \
  --packages-select rak_description rak_moveit_config
colcon test-result --test-result-base build/architecture --verbose
ROS_DOMAIN_ID=77 python3 -m pytest src/rak_task_executor/test src/rak_arm_simulation/test
ROS_DOMAIN_ID=77 PYTHONPATH=src/rak_base_driver python3 -m pytest src/rak_base_driver/test
PYTHONPATH=brain python3 brain/tests/test_ai.py
PYTHONPATH=brain python3 brain/tests/test_task_intent.py
```

Install package dependencies with rosdep. Do not create a project venv or install generic
CUDA/PyTorch wheels. The target in-process TensorRT decider is not implemented until the
ONNX and tokenizer contract is verified.

Use `build/architecture` + `install/architecture`, never the old `build/` / `install/`
artifacts. Decision-layer-only work skips the GPU packages: `--packages-up-to rak_agent`.
ROS integration tests need the install overlay sourced and an isolated `ROS_DOMAIN_ID`;
they connect only to fake action servers and must never touch real UART/CAN. Runtime
operator scripts use Cyclone DDS on domain 47 by default; test domains 77/187 stay isolated.
Treat `colcon build` + `brain/tests/test_ai.py` as the two checks to pass before claiming done.
There is no CI, linter, formatter or pre-commit config in this repo.

The ROS node runs directly under the ROS system Python. `package.xml` declares
`python3-pydantic`; JetPack provides the TensorRT Python binding.

## Entry points

```bash
ros2 launch rak_sensor_bringup lidar.launch.py
ros2 launch rak_sensor_bringup astra.launch.py
ros2 launch rak_sensor_bringup stereo.launch.py device:=/dev/video0
ros2 launch rak_navigation slam.launch.py [odom_source:=rf2o|ekf|none]
ros2 launch rak_navigation nav2.launch.py            # includes lidar + SLAM + motion stub
ros2 launch rak_task_executor task_simulation.launch.py   # fake Nav2 + estimated telemetry
ros2 run rak_task_executor task_cli                  # "go to <waypoint alias>", pause/resume/cancel
ros2 launch rak_bringup task_navigation.launch.py motion_backend:=sim   # default sim
ros2 launch rak_bringup base_uart.launch.py          # hardware_enable=false; port not opened
ros2 run rak_base_driver base_uart_node
ros2 launch rak_moveit_config demo.launch.py         # legacy five-axis demo
ros2 launch rak_moveit_config six_axis_sim.launch.py # six-axis fixture
ros2 launch rak_description view_urdf.launch.py
ros2 launch rak_bringup agent.launch.py decider:=stub sim:=true
ros2 run rak_agent agent_node --ros-args -p sim:=true -p decider:=stub
ros2 run rak_stereo_driver stereo_node
sudo scripts/rak-mode.sh status | headless | desktop   # board memory/DMA mode
```

These are independent entries — do not start them all. `nav2.launch.py` already contains
lidar + SLAM; the arm demo's static `world → base_link` cannot be merged with the mobile
base TF. Never run two Nav2 instances or two `/cmd_vel` end adapters. `task_navigation.launch.py`
with `motion_backend:=hardware` only selects the host UART adapter — `hardware_enable` is still
false, so the device is not opened; a parsed launch is not firmware compatibility. Operator
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

**`rak_agent`** (`brain/rak_ai/`) is the decision layer, built as the `ament_python` ROS
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
- `task_intent.py` — deterministic text-intent parsing (waypoint alias allow-list) used by
  `rak_task_executor`; a **separate** path from `resolve()`.
- `loop.py` — no-ROS tick loop (`sense → guard → decide → escalate → act → log`), writes
  JSONL episodes to `brain/data/episodes/`. `body.py` — `Body` protocol + `MockBody`
  scenarios. `types.py` — the frozen S1/S2 contract.
- `agent_node` is a standard `console_scripts` entry point. Runtime dependencies are
  installed as ROS/system packages; the node does not switch interpreters.

**`rak_harness`** (ADR-0010) is the **planned** slow-path task system — a standard ROS 2 package
`src/rak_harness` with node `harness_node`, started beside `agent_node`. It owns task context,
tool whitelist, budget, timeouts, event summaries and failure recovery, and talks to a cloud LLM
through a **separate `anthropic` client** pointed at `https://api.deepseek.com/anthropic`
(key from `ANTHROPIC_API_KEY`, never `brain/config.toml`). It emits structured proposals only
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

**`rak_base_driver`** implements the proposed UART v1 host protocol (codec, 100 Hz command
tick, telemetry, explicit ARM service) and is **DISARMED by default**. Its fate after ADR-0007
is an open item (ADR-0007 待闭合项 #7). **`rak_arm_simulation`** provides software-only
six-axis trajectory/gripper Action servers for the MoveIt fixture — no physics, no hardware.

**`rak_navigation`** chains the de-facto bringup:
`lidar → laser_filters → rf2o → slam_toolbox (/map, map→odom) → nav2 → /cmd_vel → cmd_vel_stub → /cmd_vel_raw`.
The stub forwards Nav2's unicycle `Twist` verbatim. `odom_source` is the seam for odometry
sources: `rf2o` today; `ekf` is reserved and does **not** auto-start an EKF (you must supply
`odom→base_link` yourself, and set rf2o `publish_tf: false` when you do, or both publish it).

## Constraints that are easy to break

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
- **`brain/rak_ai/types.py` is frozen once episodes exist.** `state_text` is stored verbatim
  in `brain/data/episodes/*.jsonl`; regenerating it silently corrupts the finetune corpus.
  There is deliberately no training pipeline here. `SceneState` uses camera x-right/z-forward;
  ROS body convention is x-forward/y-left — transform explicitly in the adapter, never by
  rewriting the frozen semantics. New task protocols get their own version.
- **`rak_task_executor` is the only task/goal lifecycle owner**; do not add a second Nav2 or
  a competing `/cmd_vel` writer.
- **`BaseUartNode` (and the future hardware interface) stays DISARMED by default.** No port,
  `allow_hardware=false`, and three speed limits that must all be finite positive before ARM —
  `0.0` is not "unlimited/pass-through", and non-zero Twist is rejected until armed. Never
  relax a default to ARM as a shortcut. A launch argument is not firmware compatibility.
- `--decider stub` is the default and must stay runnable without a model. The target System 1
  runtime is in-process TensorRT from JetPack; do not add a model service or generic
  CUDA/PyTorch wheel dependency. The TensorRT decider is not implemented yet.
- **Two LLM keys, two providers.** RAI's slow path reads `OPENAI_API_KEY` (SiliconFlow,
  `brain/config.toml [openai]`); the ADR-0010 Harness client reads `ANTHROPIC_API_KEY`
  (DeepSeek's Anthropic-compatible endpoint). Do not share one variable across providers, and
  `brain/config.toml` must never contain either key.
- ROS message type identity includes the package name: `rak_interfaces/msg/Decision` does not
  interoperate with the old `core/msg/Decision`. External nodes must be rebuilt; old rosbags
  need explicit type migration.
- The target arm is **standard six axes plus a servo gripper** (a 7th PWM servo) and an
  eye-in-hand camera (ADR-0008). The five-axis URDF is a demo and the six-axis fixture is
  test-only — do not present either as the real hardware, and do not invent the missing axis,
  limits or calibration. Do not configure a real `ros2_control` arm hardware plugin before the
  Arm Unlock Gates in `docs/hardware_acceptance.md` pass.
- `src/vendor/**` is third-party. Read-only; do not reformat or "fix" it.
- **CAN protocol ownership.** The wire format's source of truth is the leaf-node repo
  (`vehical_rak_chassis/docs/PROTOCOL.md`, CAN v3), not this one. The bus is classic CAN 2.0A
  @ 1 Mbps — never pass `fd on` (the F407 bxCAN would error-frame an FD frame). But note CAN is
  dead in the current topology; `src/rak_can_driver` / `src/rak_serial_link` are reserved, not live.

## Gotchas

- **CMA memory contention.** The 1024×600 touchscreen and the camera compete for the same
  pool. A logged-in GNOME session drives `CmaFree` to ~0 kB of the 256 MiB CMA area, and the
  stereo node asks that pool for 4 `V4L2_MEMORY_MMAP` buffers — so capture fails in a way
  that looks like a camera bug but is a memory decision. `scripts/rak-mode.sh headless`
  stops and disables gdm. GUI launchers (RViz/MoveIt) require `desktop`; the migrated
  sensor/navigation scripts do not. Autologin is off, so returning to the desktop means
  typing the password at the touchscreen once. (VNC via `x11vnc` attaches to whatever display
  GDM runs; see `README.md`.)
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
  `decider:=tensorrt` until TensorRT assets are integrated.
- The legacy PyTorch/Laya parity test measured the executed policy over 90 states. It does
  not qualify TensorRT; a new engine needs separate logits/probability parity and
  `resolve()` behavior tests before enabling it.
- **System 2 latency is 12–30 s per call** (measured), so S2 is fire-and-forget in a thread
  and replies are applied on a later tick. `_drain()` currently only *records* replies; it
  does not execute them. The S1 callback also precedes safety resolution, so this prototype
  is **not** an independent real-time safety controller.
- C++ is C++17 with `-Wall -Wextra -Wpedantic` in the GPU/stereo packages; `rak_stereo_driver`
  runs `ament_lint_auto` under `BUILD_TESTING`, `rak_gpu` has no functional tests.
- `brain/data/`, `build/`, `install/`, `log/` are gitignored. Stray
  `frames_*.gv` / `frames_*.pdf` at the root are debugging artifacts, not deliverables.
- `scripts/serial_100hz.py` is the PTY-only UART v1 test. `scripts/serial_link_test.py` is a
  **refusal-only** compatibility stub for the retired CRC-8 physical-port protocol — keep it
  refusing (exit 2); do not restore it to a sending version.
- `src/rak_moveit_config/test/validate_six_axis_sim.py` asserts the six-axis URDF contains
  **no** `<ros2_control>` tag. Adding one as part of the ADR-0007 migration will break that
  test — update it deliberately, don't just make the assertion pass.
- The lidar `lidar_name` must be `N10_P` (108 B @ 460800); `N10` fails CRC. Use the udev alias
  `/dev/wheeltec_lidar`, never `/dev/ttyUSB0` (the base adapter may also claim ttyUSB).
- **USB-serial `latency_timer` defaults to 16 ms** and adds jitter to a 100 Hz loop; set it to
  1 ms via udev (`docs/CONSTRUCTION.md` §6.1). This is likely why the MCU needs a 200 ms lease —
  fix the timer and re-test before agreeing to widen the lease.
