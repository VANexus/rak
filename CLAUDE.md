# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Current Engineering Layout (2026-10-04)

The unified package migration is documented in `README.md`, `AGENTS.md`,
`docs/architecture.md` and `docs/migration.md`. Those files supersede the old
package names, commands and arm assumptions in the historical notes below.
The notes are retained to preserve prior hardware investigations and decisions.

- `core` split into `rak_interfaces` (messages) and `rak_gpu` (GPU utilities).
- `eye`, `astra_bridge`, `motor_stub` became `rak_stereo_driver`,
  `rak_astra_driver`, `rak_motion_stub`.
- `boot` split into `rak_sensor_bringup`, `rak_navigation`, `rak_description`,
  `rak_moveit_config`, and `rak_bringup`.
- ROS `brain` became `rak_agent`; its source remains `brain/`, import remains
  `rak_ai`, and its executable is now `agent_node`.
- Use the clean `build/architecture` and `install/architecture` spaces, not
  old install artifacts. Current commands and verification are in README.
- The target arm has six positioning joints plus a separate gripper. The old
  five-axis model remains a demo, not a description of that hardware.
- S2 `_drain()` currently records replies rather than executing them. Safety
  evaluation also still waits for the S1 callback; the prototype is not an
  independent real-time safety controller.
- Topics, model policy, frozen episode schema, JetPack paths and vendor sources
  are unchanged by the migration.

## Historical Implementation Notes

## What this is

`rak` is an AI-native stereo vision + decision system for Jetson Orin Nano (JetPack 7.2),
built as a ROS 2 (Jazzy) colcon workspace. Perception is neural (depth / detection /
segmentation), not traditional CV, and image data stays GPU-resident from capture through
inference. On top of perception sits `brain`, a Python decision layer with a fast local
model (System 1) and a slow LLM path (System 2).

`README.md` is the primary design doc; `AGENTS.md` is a parallel agent-instruction file
with the same layout/commands/gotchas. Both are worth reading before changing behavior —
they record decisions that are not obvious from the code.

## System context (rak robot)

This repo is the **brain** of a three-repo robot. Perception and decision live here;
real-time control lives in two STM32F407 leaf nodes commanded over classic CAN 2.0A @ 1 Mbps.

```
[USB SBS Camera] → eye (V4L2 + nvJPEG GPU decode)
        ↓  left/right image_raw
   (depth / mind / world — PLANNED, not implemented)
        ↓  TrackedObjects
   brain (System 1 Laya + System 2 RAI)
        ↓  Decision
   (CAN bridge — PLANNED, not implemented) → CAN 2.0A 1 Mbps
                        ↓
        ┌───────────────┴───────────────┐
        ↓                               ↓
  vehical_rak_arm                   vehical_rak_chassis
  STM32F407VET6 · M4F               STM32F407VET6 · M4F
  4 joints 25 kHz + base stepper    4 mecanum wheels 25 kHz
  VANexus/vehical_rak_arm           VANexus/vehical_rak_chassis
```

> The two leaf nodes are **the same board**: WHEELTEC C30D (STM32F407VET6, onboard
> 4-channel AT8236 driver, CAN transceiver on CAN1 = PD0/PD1). Only the role differs.
> The bus is **classic CAN 2.0A @ 1 Mbps — no CAN FD**: the Orin's `mttcan` can do FD,
> the F407's bxCAN cannot, so an FD frame would be error-framed by both leaf nodes.
> Never pass `fd on` when bringing up `can0`.

- **This repo (rak):** perception + decision on the Orin. **There is no CAN bridge here
  yet** — `nerve` was never implemented and `CanFrame.msg` no longer exists. The only
  motion path is `motor_stub/cmd_vel_stub.py`, which forwards Nav2's unicycle `Twist`
  from `/cmd_vel` to `/cmd_vel_raw` verbatim (no `Vy`, no enable bit, no CRC/counter).
- **vehical_rak_chassis:** mecanum base. Takes `(Vx, Vy, ωz)` at 100 Hz, does its own wheel
  inverse kinematics, closes four wheels at **25 kHz** (fixed-point, TIM ISR). Reports
  measured body velocity at 100 Hz.
- **vehical_rak_arm:** 4-DOF arm. Takes joint targets from MoveIt — **IK lives here, not on
  the MCU** — closes 4 geared motors at **25 kHz** plus a base stepper on a 1 kHz position
  loop. Never powers into motion by itself: homing is explicit (`HOME_REQUEST`, low-speed
  stall detection), and a lost bus makes it *hold position* rather than release, because
  the arm falls when unpowered.

The CAN wire format is **not defined in this repo**: the source of truth is
`docs/PROTOCOL.md` (CAN v3) + machine-readable `docs/protocol-signals.yaml`, shipped in
each leaf-node repo, with `tools/can_check.py` as the codec gate. Frame IDs carry priority
— safety `0x010`, EMCY `0x020/0x021`, and the 1 kHz interlock `0x030/0x031` all sit below
command frames `0x100/0x101/0x102`, so an e-stop always wins arbitration. Bus timing:
100 Hz Jetson↔leaf, 1 kHz between the two leaf nodes.

## Layout

`colcon build` runs from the repo root and picks up **both** `src/` and `brain/`.

| Package | Language | Status |
|---------|----------|--------|
| `src/core` | C++ | Foundation. Custom messages + CUDA/TensorRT/VPI shared library `core_lib` |
| `src/eye` | C++ | Stereo capture node. Only fully implemented perception node |
| `src/astra_bridge` | C++ | Orbbec Astra S color/depth bridge (OrbbecSDK 1.10.18). No launch file |
| `src/motor_stub` | Python | `/cmd_vel` → `/cmd_vel_raw` stub. No CAN |
| `src/boot` | ament_cmake | Launch files + config + URDF/MoveIt. No compiled code |
| `src/vendor/lslidar/*` | C++ | Vendored Leishen lidar driver + msgs. **Read-only** |
| `src/vendor/rf2o` | C++ | Vendored `rf2o_laser_odometry` (laser-scan odometry). **Read-only** |
| `brain/` | Python | System 1 + System 2 decision layer. Also an `ament_python` package |

`depth`, `mind`, `world`, `forge`, and `nerve` are documented in `README.md` but **do not
exist in the repo**. Treat the README pipeline diagram as a roadmap, not the current tree.

## Build

NVIDIA libraries are hardcoded to JetPack paths, so the C++ packages build only on the
Jetson (aarch64) — VPI at `/opt/nvidia/vpi4`, pkg-config `nvjpeg-13.2`, CUDA headers at
`/usr/local/cuda-13.2/targets/sbsa-linux/include`, OrbbecSDK at `/opt/OrbbecSDK_v1.10.18`.

```bash
source /opt/ros/jazzy/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release   # from repo root; builds brain too
source install/setup.bash
```

- `core` must build first — `eye` links `core::core_lib`, `brain` imports `core.msg`
  (`TrackedObjects`/`Action`/`Decision`). colcon already knows this from `package.xml`.
- `core` needs `vision_msgs` (apt: `ros-jazzy-vision-msgs`), used by `TrackedObject.msg`.
- The only CI-less checks are `ament_lint_auto` blocks (`eye` under `BUILD_TESTING`);
  `core` has no tests. There is no linter/formatter/pre-commit config in this repo.

## Commands

```bash
# Perception
ros2 run eye eye_capture
ros2 run astra_bridge astra_bridge_node          # Orbbec color + depth

# Navigation stack (the de-facto system bringup)
ros2 launch boot nav2.launch.py                  # slam.launch.py + motor_stub + nav2
ros2 launch boot slam.launch.py                  # lidar + rf2o + slam_toolbox → /map
ros2 launch boot lidar.launch.py                 # lslidar driver + scan filter

# Arm (MoveIt2)
ros2 launch boot demo.launch.py
ros2 launch boot view_urdf.launch.py             # headless URDF check

# Decision layer
ros2 launch boot brain.launch.py decider:=laya sim:=true
ros2 run brain brain_node --ros-args -p sim:=true -p decider:=stub
# -p s1_weights:=fp32 is the escape hatch if a decision ever looks wrong; see Gotchas.

# Board memory/DMA mode (needs sudo; see Gotchas)
sudo scripts/rak-mode.sh status | headless | desktop

# Python decision layer, no ROS (run from brain/)
python3 -m venv .venv && .venv/bin/pip install -e ".[laya]"
env -u PYTHONPATH .venv/bin/python -m rak_ai.loop --scenario approach --decider stub --ticks 50
env -u PYTHONPATH .venv/bin/python tests/test_ai.py                 # plain asserts, no pytest
env -u PYTHONPATH .venv/bin/python tests/test_laya_fp16_parity.py   # needs model + GPU, ~10 min
```

`env -u PYTHONPATH` is **mandatory** for anything run inside the venv: `~/.bashrc` sources
ROS unconditionally and injects `/opt/ros/jazzy/.../site-packages` into the venv, defeating
its isolation. It does **not** apply to `ros2 run`/`ros2 launch`, which need that same
PYTHONPATH to find `rclpy` and the install space.

Operator launchers live in `src/boot/scripts/run_*.sh` (set `DISPLAY=:1`, `ROS_DOMAIN_ID=45`,
Cyclone DDS, then `setsid nohup ros2 launch ...`); see `src/boot/scripts/README.md`.

Treat `colcon build` + `brain/tests/test_ai.py` as the two checks to pass before claiming done.

## Architecture

**`core`** (`src/core/include/core/*.hpp`) is the shared foundation and the idiomatic
patterns for new C++ code:
- `CUDA_CHECK` / `CU_CHECK` / `VPI_CHECK` macros throw `std::runtime_error` with file:line
  — never leak raw status codes.
- RAII wrappers: `CudaStream` + `StreamPool` + `GpuTimer` (`cuda.hpp`), `DeviceBuffer` /
  `PinnedBuffer` (templated, move-only), `TrtEngine` (`trt.hpp`: load `.plan`,
  `build_from_onnx`, `infer`), `VpiStream` / `VpiImage` / `VpiRectifier` (`vpi_res.hpp`).
- Messages (`src/core/msg/`): `TrackedObject` / `TrackedObjects` (3D tracks),
  `Action` (`kind`: stop|slow|continue|turn_left|turn_right|inspect|ask_operator),
  `Decision` (Action + source/p/margin/latency_ms/tier).

**`eye`** (`src/eye/src/capture_node.cpp`) is the reference implementation and the subtle part:
- V4L2 MJPEG capture of a side-by-side frame (SP2812, 1280×720 @ 30 Hz),
  `NVJPEG_BACKEND_GPU_HYBRID` decode straight into a GPU BGR buffer.
- **Two threads, two CUDA streams, ping-pong double buffering.** `nvjpegDecode` blocks the
  CPU ~10 ms, so it runs on a dedicated decode thread; the main thread does grab (~33 ms) +
  copy + publish. Because grab > decode, the decode of frame N finishes while frame N+1 is
  being grabbed.
- Main thread copies the decoded SBS frame to host with two `cudaMemcpy2DAsync` calls (one
  per eye half) into **pinned** buffers, then publishes `bgr8` on `eye/left/image_raw` and
  `eye/right/image_raw` (and matching `camera_info` topics — publishers exist, nothing
  publishes to them yet). `device`, `sbs_width/height`, `fps`, `frame_id`, and the four
  topic names are all node parameters.
- FPS pitfall: some USB cameras support `V4L2_CID_EXPOSURE_DYNAMIC_FRAMERATE` (0x009a0903),
  which silently drops output to ~20 fps in low light. The node disables it at init — keep that.
- Registered as composable component `rak::eye::CaptureNode`, executable `eye_capture`.

**`boot`** is assets only: `launch/`, `config/` (nav2, slam_toolbox, lslidar, MoveIt YAMLs,
Cyclone DDS), `urdf/` (5-DOF arm xacro + SRDF), `meshes/`, `scripts/`. `nav2.launch.py` is
the de-facto bringup:

```
lidar → laser_filters → rf2o → slam_toolbox (/map, map→odom) → nav2 → /cmd_vel
      → cmd_vel_stub → /cmd_vel_raw (MCU)
```

`astra_bridge`, `brain`, and the MoveIt arm are **not** tied into `nav2.launch.py`; each has
its own launch file. `README.md:49` references `boot stereo.launch.py`, which does not exist.

**`brain`** (`brain/rak_ai/`) is the decision layer, shipped as both a pip package (`rak-brain`)
and an `ament_python` package named `brain` (its `COLCON_IGNORE` is gone on purpose — do not
re-add it). Entry points share one policy, `system1.resolve()`:
- `system1.py` — fast path. `guard()` is the deterministic geometric veto (real authority);
  `gate()` only picks the escalation tier. Deciders: `StubDecider`, `LayaDecider`.
- `system2.py` — slow path behind a `Consultant` protocol: `StubConsultant`, `RaiConsultant`
  (drives RAI's LLM plumbing directly, LangGraph underneath).
- `node.py` — ROS edge. Subscribes `/perception/tracked_objects` (BEST_EFFORT, keep-last-1:
  newest frame only, a backlog is worse than a gap), publishes `/brain/decision` (RELIABLE,
  depth 10: the consumer is an actuator). Params: `sim`, `decider`, `scenario`, `s1_weights`.
- `loop.py` — the no-ROS tick loop (`sense → guard → decide → escalate → act → log`), writes
  JSONL episodes to `brain/data/episodes/`.
- `body.py` — hardware seam `Body` protocol + `MockBody` with `approach`/`cross`/`clear`
  scenarios. CAN actuators are a future real implementation.
- `types.py` — the frozen S1/S2 contract (`Entity`, `SceneState`, `Action`, `Decision`, `Tier`).

## Constraints that are easy to break

- **System 1 (Laya) is a direct function call.** Never route it through an orchestration
  framework or a cross-process boundary. The RAI harness belongs only on the System 2 slow path.
- **`guard()` is the only authority** (deterministic geometric veto, overrides everything).
  `gate()` only decides escalation tier. System 2 may narrow authority, never widen it.
  Laya's confidence is out-of-domain and untrusted — a malformed model answer is a normal
  path (→ LOW → escalate), not an exception.
- **`brain/rak_ai/types.py` is frozen once episodes exist.** `state_text` is stored verbatim
  in `brain/data/episodes/*.jsonl`; regenerating it silently corrupts the finetune corpus.
  There is deliberately no training pipeline here.
- `--decider stub` is the default and must stay runnable without torch or a model — torch on
  aarch64/JetPack is a real install risk. `laya`/`rai` are optional extras in
  `brain/pyproject.toml`; do not make them hard dependencies.
- LLM key comes from `OPENAI_API_KEY`; `brain/config.toml` points `[openai]` at SiliconFlow
  and must never contain the key.
- `src/vendor/**` is third-party. Treat as read-only; do not reformat or "fix" it.

## Gotchas

- **CMA memory contention.** The 1024×600 touchscreen and the camera compete for the same
  pool. A logged-in GNOME session drives `CmaFree` to **0 kB** of the 256 MiB CMA area, and
  `eye` asks that pool for 4 `V4L2_MEMORY_MMAP` buffers — so capture fails in a way that
  looks like a camera bug but is a memory decision. `scripts/rak-mode.sh headless` stops and
  disables gdm (~1.8 → ~2.3 GiB available, 75–160 MiB CMA back); it also kills every
  `DISPLAY=:1` launch script until `desktop`, and because autologin is off here, coming back
  means typing the password at the touchscreen once.
- **nvpmodel sits in the 10W mode** on this board (6 cores, capped EMC bandwidth), so the
  ~158 ms/predict figure and eye's 30 fps are both measured under that cap. Raising it
  (`sudo nvpmodel -m 0 && sudo jetson_clocks`) is root and changes the thermal envelope — do
  not assume a benchmark is comparable across the two.
- **Laya costs ~1.4 GB of this box's 3.4 GB unified memory** to sit on the GPU: 804 MiB of
  resident weights (421M params, bf16) plus ~600 MiB of CUDA context and cuDNN/cuBLAS
  workspace. The checkpoint is F16 on disk, but `laya.load()` builds its module in torch's
  *default* dtype (fp32) and `load_state_dict` copies up, so weights used to sit resident at
  1606 MiB; on Tegra CUDA's "free" is just pages nobody holds, so the fp32 build peaked on
  the CPU side and starved the GPU side — `laya.load(model)` OOMed and fell back to CPU
  before anything could be cast. `LayaDecider` therefore loads on CPU with the default dtype
  already set to the checkpoint's `amp_dtype` (bf16) and places afterwards, so fp32 never
  exists. `s1_weights=auto|fp16|bf16|fp32` selects it; `auto` follows the checkpoint.
- Half weights only run where autocast is on, and laya only turns it on for CUDA (CPU bf16
  is the documented `LAYA_CPU_AMP=bf16`). Without it the forward dies in `F.linear` with
  "mat1 and mat2 must have the same dtype". So `LayaDecider` degrades to fp32 when there is
  no CUDA rather than hand back a model that cannot run.
- If it does not fit, `laya` catches the CUDA OOM itself and drops to CPU at ~1.6 s/predict
  — slower than the sensor rate, with no exception raised. `LayaDecider.device/.weights/
  .place_error` and the node's `s1_device=`/`s1_weights=` startup fields say which one you
  got; `brain_node` warns on CPU. Keep baseline RAM low before running `decider:=laya`.
- The fp16/bf16 cast is verified, not assumed: `tests/test_laya_fp16_parity.py` compares fp32
  against the cast over 90 states and asserts on what `resolve()` *executes*, not the model's
  label — at LOW tier the label is discarded and the geometric fallback runs. Measured: 0
  executed-action, 0 source, 0 tier flips, max |dp| 0.004, 5 label flips all at LOW.
- **System 2 latency is 12–30 s per call** (measured), so S2 is fire-and-forget in a thread
  and replies are applied on a later tick. Stale replies being discarded (~half in a real
  run) and a bounded run taking an extra S2 latency at flush are expected, not bugs.
- C++ is C++17 with `-Wall -Wextra -Wpedantic` (both `core` and `eye`).
- `brain/data/`, `brain/.venv/`, `build/`, `install/`, `log/` are gitignored. Stray
  `frames_*.gv` / `frames_*.pdf` at the root are debugging artifacts, not deliverables.
- `brain/pyproject.toml` says `dynamic = ["requires-python"]` and `setup.py` supplies the
  value **on purpose**: setuptools' pyproject loader turns `requires-python` into a
  `SpecifierSet` object, and colcon reads the setup.py distribution through
  `ast.literal_eval`, so moving it back under `[project]` makes `colcon build` fail with
  `SyntaxError: invalid syntax (<unknown>, line 1)`.
- `brain/scripts/brain_node` is a stdlib-only bootstrap, not a `console_scripts` entry point:
  colcon builds scripts with its own system `python3`, which cannot `import pydantic`. It
  re-execs `$RAK_BRAIN_PYTHON` or `<repo>/brain/.venv/bin/python`, so the venv never needs
  to be activated at build time.
