# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Orientation

Layered embodied-robot workspace for Jetson Orin Nano / ROS 2 Jazzy: perception +
decision on the Orin, real-time control on two STM32F407 leaf nodes. Read these before
changing behavior — they record decisions the code does not show:

- `README.md` — primary design doc (Chinese): layout, build, launch entries, hardware.
- `docs/architecture.md` — package boundaries, target runtime architecture, acceptance
  stages, and an explicit list of gaps that are **not** closed.
- `docs/migration.md` — the 2026-10-04 unified rename (old name → new name table).
- `AGENTS.md` — parallel agent-instruction file; same content as this one.

The 2026-10-04 migration renamed every owned package to a `rak_` prefix. Any doc or comment
using `core` / `eye` / `astra_bridge` / `motor_stub` / `boot` / `brain_node` is historical.
Topics, node names, QoS, message fields, the frozen AI contract and vendor sources did
**not** change — only package identity did.

## Layout

`colcon build` runs from the repo root and picks up both `src/` and `brain/`.

| Package | Contents |
|---|---|
| `src/rak_interfaces` | Messages only (`TrackedObject(s)`, `Action`, `Decision`). No GPU/model deps |
| `src/rak_gpu` | CUDA / TensorRT / VPI RAII utilities. JetPack-only. Export target `rak_gpu::gpu` |
| `src/rak_stereo_driver` | V4L2 + nvJPEG stereo capture. Component `rak::stereo_driver::CaptureNode`, exec `stereo_node` |
| `src/rak_astra_driver` | Orbbec Astra depth/color bridge (OrbbecSDK 1.10.18). Exec `astra_node` |
| `src/rak_sensor_bringup` | Device launch + config: `lidar`, `astra`, `stereo` |
| `src/rak_description` | URDF/xacro, meshes, RViz. **Still the legacy five-axis demo model** |
| `src/rak_navigation` | SLAM, odometry sources, Nav2: `slam.launch.py`, `nav2.launch.py` |
| `src/rak_moveit_config` | SRDF, IK, planning config. No real trajectory execution path |
| `src/rak_motion_stub` | Test-only `/cmd_vel` → `/cmd_vel_raw` relay (`cmd_vel_stub`). Not a CAN driver |
| `src/rak_bringup` | Integration entry (`agent.launch.py`), DDS config, operator scripts |
| `brain/` | Decision layer. ROS package `rak_agent`, Python module `rak_ai`, pip project `rak-brain` |
| `src/vendor/*` | Leishen lidar + `rf2o`. Third-party, **read-only** |

`brain/` stays at the repo root on purpose: its `.venv`, episode data and pip workflow do
not move with the ROS package name. Its `COLCON_IGNORE` is gone deliberately — do not
re-add it. `depth` / `mind` / `world` / `forge` / `nerve` are roadmap names; they do not
exist here.

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
colcon test --build-base build/architecture --packages-select rak_description rak_moveit_config
colcon test-result --test-result-base build/architecture --verbose
env -u PYTHONPATH brain/.venv/bin/python brain/tests/test_ai.py
```

Use `build/architecture` + `install/architecture`, never the old `build/` / `install/`
artifacts. Decision-layer-only work skips the GPU packages: `--packages-up-to rak_agent`.
Treat `colcon build` + `brain/tests/test_ai.py` as the two checks to pass before claiming done.
There is no CI, linter, formatter or pre-commit config in this repo.

`env -u PYTHONPATH` is mandatory for anything run inside `brain/.venv`: `~/.bashrc` sources
ROS unconditionally and injects `/opt/ros/jazzy/.../site-packages` into the venv, defeating
its isolation. It does **not** apply to `ros2 run`/`ros2 launch`, which need that PYTHONPATH.

## Entry points

```bash
ros2 launch rak_sensor_bringup lidar.launch.py
ros2 launch rak_sensor_bringup astra.launch.py
ros2 launch rak_sensor_bringup stereo.launch.py device:=/dev/video0
ros2 launch rak_navigation slam.launch.py
ros2 launch rak_navigation nav2.launch.py            # includes lidar + SLAM + motion stub
ros2 launch rak_moveit_config demo.launch.py
ros2 launch rak_description view_urdf.launch.py
ros2 launch rak_bringup agent.launch.py decider:=laya sim:=true
ros2 run rak_agent agent_node --ros-args -p sim:=true -p decider:=stub
ros2 run rak_stereo_driver stereo_node
# -p s1_weights:=fp32 is the escape hatch if a decision ever looks wrong; see Gotchas.
sudo scripts/rak-mode.sh status | headless | desktop   # board memory/DMA mode
```

These are independent entries — do not start them all. `nav2.launch.py` already contains
lidar + SLAM; the arm demo's static `world → base_link` cannot be merged with the mobile
base TF. Operator launchers live in `src/rak_bringup/scripts/run_*.sh` and share
`environment.sh` (they resolve the repo from their own path, default DDS Cyclone, domain 45,
logs to `/tmp/*_run.log`); see that directory's `README.md`.

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

**`rak_agent`** (`brain/rak_ai/`) is the decision layer, shipped both as the pip package
`rak-brain` and the `ament_python` package `rak_agent`. All entry points share one policy,
`system1.resolve()`:
- `system1.py` — fast path. `guard()` is the deterministic geometric veto (real authority);
  `gate()` only picks the escalation tier. Deciders: `StubDecider`, `LayaDecider`.
- `system2.py` — slow path behind a `Consultant` protocol: `StubConsultant`,
  `RaiConsultant` (drives RAI's LLM plumbing directly, LangGraph underneath).
- `node.py` — ROS edge. Subscribes `/perception/tracked_objects` (BEST_EFFORT, keep-last-1:
  newest frame only, a backlog is worse than a gap), publishes `/brain/decision` (RELIABLE,
  depth 10: the consumer is an actuator). Params: `sim`, `decider`, `scenario`, `s1_weights`.
- `loop.py` — no-ROS tick loop (`sense → guard → decide → escalate → act → log`), writes
  JSONL episodes to `brain/data/episodes/`. `body.py` — `Body` protocol + `MockBody`
  scenarios. `types.py` — the frozen S1/S2 contract.
- `scripts/agent_node` is a stdlib-only bootstrap, not a `console_scripts` entry point:
  colcon builds scripts with its system `python3`, which cannot `import pydantic`. It
  re-execs `$RAK_BRAIN_PYTHON` or `<repo>/brain/.venv/bin/python`, so the venv never needs
  activating at build time.

**`rak_navigation`** chains the de-facto bringup:
`lidar → laser_filters → rf2o → slam_toolbox (/map, map→odom) → nav2 → /cmd_vel → cmd_vel_stub → /cmd_vel_raw`.
The stub forwards Nav2's unicycle `Twist` verbatim — no `Vy`, no enable bit, no CRC/counter;
mecanum strafing is configured to zero and collision-monitor is set not to intervene.

## Constraints that are easy to break

- **System 1 (Laya) is a direct function call.** Never route it through an orchestration
  framework or a cross-process boundary. The RAI harness belongs only on the System 2 slow path.
- **`guard()` is the only authority** (deterministic geometric veto, overrides everything).
  `gate()` only decides escalation tier. System 2 may narrow authority, never widen it.
  Laya's confidence is out-of-domain and untrusted — a malformed model answer is a normal
  path (→ LOW → escalate), not an exception.
- **`brain/rak_ai/types.py` is frozen once episodes exist.** `state_text` is stored verbatim
  in `brain/data/episodes/*.jsonl`; regenerating it silently corrupts the finetune corpus.
  There is deliberately no training pipeline here. `SceneState` uses camera x-right/z-forward;
  ROS body convention is x-forward/y-left — transform explicitly in the adapter, never by
  rewriting the frozen semantics.
- `--decider stub` is the default and must stay runnable without torch or a model — torch on
  aarch64/JetPack is a real install risk. `laya`/`rai` are optional extras in
  `brain/pyproject.toml`; do not make them hard dependencies.
- LLM key comes from `OPENAI_API_KEY`; `brain/config.toml` points `[openai]` at SiliconFlow
  and must never contain the key.
- ROS message type identity includes the package name: `rak_interfaces/msg/Decision` does not
  interoperate with the old `core/msg/Decision`. External nodes must be rebuilt; old rosbags
  need explicit type migration.
- The target arm is **six positioning joints plus an independent gripper**. The five-axis
  URDF is a demo — do not present it as the real hardware, and do not invent the missing
  axis or calibration.
- `src/vendor/**` is third-party. Read-only; do not reformat or "fix" it.
- No CAN bridge exists here. The wire format's source of truth is `docs/PROTOCOL.md` (CAN v3)
  in each leaf-node repo, not this one. The bus is **classic CAN 2.0A @ 1 Mbps — never pass
  `fd on`** (the F407 bxCAN would error-frame an FD frame).

## Gotchas

- **CMA memory contention.** The 1024×600 touchscreen and the camera compete for the same
  pool. A logged-in GNOME session drives `CmaFree` to ~0 kB of the 256 MiB CMA area, and the
  stereo node asks that pool for 4 `V4L2_MEMORY_MMAP` buffers — so capture fails in a way
  that looks like a camera bug but is a memory decision. `scripts/rak-mode.sh headless`
  stops and disables gdm (≈1.8 → 2.3 GiB free, 75–160 MiB CMA back). GUI launchers
  (RViz/MoveIt) require `desktop`; the migrated sensor/navigation scripts do not. Autologin
  is off, so returning to the desktop means typing the password at the touchscreen once.
- **nvpmodel sits in the 10W mode** on this board (6 cores, capped EMC bandwidth), so the
  ~158 ms/predict figure and stereo's 30 fps were both measured under that cap. Raising it
  (`sudo nvpmodel -m 0 && sudo jetson_clocks`) is root and changes the thermal envelope — do
  not compare benchmarks across the two.
- **Laya costs ~1.4 GB of this box's 3.4 GB unified memory** to sit on the GPU: 804 MiB of
  bf16 weights (421M params) plus ~600 MiB of CUDA context and cuDNN/cuBLAS workspace. The
  checkpoint is F16 on disk, but `laya.load()` builds in torch's *default* dtype (fp32) and
  `load_state_dict` copies up, so weights used to sit resident at 1606 MiB; on Tegra CUDA's
  "free" is just pages nobody holds, so the fp32 build peaked on the CPU side and starved
  the GPU side at the same moment — `laya.load(model)` OOMed and fell back to CPU before
  anything could be cast. `LayaDecider` therefore loads on CPU with the default dtype
  already set to the checkpoint's `amp_dtype` (bf16) and places afterwards, so fp32 never
  exists. `s1_weights=auto|fp16|bf16|fp32` selects it; `auto` follows the checkpoint.
- Half weights only run where autocast is on, and laya only turns it on for CUDA (CPU bf16
  is the documented `LAYA_CPU_AMP=bf16`). Without it the forward dies in `F.linear` with
  "mat1 and mat2 must have the same dtype". So `LayaDecider` degrades to fp32 when there is
  no CUDA rather than hand back a model that cannot run.
- If it does not fit, `laya` catches the CUDA OOM itself and drops to CPU at ~1.6 s/predict
  — slower than the sensor rate, with no exception raised. `LayaDecider.device/.weights/
  .place_error` and the node's `s1_device=`/`s1_weights=` startup fields say which one you
  got; `agent_node` warns on CPU. Keep baseline RAM low before running `decider:=laya`.
- The fp16/bf16 cast is verified, not assumed: `brain/tests/test_laya_fp16_parity.py`
  compares fp32 against the cast over 90 states and asserts on what `resolve()` *executes*,
  not the model's label — at LOW tier the label is discarded and the geometric fallback
  runs. Measured: 0 executed-action / source / tier flips, max |dp| 0.004, 5 label flips all
  at LOW. Autocast does not cover LayerNorm/embeddings/softmax, which is why it must run.
- **System 2 latency is 12–30 s per call** (measured), so S2 is fire-and-forget in a thread
  and replies are applied on a later tick. `_drain()` currently only *records* replies; it
  does not execute them. The S1 callback also precedes safety resolution, so this prototype
  is **not** an independent real-time safety controller. Stale replies being discarded
  (~half in a real run) and a bounded run taking an extra S2 latency at flush are expected.
- C++ is C++17 with `-Wall -Wextra -Wpedantic` in the GPU/stereo packages; `rak_stereo_driver`
  runs `ament_lint_auto` under `BUILD_TESTING`, `rak_gpu` has no functional tests.
- `brain/data/`, `brain/.venv/`, `build/`, `install/`, `log/` are gitignored. Stray
  `frames_*.gv` / `frames_*.pdf` at the root are debugging artifacts, not deliverables.
- `brain/pyproject.toml` says `dynamic = ["requires-python"]` and `setup.py` supplies the
  value **on purpose**: setuptools' pyproject loader turns `requires-python` into a
  `SpecifierSet` object, and colcon reads the setup.py distribution through
  `ast.literal_eval`, so moving it back under `[project]` makes `colcon build` fail with
  `SyntaxError: invalid syntax (<unknown>, line 1)`.
