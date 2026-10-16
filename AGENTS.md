# AGENTS.md

Layered embodied robot system for Jetson Orin Nano (JetPack 7.2). C++/ROS 2 (Jazzy) + a
plain-Python decision layer. `README.md` is the primary design doc — read it before
changing behavior; it records decisions that are not obvious from the code. `CLAUDE.md`
is the parallel instruction file (Claude Code) and goes deeper on architecture; keep
repository-wide guidance in sync between the two.

## Topology changed on 2026-10-05, again on 2026-10-09 — read this first

The arm board burned out. There is now **one** F407 driving 4 wheels + 7 servos.
**2026-10-09 (ADR-0012): CAN v4 is back as the only primary link** (SN65HVD230 transceiver,
protocol v4 in the leaf repo); USB v2 is the frozen backup, selectable via `transport:=usb|can`
behind the `Session` abstraction in `src/rak_chassis`. Upper layers never see which transport
is active. **2026-10-09 (ADR-0013): the workspace was regrouped by subsystem** — 23 → 18
buildable packages, `brain/` moved to `src/rak_agent`, CAN v3 stack retired (local `attic/`
archive deleted 2026-10-10 — the code lives only in git history).

- `docs/adr/*` are immutable (append/supersede only), indexed by `docs/adr/README.md`.
  **ADR-0007 supersedes ADR-0001**; the valid set is 0002 + 0003 (v4 layer) + 0004–0015.
  **ADR-0011 (WBC/HQP motion arbitration) was accepted 2026-10-08**: `src/rak_wbc`
  (ament_python skeleton + desktop numeric tests) exists; arm interfaces and measured
  masses/geometry remain hard gates — read its closed items before wiring hardware.
- ADR-0010's target — Harness as a standard ROS 2 package `src/rak_harness` + node
  `harness_node`, the Jev↔executor arbitration seam, a separate `anthropic` cloud client and a
  gated "呼吸感" expression interface — is **accepted but design-only**: there is no
  `src/rak_harness`, and the expression layer is a no-op stub behind closed hardware gates.
- ADR-0007's target — whole robot on `ros2_control`, host-side mecanum IK — is **partially
  implemented as of 2026-10-14**: `src/rak_chassis` (ex `rak_hardware`, renamed by ADR-0013)
  exports 4 wheel velocity interfaces (F407 chassis smoke test, firmware R37) **plus 7 arm
  position command interfaces** (6 joints + gripper, command domain = servo command angle,
  contract `docs/arm_moveit_contract.md` §1), with `arm_controller`
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
  the local `attic/` archive was **deleted on 2026-10-10** — the code lives only in git
  history (retrievable at commit `4f34795` and earlier). The live CAN path is the
  `CanSession` backend inside `src/rak_chassis`.
- Docs disagree by date, not correctness. The 2026-10-04 set (`README.md`,
  `docs/architecture.md`, `docs/project_status.md`) describes the code as it stands;
  `TODO.md`, `docs/CONSTRUCTION.md` and `docs/adr/0007` describe the accepted target.
  Do not read a 10-04 status line as current, and do not implement the target as if the
  code already supported it.

## Layout

The 2026-10-09 subsystem regroup (ADR-0013) supersedes the 2026-10-04 package layout.
Read `docs/architecture.md` and `docs/migration.md` (dated mapping tables) before moving
package boundaries. 18 buildable packages, every one with a real responsibility:

- `src/rak_interfaces`: messages/actions/services only, no GPU/model dependencies.
- `src/rak_gpu`: JetPack library, exported target `rak_gpu::gpu`, namespace `rak::gpu`.
- `src/rak_stereo_driver`: plugin `rak::stereo_driver::CaptureNode`, executable `stereo_node`.
- `src/rak_astra_driver`: tested depth camera adapter, executable `astra_node`.
- `src/rak_pointcloud`: GPU point cloud node (`gpu_pointcloud_node`).
- `src/rak_description`: geometry and visualization — legacy five-axis demo plus the
  isolated six-axis fixture `urdf/six_axis_sim.urdf.xacro`.
- `src/rak_moveit_config`: SRDF, IK and planning configuration; six-axis fixture is simulation-only.
- `src/rak_chassis`: (ex `rak_hardware`) F407 transport + ros2_control plugin — `Session`
  abstraction with `SerialSession` (USB v2, frozen backup) and `CanSession` (SocketCAN CAN v4,
  primary), `transport:=usb|can` factory (ADR-0012); 4 wheel velocity + 7 arm position
  interfaces (arm since 2026-10-14).
- `src/rak_base_driver`: USB v2 protocol codec + UART v1 host driver node; the codec is
  shared by commissioning tools. Node hardware disabled by default; its node fate stays
  open per ADR-0007 待闭合项 #7.
- `src/rak_commissioning`: (ex `rak_drive_tui` + `rak_arm_tui` + `chassis_key_test`)
  commissioning entries: `chassis_tui`, `arm_tui`, `chassis_key_test`,
  `launch/chassis_test.launch.py`. Both TUIs are pure ROS 2 tools (actions/topics only, never
  touch CAN directly — `check_workspace.py` forbids this package depending on `rak_chassis`).
  Not production motion interfaces.
- `src/rak_arm_simulation`: software-only six-axis trajectory/gripper action fixture.
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
  live in `docs/microphone_array.md`.
- `src/rak_wbc`: velocity-level whole-body arbitration (ADR-0011): ament_python standalone
  node `wbc_node` — single regular motion writer for base + arm; skeleton + desktop numeric
  tests exist, hardware wiring gated by the ADR's closed items. Fixtures/smoke:
  `fake_arm_stream` (command-echo arm fixture) and `scripts/wbc_shadow_smoke.sh`
  (shadow contract smoke; remaps the base output — no actuator is commanded).
- `src/rak_motion_stub`: test relay/estimate, not a hardware driver.
- `src/rak_bringup`: the single composition layer — all launch files (lidar/astra/stereo/
  gemini335l, slam, nav2, task_navigation, agent, base_uart, chassis_bringup), params,
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
- `~/vehical_rak_chassis` — STM32 F407 firmware project, **not a ROS ws**. Frozen source of
  truth for the CAN v4 protocol (referenced by `scripts/can_link_smoke.py` and
  `docs/adr/0016`); do not move or absorb it.

Two gitignored local directories live *inside* the repo (moved out of `~` on 2026-10-14;
`.gitignore` carries `/tools/` and `/debug/`, root-anchored so they cannot swallow a nested
`tools/` in `src/`):

- `tools/` — local tooling and third-party copies: `tools/camera/` (OrbbecSDK 1.10.18),
  `tools/graspnet/` (capture/verification artifacts), `tools/remote_check.sh`,
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
colcon test --build-base build/architecture --install-base install/architecture --packages-select rak_description rak_moveit_config
colcon test-result --test-result-base build/architecture --verbose
ROS_DOMAIN_ID=77 python3 -m pytest src/rak_task_executor/test src/rak_arm_simulation/test
ROS_DOMAIN_ID=77 PYTHONPATH=src/rak_base_driver:$PYTHONPATH python3 -m pytest src/rak_base_driver/test
ros2 run rak_stereo_driver stereo_node
ros2 launch rak_bringup lidar.launch.py
ros2 launch rak_bringup speech.launch.py  # array: capture -> ASR, board wake -> /speech/wake, /speech/say -> speaker

# Memory/DMA mode of the board itself: status | headless | desktop (needs sudo; see Gotchas)
sudo scripts/rak-mode.sh headless

# The brain as a standard ROS node (system Python; TRT backend is not implemented yet)
ros2 launch rak_bringup agent.launch.py decider:=stub sim:=true
ros2 run rak_agent agent_node --ros-args -p sim:=true -p decider:=stub

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
  `--packages-skip rak_astra_driver lslidar_driver rak_stereo_driver` — the other 18
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
- **`PYTHONPATH=src/rak_base_driver` replaces ROS's own PYTHONPATH** →
  `ModuleNotFoundError: No module named 'rclpy'`. Append `:$PYTHONPATH`; putting
  `src/...` first also makes pytest test your edits without a rebuild.
- **`test_protocol.py` duplicates are a pytest hazard**: `rak_base_driver` has one, and the
  retired CAN v3 stack (git history only since 2026-10-10) had another. Two in one pytest
  run → `ImportPathMismatchError`, even with `--import-mode=importlib`. Run one package per
  invocation, or `colcon test --packages-select`, which isolates them.
- **ROS pytest needs the install overlay sourced** — `rak_interfaces` exists only there.
  `ROS_DOMAIN_ID=77` keeps it off runtime domain 47, which `~/.bashrc` pins for the operator.
  `colcon test` on `rak_moveit_config` fails `legacy_srdf` until `python3-defusedxml`
  (a declared `test_depend`) is installed.
- `rak_agent` requires the Ubuntu `python3-pydantic` system package. The implementation
  supports Ubuntu's Pydantic 1 API; `package.xml` declares it for rosdep. The TensorRT
  binding is supplied by JetPack, while the model engine and tokenizer are not yet integrated.

`rak_agent` runs under the sourced ROS system Python. ROS integration tests must keep the
install overlay on `PYTHONPATH` and use an isolated `ROS_DOMAIN_ID`; plain decision tests
can use `PYTHONPATH=src/rak_agent`.

Text navigation smoke test: source `install/architecture/local_setup.bash`, then run
`ros2 launch rak_task_executor task_simulation.launch.py` and in another terminal run
`ros2 run rak_task_executor task_cli`. This fixture uses fake Nav2 and explicitly estimated
base telemetry; it does not use sensors or represent physical feedback. For sensor-backed
Nav2 composition use `ros2 launch rak_bringup task_navigation.launch.py motion_backend:=sim`.
Current progress and acceptance gaps are tracked in `docs/project_status.md`.

Build `rak_interfaces` before `rak_agent`, and `rak_gpu` before `rak_stereo_driver`;
colcon knows the order from package manifests. Treat `colcon build` +
`tests/test_ai.py` as the two checks to pass before claiming done — subject to the
caveats above.

## Constraints that are easy to break

- **Layering rules (ADR-0013) — the workspace contract.** Modules talk only through frozen
  message contracts (`rak_interfaces` + standard ROS msgs); changing a message is a breaking
  change requiring synchronized consumers. Upper layers never branch on the transport backend
  (ADR-0012 `transport:=usb|can`) or on a sensor SDK — same-need sensors publish the same
  message set and switch via launch args. rclpy nodes are always `setup.py` entry points,
  never bare scripts. `rak_agent` (the decision layer) must not gain dependents beyond
  `rak_bringup`; `check_workspace.py` enforces all of this — fix the structure, not the assert.
- **System 1 (Laya) is a direct function call.** Never route it through an orchestration
  framework or a cross-process boundary. The RAI harness belongs only on the System 2
  slow path.
- **`guard()` is the only authority** (deterministic geometric veto, overrides everything).
  `gate()` only decides escalation tier. System 2 may narrow authority, never widen it.
  Laya's confidence is out-of-domain and untrusted — a malformed model answer is a normal
  path (→ LOW → escalate), not an exception.
- **`src/rak_agent/rak_agent/types.py` is frozen once episodes exist.** `state_text` is stored
  verbatim in `data/episodes/*.jsonl` (repo root, ADR-0013); regenerating it silently corrupts
  the finetune corpus. There is deliberately no training pipeline here.
- `--decider stub` is the default and stays runnable without a model. The target System 1
  runtime is in-process TensorRT from JetPack; do not add a model service or generic
  CUDA/PyTorch wheel dependency. TensorRT model loading is not implemented until its
  ONNX/tokenizer contract is validated.
- `rak_task_executor` is the only task/goal lifecycle owner. Pause is not reported until
  Nav2 cancellation is confirmed and fresh repeated stationary telemetry arrives; resume
  submits the semantic map goal again. Its default parser is local; optional RAI planning
  may only choose a configured waypoint alias and never issues velocity/joint commands.
  It consumes the shared `rak_intent` contract — never parse logic private to the executor.
- `BaseUartNode` defaults to no port, no hardware access, zero speed limits and DISARMED.
  The bringup launch currently supplies `/dev/wheel_motor`, but `hardware_enable=false`
  means the node does not open it. All three finite positive speed limits are required
  before ARM; zero is not pass-through. UART v1 is a proposed host protocol; this repository
  has no matching F407 firmware. PTY tests do not prove device compatibility.
  Never change defaults to ARM as a shortcut.
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
  `rak_task_executor`); the executor must not subscribe to `/brain/decision`, and Harness emits
  structured goals only — never velocity or joint commands.

## Gotchas

- Build files hardcode JetPack paths: VPI at `/opt/nvidia/vpi4`, pkg-config module
  `nvjpeg-13.2`, CUDA headers at `/usr/local/cuda-13.2/targets/sbsa-linux/include`.
  This workspace only builds on the Jetson (aarch64); do not "portable-ize" casually
  without an x86 alternative to verify against.
- C++ is C++17 with `-Wall -Wextra -Wpedantic` in GPU/stereo packages. Stereo runs
  `ament_lint_auto` under `BUILD_TESTING`; GPU has no functional tests.
- System 2 latency is 12–30 s per call (measured), so S2 is fire-and-forget in a thread
  and replies are checked on a later tick. Currently `_drain()` only records replies;
  it does not execute them. The S1 callback also precedes safety resolution, so the
  prototype is not an independent real-time safety path. See architecture.md.
- `data/`, `build/`, `install/`, `log/` are gitignored. Stray
  `frames_*.gv` / `frames_*.pdf` at the root are debugging artifacts, not deliverables.
- **`colcon build` only ever runs from the repo root** (`--base-paths src`). Run it from `~`
  and it drops orphan `~/build` / `~/install` / `~/log` in the home directory *and* drags
  `~/ws_thirdparty` and `~/vehical_rak_chassis` into package discovery — that is exactly how
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
- `scripts/serial_100hz.py` is the PTY-only UART v1 test. `scripts/serial_link_test.py`
  is refusal-only for the retired CRC-8 physical-port utility; keep it that way.
- Simulation task telemetry is a command-motion estimate, not measured odometry. A
  successful software pause/resume test is not hardware acceptance; see
  `docs/hardware_acceptance.md`.
- **`nvpmodel` is already at mode 0 = 10 W, this board's *highest* mode** (the only other
  modes are `7W_AI` and `7W_CPU`), so there is nothing to raise. `nvpmodel.service` runs
  at boot, but `jetson-clocks.service` is enabled and has **not** run (inactive, no
  journal entries): the governor is `schedutil` and CPU clock floats 729–1510 MHz. Do not
  assume clocks are locked, and never compare benchmarks across power/clock states — the
  ~158 ms/predict figure and eye's 30 fps are 10 W-mode numbers.
- **Memory is zram + swap now** (`docs/CONSTRUCTION.md` §2.2, not in git): 1.7 G zram plus
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
  90 states. Do not use it to qualify TensorRT: a new engine requires separate output
  parity and `resolve()` behavior tests before it can be enabled.
- **The left wheels (FL/RL) are wired inverted — measured 2026-10-14.** `DRIVE_CMD`
  `[++++]` (all four commanded identical, i.e. what a straight line looks like on the
  wire) makes the left pair roll backward and the right pair forward, so the base spins in
  place. Signature: `W`/`S` become rotation, `Q`/`E` become forward/backward, and `A`/`D`
  cancel all three DOF at once (wheels spin, body does not move). It is **not** an ABCD
  port permutation — `[++++]` maps to itself under any permutation, so only a sign flip
  can explain it; do not go hunting in the firmware port table. The correction is the
  `wheel_sign` hardware parameter in `src/rak_description/urdf/rak_chassis.urdf.xacro`
  (default `-1 1 -1 1`), applied by `rak_system.cpp`, which negates that wheel's command
  **and** feedback together so the closed loop stays self-consistent and ROS's
  "positive = rolling forward" becomes true. Override it on
  `chassis_bringup.launch.py wheel_sign:="..."` to A/B without rebuilding; the pass logs
  `wheel_sign = [1 -1 1 -1] (FL FR RL RR)` at startup. The code default stays `+1 1 1 1`
  (neutral) on purpose — the layer must not hardcode one robot's wiring. Re-calibrate
  after any wheel, motor or encoder change.
- **A `read() lost feedback` line is usually a downstream symptom, and `cmd_stale` is not
  a fault.** `kStateFaultMask = 0x003D` in `can_session.cpp` includes `STATE.bit3
  cmd_stale`, which the F407 sets whenever `DRIVE_CMD` stops for 100 ms (PROTOCOL v4
  §6.2) — that is, exactly when no controller is active. So the chain is: controllers
  fail to load → nobody sends `DRIVE_CMD` → deadman trips → `fault_bitmap=0x0008` →
  `read()` errors → controller_manager deactivates the hardware → the controllers can
  never load. **Only a full relaunch escapes it.** The healthy case prints
  `connected=1 handshaken=1 feedback_valid=1 faulted=0 feedback_age=2ms` with `0x0008` the
  only nonzero field; do not read that as a hardware fault. `faulted` itself correctly
  excludes bit3 (it tests `state & 0x0005` plus MODE) — it is the extra
  `fault_bitmap != 0` check inside `read()` that pulls it in, which is a self-latching
  design flaw to be settled with ADR-0017, not by editing the mask ad hoc.
- **Never run two `chassis_bringup` instances.** Both put a node named
  `/controller_manager` on the domain, so the second launch's `spawner`s reach the first
  one's manager and die with `Failed to configure controller` / `Failed loading
  controller` (its YAML was parsed long ago and has no `arm_controller` or
  `gripper_controller`). Check with
  `ps -eo pid,etime,cmd | grep chassis_bringup` — it must print exactly one line — before
  launching.
