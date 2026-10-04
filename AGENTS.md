# AGENTS.md

Layered embodied robot system for Jetson Orin Nano (JetPack 7.2). C++/ROS 2 (Jazzy) + a
plain-Python decision layer. `README.md` is the primary design doc — read it before
changing behavior; it records decisions that are not obvious from the code.

## Layout

The 2026-10-04 unified migration supersedes the old package layout.
Read `docs/architecture.md` and `docs/migration.md` before moving package boundaries.

- `src/rak_interfaces`: messages/actions/services only, no GPU/model dependencies.
- `src/rak_gpu`: JetPack library, exported target `rak_gpu::gpu`, namespace `rak::gpu`.
- `src/rak_stereo_driver`: plugin `rak::stereo_driver::CaptureNode`, executable `stereo_node`.
- `src/rak_astra_driver`: tested depth camera adapter, executable `astra_node`.
- `src/rak_sensor_bringup`: camera/lidar launch and device configuration.
- `src/rak_description`: geometry and visualization, still a legacy five-axis demo.
- `src/rak_moveit_config`: SRDF, IK and planning configuration; six-axis fixture is simulation-only.
- `src/rak_motion_stub`: test relay/estimate, not a hardware driver.
- `src/rak_navigation`: SLAM, omnidirectional Nav2 and velocity safety chain.
- `src/rak_task_executor`: sole owner of text-navigation task and Nav2 goal lifecycle.
- `src/rak_base_driver`: Jetson-to-F407 UART v1 host driver; hardware disabled by default.
- `src/rak_arm_simulation`: software-only six-axis trajectory/gripper action fixture.
- `src/rak_bringup`: composition, DDS configuration and operator scripts.
- `src/vendor`: third-party code, read-only.
- `brain/`: ROS package `rak_agent`, Python module `rak_ai`, pip project `rak-brain`.
  Preserve its physical location, `.venv`, data and pip workflow. Do not add COLCON_IGNORE.

Target arm: six positioning joints plus an independent gripper. Never invent the
missing axis or calibration from the legacy URDF. Models, topics and episode schemas
were not redesigned during the package migration.

## Commands

No CI, no linter/formatter config, no pre-commit. Verification commands:

```bash
# C++ / ROS 2
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src brain --executor sequential --build-base build/architecture --install-base install/architecture --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/architecture/local_setup.bash
python3 scripts/check_workspace.py --installed
colcon test --build-base build/architecture --packages-select rak_description rak_moveit_config
python3 -m pytest src/rak_task_executor/test src/rak_arm_simulation/test
PYTHONPATH=src/rak_base_driver python3 -m pytest src/rak_base_driver/test
ros2 run rak_stereo_driver stereo_node
ros2 launch rak_sensor_bringup lidar.launch.py

# Memory/DMA mode of the board itself (needs sudo; see Gotchas)
sudo scripts/rak-mode.sh status | headless | desktop

# The brain as a node (model + deps come from brain/.venv, resolved at run time)
ros2 launch rak_bringup agent.launch.py decider:=laya sim:=true
ros2 run rak_agent agent_node --ros-args -p sim:=true -p decider:=stub
# -p s1_weights:=fp32 is the escape hatch if a decision ever looks wrong; see Gotchas.

# Python decision layer, no ROS (from brain/)
python3 -m venv .venv && .venv/bin/pip install -e ".[laya]"
env -u PYTHONPATH .venv/bin/python -m rak_ai.loop --scenario approach --decider stub --ticks 50
env -u PYTHONPATH .venv/bin/python tests/test_ai.py     # plain asserts, no pytest needed
env -u PYTHONPATH .venv/bin/python tests/test_laya_fp16_parity.py  # needs the model + GPU, ~10 min
```

`env -u PYTHONPATH` is mandatory for anything run inside the venv: `~/.bashrc` sources
ROS unconditionally and injects `/opt/ros/jazzy/.../site-packages` into the venv,
defeating its isolation. It does **not** apply to `ros2 run`/`ros2 launch`, which need
that same PYTHONPATH to find `rclpy` and the install space.

Text navigation smoke test: source `install/architecture/local_setup.bash`, then run
`ros2 launch rak_task_executor task_simulation.launch.py` and in another terminal run
`ros2 run rak_task_executor task_cli`. This fixture uses fake Nav2 and explicitly estimated
base telemetry; it does not use sensors or represent physical feedback. For sensor-backed
Nav2 composition use `ros2 launch rak_bringup task_navigation.launch.py motion_backend:=sim`.
Current progress and acceptance gaps are tracked in `docs/project_status.md`.

Build `rak_interfaces` before `rak_agent`, and `rak_gpu` before `rak_stereo_driver`;
colcon knows the order from package manifests. Treat `colcon build` +
`tests/test_ai.py` as the two checks to pass before claiming done.

## Constraints that are easy to break

- **System 1 (Laya) is a direct function call.** Never route it through an orchestration
  framework or a cross-process boundary. The RAI harness belongs only on the System 2
  slow path.
- **`guard()` is the only authority** (deterministic geometric veto, overrides everything).
  `gate()` only decides escalation tier. System 2 may narrow authority, never widen it.
  Laya's confidence is out-of-domain and untrusted — a malformed model answer is a normal
  path (→ LOW → escalate), not an exception.
- **`brain/rak_ai/types.py` is frozen once episodes exist.** `state_text` is stored
  verbatim in `brain/data/episodes/*.jsonl`; regenerating it silently corrupts the
  finetune corpus. There is deliberately no training pipeline here.
- `--decider stub` is the default and stays runnable without torch or a model — torch on
  aarch64/JetPack is a real install risk. Do not make `laya`/`rai` hard dependencies
  (both are optional extras in `brain/pyproject.toml`).
- `rak_task_executor` is the only task/goal lifecycle owner. Pause is not reported until
  Nav2 cancellation is confirmed and fresh repeated stationary telemetry arrives; resume
  submits the semantic map goal again. Its default parser is local; optional RAI planning
  may only choose a configured waypoint alias and never issues velocity/joint commands.
- `BaseUartNode` defaults to no port, no hardware access, zero speed limits and DISARMED.
  The bringup launch currently supplies `/dev/wheel_motor`, but `hardware_enable=false`
  means the node does not open it. All three finite positive speed limits are required
  before ARM; zero is not pass-through. UART v1 is a proposed host protocol; this repository
  has no matching F407 firmware. PTY tests do not prove device compatibility.
  Never change defaults to ARM as a shortcut.
- The six-axis MoveIt fixture and `rak_arm_simulation` action servers are software-only.
  They contain test geometry/limits and must not be connected to hardware.
- LLM key comes from `OPENAI_API_KEY`; `brain/config.toml` points `[openai]` at
  SiliconFlow and must never contain the key.

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
- `brain/data/`, `brain/.venv/`, `build/`, `install/`, `log/` are gitignored. Stray
  `frames_*.gv` / `frames_*.pdf` at the root are debugging artifacts, not deliverables.
- `brain/pyproject.toml` says `dynamic = ["requires-python"]` and `setup.py` supplies the
  value **on purpose**: setuptools' pyproject loader turns `requires-python` into a
  `SpecifierSet` object, and colcon reads the setup.py distribution through
  `ast.literal_eval`, so moving it back under `[project]` makes `colcon build` fail with
  `SyntaxError: invalid syntax (<unknown>, line 1)`.
- `brain/scripts/agent_node` is a stdlib-only bootstrap, not a `console_scripts` entry
  point: colcon builds scripts with its own system `python3`, which cannot `import
  pydantic`. It re-execs `$RAK_BRAIN_PYTHON` or `<repo>/brain/.venv/bin/python`, so the
  venv never needs to be activated at build time.
- The 1024x600 touchscreen and the camera compete for the same pool. A logged-in
  GNOME session drives `CmaFree` to **0 kB** of the 256 MiB CMA area, and `eye`
  asks that pool for 4 `V4L2_MEMORY_MMAP` buffers -- so capture fails in a way that
  looks like a camera bug but is a memory decision. `scripts/rak-mode.sh headless`
  stops and disables gdm (~1.8 -> ~2.3 GiB available, 75-160 MiB CMA back).
  GUI launchers require `desktop`; migrated sensor/navigation wrappers do not
  require DISPLAY. Autologin is off, so returning to the desktop requires login.
- `scripts/serial_100hz.py` is the PTY-only UART v1 test. `scripts/serial_link_test.py`
  is refusal-only for the retired CRC-8 physical-port utility; keep it that way.
- Simulation task telemetry is a command-motion estimate, not measured odometry. A
  successful software pause/resume test is not hardware acceptance; see
  `docs/hardware_acceptance.md`.
- nvpmodel sits in the **10W** mode on this board (6 cores, capped EMC bandwidth),
  so the ~158 ms/predict figure and eye's 30 fps are both measured under that cap.
  Raising it (`sudo nvpmodel -m 0 && sudo jetson_clocks`) is root and changes the
  thermal envelope -- do not assume a benchmark is comparable across the two.
- Laya costs **~1.4 GB of this box's 3.4 GB of unified memory** to sit on the GPU:
  804 MiB of resident weights (421M params, bf16) plus ~600 MiB of CUDA context and
  cuDNN/cuBLAS workspace, measured at 158 ms/predict. It used to need ~1.9 GB, and
  that was pure waste: the checkpoint is F16 on disk, but `laya.load()` builds its
  module in torch's *default* dtype (fp32) and `load_state_dict` copies up into
  those parameters, so the weights sat resident at 1606 MiB. Worse, on Tegra CUDA's
  notion of "free" is just the pages nobody is holding, so the fp32 build peaked on
  the CPU side and starved the GPU side at the same moment -- `laya.load(model)`
  OOMed and fell back to CPU before anything could be cast. `LayaDecider` therefore
  loads on CPU with the default dtype already set to the checkpoint's `amp_dtype`
  (bf16) and places afterwards, so fp32 never exists. `s1_weights=auto|fp16|bf16|fp32`
  selects it; `auto` follows the checkpoint.
- Half weights only run where autocast is on, and laya only turns it on for CUDA
  (CPU bf16 is the documented `LAYA_CPU_AMP=bf16`). Without it the forward dies in
  `F.linear`: "mat1 and mat2 must have the same dtype, but got Float and
  BFloat16" -- fp32 activations against half weights. So `LayaDecider` degrades to
  fp32 when there is no CUDA rather than hand back a model that cannot run.
- If it does not fit, `laya` catches the CUDA OOM itself and drops to CPU at
  ~1.6 s/predict -- slower than the sensor rate, with no exception raised.
  `LayaDecider.device/.weights/.place_error` and the node's `s1_device=`/`s1_weights=`
  startup fields say which one you got, and `agent_node` warns on CPU; keep baseline
  RAM low before running `decider:=laya`.
- The fp16/bf16 cast is verified, not assumed: `tests/test_laya_fp16_parity.py`
  compares fp32 against the cast over 90 states and asserts on what `resolve()`
  *executes*, not on the model's label -- at LOW tier the label is discarded and the
  geometric fallback runs, so a label flip there cannot move the robot. Measured:
  0 executed-action, 0 source and 0 tier flips, max |dp| 0.004, 5 label flips all
  at LOW. Autocast does not cover LayerNorm/embeddings/softmax, which is why the
  comparison has to run at all rather than rest on "autocast already used bf16".
