# AGENTS.md

AI-native stereo vision system for Jetson Orin Nano (JetPack 7.2). C++/ROS 2 (Jazzy) + a
plain-Python decision layer. `README.md` is the primary design doc — read it before
changing behavior; it records decisions that are not obvious from the code.

## Layout

- `src/` — colcon workspace (`colcon build` runs from repo root and picks this up **and**
  `brain/`):
  - `core/` — custom msgs (`TrackedObject(s)`, `Action`, `Decision`) + shared
    CUDA/TensorRT/VPI library `core_lib`. `eye` links `core::core_lib` and `brain` imports
    `core.msg`; msg changes require rebuilding `core` first.
  - `eye/` — stereo capture node. Plugin `rak::eye::CaptureNode`, built as executable
    `eye_capture` from component `capture_component`.
  - `boot/` — launch files + config only (lidar bringup, SLAM, brain). No code.
  - `vendor/` — vendored third-party (`rf2o_laser_odometry`, `lslidar_*`). Treat as
    read-only; do not reformat or "fix" it.
- `brain/` — Python package `rak_ai` (System 1 / System 2 decision layer), **also an
  `ament_python` ROS 2 package** (`package.xml`, `setup.py`, `resource/brain`) so
  `colcon build` installs it and `ros2 run brain brain_node` works. `brain/COLCON_IGNORE`
  is gone on purpose — do not re-add it. It is still pip-installable (`pip install -e .`)
  and the CLI loop still runs with ROS unsourced.

## Commands

No CI, no linter/formatter config, no pre-commit. The whole verification surface is:

```bash
# C++ / ROS 2
source /opt/ros/jazzy/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release   # from repo root; builds brain too
source install/setup.bash
ros2 run eye eye_capture
ros2 launch boot lidar.launch.py

# The brain as a node (model + deps come from brain/.venv, resolved at run time)
ros2 launch boot brain.launch.py decider:=laya sim:=true
ros2 run brain brain_node --ros-args -p sim:=true -p decider:=stub

# Python decision layer, no ROS (from brain/)
python3 -m venv .venv && .venv/bin/pip install -e ".[laya]"
env -u PYTHONPATH .venv/bin/python -m rak_ai.loop --scenario approach --decider stub --ticks 50
env -u PYTHONPATH .venv/bin/python tests/test_ai.py     # plain asserts, no pytest needed
```

`env -u PYTHONPATH` is mandatory for anything run inside the venv: `~/.bashrc` sources
ROS unconditionally and injects `/opt/ros/jazzy/.../site-packages` into the venv,
defeating its isolation. It does **not** apply to `ros2 run`/`ros2 launch`, which need
that same PYTHONPATH to find `rclpy` and the install space.

Run order that matters: build `core` before `eye` and `brain` (msg dependency, though
colcon already knows this from `package.xml`). Treat `colcon build` +
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
- LLM key comes from `OPENAI_API_KEY`; `brain/config.toml` points `[openai]` at
  SiliconFlow and must never contain the key.

## Gotchas

- Build files hardcode JetPack paths: VPI at `/opt/nvidia/vpi4`, pkg-config module
  `nvjpeg-13.2`, CUDA headers at `/usr/local/cuda-13.2/targets/sbsa-linux/include`.
  This workspace only builds on the Jetson (aarch64); do not "portable-ize" casually
  without an x86 alternative to verify against.
- C++ is C++17 with `-Wall -Wextra -Wpedantic` (both `core` and `eye`). `eye` runs
  `ament_lint_auto` under `BUILD_TESTING`; `core` has no tests.
- System 2 latency is 12–30 s per call (measured), so S2 is fire-and-forget in a thread
  and replies are applied on a later tick. Stale replies being discarded (~half in a real
  run) and a bounded run taking an extra S2 latency at flush are expected, not bugs.
- `brain/data/`, `brain/.venv/`, `build/`, `install/`, `log/` are gitignored. Stray
  `frames_*.gv` / `frames_*.pdf` at the root are debugging artifacts, not deliverables.
- `brain/pyproject.toml` says `dynamic = ["requires-python"]` and `setup.py` supplies the
  value **on purpose**: setuptools' pyproject loader turns `requires-python` into a
  `SpecifierSet` object, and colcon reads the setup.py distribution through
  `ast.literal_eval`, so moving it back under `[project]` makes `colcon build` fail with
  `SyntaxError: invalid syntax (<unknown>, line 1)`.
- `brain/scripts/brain_node` is a stdlib-only bootstrap, not a `console_scripts` entry
  point: colcon builds scripts with its own system `python3`, which cannot `import
  pydantic`. It re-execs `$RAK_BRAIN_PYTHON` or `<repo>/brain/.venv/bin/python`, so the
  venv never needs to be activated at build time.
- Laya needs **~1.9 GB of this box's 3.4 GB of unified memory** (421M fp32 weights ≈
  1.57 GB + CUDA context) to land on the GPU, where it runs at ~190 ms/predict. If it
  does not fit, `laya` catches the CUDA OOM itself and drops to CPU at ~1.6 s/predict —
  slower than the sensor rate, with no exception raised. `LayaDecider.device` and the
  node's `s1_device=cuda|cpu` startup field say which one you got, and `brain_node`
  warns on CPU; keep baseline RAM low before running `decider:=laya`.
