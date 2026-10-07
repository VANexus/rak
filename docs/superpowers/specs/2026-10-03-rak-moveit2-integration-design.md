# Rak MoveIt2 Integration — Design Spec

- **Date:** 2026-10-03
- **Author:** yiyuhao (drafted via Claude Code brainstorming)
- **Status:** Pending review
- **Scope:** MoveIt2 motion planning for the Rak robot's arm subsystem
- **Repo:** `rak` (workspace `/home/xrak/rak`)
- **ROS Reviewers:** TBD (team leader)

---

## 1. Overview

Add MoveIt2 motion planning to the Rak robot's **arm subsystem only** (4-DOF geared-motor arm + 1-DOF base stepper = 5 active joints). The mecanum chassis remains on `nav2`; MoveIt never plans chassis motion.

This spec delivers a hand-written URDF/SRDF/XMCR + MoveIt config package, validates a four-milestone chain (`xacro` parse → TF tree → SRDF groups → fake_controller loop), and stops at the boundary of the existing CAN bridge (`nerve`) which is not yet implemented.

> **Engineering priority (user-mandated):** every file must justify its presence (why this file exists), every line must justify its value (why this exact number/structure). Comments cite sources (`CLAUDE.md` / datasheet / estimator / placeholder-with-TODO).

---

## 2. Background

### 2.1 Robot topology (from CLAUDE.md)

```
[USB SBS Camera] → eye → depth → mind → world → CAN bridge (NOT YET)
                                                            ↓
                                            ┌───────────────┴───────────────┐
                                            ↓                               ↓
                                  vehical_rak_arm                   vehical_rak_chassis
                                  STM32F407VET6 · M4F               STM32F407VET6 · M4F
                                  4 joints @ 25 kHz                 4 mecanum wheels @ 25 kHz
                                  + base stepper @ 1 kHz
```

- **Arm details (CLAUDE.md verbatim):** "4-DOF arm. Takes joint targets from MoveIt — the IK lives here, not on the MCU — and closes 4 identical MG513P30_12V geared motors at 25 kHz, plus a base stepper on a 1 kHz position loop."
- **Chassis:** Takes `(Vx, Vy, ωz)` at 100 Hz; runs its own wheel IK; reports measured body velocity at 100 Hz.
- **CAN bus:** Classic CAN 2.0A @ 1 Mbps (no CAN FD). Two STM32F407 leaf nodes.

### 2.2 Current state on xrak (verified 2026-10-03)

- **MoveIt:** not installed (`dpkg -l | grep moveit` empty; `ros2 pkg list | grep -c -i moveit` returns `0`).
- **URDF/SRDF/xacro:** zero files in repo.
- **`boot/scripts/`:** canonical wrapper pattern using Cyclone DDS + `ROS_DOMAIN_ID=45` + setsid + nohup. New scripts in `rak_description` must follow this pattern.
- **`motor_stub/cmd_vel_stub.py`:** Nav2 `/cmd_vel` → `/cmd_vel_raw`. MoveIt will not interact with this (chassis is nav2-only).

### 2.3 Out of scope (this spec)

- Writing `nerve` (CAN bridge Rak-side). Not started; CLAUDE.md says it was never implemented.
- Replacing `brain/`'s Python decision layer with C++. User constraint: only low-level drivers should be C++; high-level logic remains Python.
- Modifying any leaf-node repo (`vehical_rak_arm`, `vehical_rak_chassis`). The user's working scope is **rak repo only**.

---

## 3. Goals & Non-Goals

### 3.1 Goals

1. URDF/SRDF/XMCR for a 5-DOF arm chain (base stepper yaw + 4 arm joints) — **hand-written**, sourced from CLAUDE.md, fully commented.
2. `rak_moveit_config`-style YAML set (joint limits, kinematics, planning, controllers) — hand-written.
3. `display.launch.py` + `demo.launch.py` + `scripts/run_demo.sh` — validate four-milestone chain (`xacro` → TF → SRDF → fake_controller).
4. Single-source URDF: **all geometry/kinematics live in `rak/src/rak_description/`**; no references to external repos.
5. Engineering hygiene: `README.md` per new package, dense comments, `ament_lint_auto` clean, commit messages scoped.

### 3.2 Non-goals

- No real CAN communication. The `fake_joint_driver` (C++, shipped with MoveIt) is the only `FollowJointTrajectory` server in scope.
- No self-collision tuning beyond the obvious adjacency exclusions. The default `<disable_collisions>` set is sufficient for M1-M4.
- No real-time guarantees. MoveIt runs at planning rates; the 25 kHz control loop stays in the leaf-node MCUs.
- No multi-robot / multi-arm. Single-arm configuration only.

---

## 4. Constraints

### 4.1 Hard constraints (from project / system)

- **MoveIt2 only.** Not MoveIt1.
- **Jazzy.** ROS 2 Jazzy (matches `core`, `eye`, existing boot/scripts).
- **Cyclone DDS + domain 45.** Inherited from `boot/scripts/`. `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, `ROS_DOMAIN_ID=45`.
- **Single repo scope.** No cross-repo dependencies. No git submodule to leaf-node repos. No shared-description repo.
- **No GUI-driven config.** No `moveit_setup_assistant`. All files hand-written.

### 4.2 Language constraints (user-mandated 2026-10-03)

- **Low-level drivers** (CAN bridge, joint controllers, `joint_state_publisher`) → **C++**.
- **High-level logic** (decision layer, planning request wrappers, LLM integration) → **Python**.

This applies to **new** code added in this and future iterations. Existing Python (`motor_stub`, `brain/`) is not in scope for rewrite.

**Practical impact for this spec:** the MoveIt Python bindings (`moveit_commander`, `moveit_py`) are not used. Anything wrapping MoveIt on this repo's side would be a thin C++ shim if needed (out of scope this round).

### 4.3 Engineering hygiene (user-mandated)

- Every new file: header comment explaining why it exists.
- Every non-obvious number: comment citing source.
- Placeholder values: explicit `# TODO: replace with measured value` markers.
- Naming consistent with `core/`, `eye/`, `boot/` (snake_case, full English, no abbreviations).
- All packages pass `ament_lint_auto`.
- Commit messages scoped per concern, body explains *why*, not just *what*.

---

## 5. Decision Summary

| # | Decision | Rationale |
|---|---|---|
| D1 | MoveIt2 (not MoveIt1) | Project decision. |
| D2 | Arm-only planning | User-confirmed; nav2 already handles chassis. |
| D3 | URDF/SRDF in `rak/src/rak_description/` | User-confirmed: working scope is rak repo only. |
| D4 | Hand-written URDF/SRDF (no GUI) | Team-leader mandate. |
| D5 | One package (`rak_description`) for description + config | Avoids dual-package synchronization cost. |
| D6 | 5-DOF chain: base stepper yaw + 4 arm joints | CLAUDE.md describes "4 joints 25 kHz + base stepper 1 kHz". |
| D7 | Per-link length = 20 cm | Team-leader mandate. |
| D8 | `joint_state_publisher` (C++) not `joint_state_publisher_gui` (Python) | Constraint 4.2; demo uses RViz InteractiveMarker for drag. |
| D9 | `fake_joint_driver` (C++, MoveIt-shipped) as the controller | nerve not yet implemented; no real CAN. |
| D10 | Validation by four-milestone chain | M1 (parse) → M2 (TF tree) → M3 (SRDF groups in RViz) → M4 (controller loop). |

---

## 6. Architecture

### 6.1 Component diagram

```
                          ┌────────────────────────────────┐
                          │   rak_description (this spec)   │
                          │                                │
                          │  urdf/rak.urdf.xacro  ──────── │ ← hand-written, dense comments
                          │  urdf/rak.srdf        ──────── │ ← hand-written
                          │  config/*.yaml        ──────── │
                          │  launch/*.launch.py   ──────── │
                          │  scripts/run_demo.sh  ──────── │ ← Cyclone DDS + domain 45
                          └─────────────┬──────────────────┘
                                        │ robot_description (param)
                                        ▼
                          ┌────────────────────────────────┐
                          │ robot_state_publisher          │
                          │ (Jazzy apt, C++)                │ → /tf_static
                          └────┬───────────────────────────┘
                                │
            ┌───────────────────┼───────────────────────────┐
            ▼                   ▼                            ▼
    ┌──────────────┐   ┌──────────────────┐    ┌──────────────────────┐
    │ RViz2 (Jazzy)│   │  move_group      │    │ joint_state_publisher│
    │ C++          │   │  (Jazzy apt, C++) │    │ (C++) ────→ /joint_ │
    │              │   │  KDL kinematics   │    │         states       │
    └──────────────┘   └─────┬────────────┘    └──────────────────────┘
                              │ FollowJointTrajectory action
                              ▼
                    ┌──────────────────────────┐
                    │ fake_joint_driver (C++)  │ ← MoveIt-shipped
                    │ ROS_DOMAIN_ID=45         │
                    └─────────────┬────────────┘
                                  │ (eventually: nerve / STM32F407 CAN)
                                  ▼
                          [ STM32F407 arm MCU ]
                          (future: out of scope here)
```

### 6.2 Data flow at M4 (fake-controller loop)

1. RViz InteractiveMarker (move_group-owned) → `x, y, z, qx, qy, qz, qw` goal pose.
2. `move_group` calls KDL IK → 5 joint targets.
3. `move_group` sends `FollowJointTrajectory` action to `/fake_joint_trajectory_controller`.
4. `fake_joint_driver` interpolates trajectory, publishes each waypoint as `/joint_states`.
5. `robot_state_publisher` consumes `/joint_states`, updates `/tf`.
6. RViz renders the moving arm.

### 6.3 Interfacing with existing stack

| Existing component | MoveIt interaction | This spec touches it? |
|---|---|---|
| `core` (CUDA/TensorRT/VPI shared lib) | None | No |
| `eye` (stereo capture) | None | No |
| `boot/scripts/run_*.sh` | Mimic the wrapper pattern in `rak_description/scripts/run_demo.sh` | Adds new file, doesn't modify boot |
| `motor_stub/cmd_vel_stub.py` | MoveIt doesn't touch `/cmd_vel` | No |
| `brain/rak_ai` (Python decision layer) | None this round; future integration possible | No |
| `nerve` (planned) | Future: takes joint targets from `/fake_joint_trajectory_controller` instead of fake driver | No (out of scope) |

---

## 7. Detailed Design

### 7.1 Package layout

```
rak/
└── src/
    └── rak_description/
        ├── CMakeLists.txt
        ├── package.xml
        ├── README.md
        ├── LICENSE                                (Apache-2.0, matches `core`)
        ├── urdf/
        │   ├── rak.urdf.xacro                    # single URDF entry point
        │   ├── rak.srdf                          # semantic description
        │   ├── macros/
        │   │   ├── inertial_macros.xacro         # box/cylinder inertia calculators
        │   │   └── materials.xacro               # shared materials
        │   └── properties.xacro                  # tunable lengths (link length = 0.20 m default)
        ├── meshes/
        │   └── arm/
        │       └── README.md                     # placeholder; STL mesh files TBD
        ├── config/
        │   ├── joint_limits.yaml
        │   ├── kinematics.yaml                   # KDL (default); IKFast optional later
        │   ├── ompl_planning.yaml                # RRT* (default) + PRM
        │   ├── fake_controllers.yaml             # FollowJointTrajectory → fake driver
        │   └── rviz_view.rviz                    # copy of `boot/config/rviz_view.rviz` style
        ├── launch/
        │   ├── view_urdf.launch.py               # M1-M2: headless-friendly RSP + joint_state_publisher
        │   ├── display.launch.py                 # M2: RSP + joint_state_publisher + RViz
        │   └── demo.launch.py                    # M3-M4: + move_group + fake_joint_driver
        └── scripts/
            └── run_demo.sh                       # Cyclone DDS + domain 45 + setsid pattern
```

### 7.2 URDF: link & joint table

| # | Link | Parent | Joint | Joint type | Range / continuous | Axis | Length | Source |
|---|---|---|---|---|---|---|---|---|
| 1 | `base_link` | world (virtual) | `virtual_joint/world_to_base` (in SRDF) | planar | — | — | — | Nav2-driven chassis origin |
| 2 | `arm_root_link` | `base_link` | `base_pan_joint` | continuous | ±∞ | (0,0,1) | — | "base stepper on 1 kHz position loop" (CLAUDE.md) |
| 3 | `shoulder_link` | `arm_root_link` | `shoulder_pitch_joint` | revolute | ±90° | (0,1,0) | 0.0 m (the joint origin is on `arm_root_link` top) | 4 joints at 25 kHz (CLAUDE.md) |
| 4 | `upper_arm_link` | `shoulder_link` | `elbow_pitch_joint` | revolute | ±135° | (0,1,0) | 0.20 m | Team-leader mandate |
| 5 | `forearm_link` | `upper_arm_link` | `wrist_pitch_joint` | revolute | ±90° | (0,1,0) | 0.20 m | Team-leader mandate |
| 6 | `wrist_link` | `forearm_link` | `wrist_roll_joint` | continuous | ±∞ | (1,0,0) | 0.20 m | Team-leader mandate (4th joint) |
| 7 | `gripper_link` | `wrist_link` | `gripper_mount_joint` | fixed | — | — | 0.05 m | placeholder (gripper not yet designed) |

**Total arm reach:** ~0.65 m from `arm_root_link` + gripper.

### 7.3 URDF: xacro macro strategy

- **Single source of tunable values.** `properties.xacro` exposes `<xacro:property name="link_length" value="0.20"/>` (and the per-link offsets). Changing `link_length` regenerates all link dimensions and inertias.
- **Inertia macros.** `inertial_macros.xacro` defines `box_inertial`, `cylinder_inertial` — every `<inertial>` block uses these. **No raw `<inertia ixx="0.1">`** anywhere.
- **Material macros.** `materials.xacro` centralizes colors. One color per link, named by link.
- **Comment density.** Each link block: 4 lines of comment: (a) physical source, (b) geometric source, (c) inertia source, (d) TODO if applicable.

### 7.4 SRDF

```xml
<?xml version="1.0"?>
<robot name="rak">
  <!-- virtual_joint: chassis is planar in /world, driven by nav2 (not MoveIt). -->
  <virtual_joint name="world_to_base" type="planar"
                 parent_frame="world" child_link="base_link"/>

  <!-- planning group: the full 5-DOF arm chain -->
  <group name="arm_with_base">
    <chain base_link="base_link" tip_link="gripper_link"/>
  </group>

  <!-- end effector: gripper mount -->
  <end_effector name="gripper_eef" parent_link="gripper_link"
                group="arm_with_base"/>

  <!-- self-collision matrix: disable adjacent pairs only. -->
  <disable_collisions link1="base_link"       link2="arm_root_link"  reason="adjacent"/>
  <disable_collisions link1="arm_root_link"   link2="shoulder_link"   reason="adjacent"/>
  <disable_collisions link1="shoulder_link"   link2="upper_arm_link"  reason="adjacent"/>
  <disable_collisions link1="upper_arm_link"  link2="forearm_link"    reason="adjacent"/>
  <disable_collisions link1="forearm_link"    link2="wrist_link"      reason="adjacent"/>
  <disable_collisions link1="wrist_link"      link2="gripper_link"    reason="adjacent"/>
  <!-- Intentionally NOT disabled: base_link ↔ upper_arm_link (must be checked at planning time) -->
</robot>
```

### 7.5 MoveIt config YAML (hand-written skeleton)

- `joint_limits.yaml` — populated from CLAUDE.md's control-loop rates (25 kHz / 1 kHz → derive upper velocity/acceleration bounds; flagged as `TODO: replace with measured limits`).
- `kinematics.yaml` — `kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin`, search timeout 0.05 s, default 5 attempts.
- `ompl_planning.yaml` — `RRTConnect` as default planner, `RRTstar` for optimization requests.
- `fake_controllers.yaml` — maps `arm_with_base` group to `fake_joint_trajectory_controller` (MoveIt-shipped C++ controller).

### 7.6 Launch files

- `view_urdf.launch.py` — `robot_state_publisher` + `joint_state_publisher` (C++, zero-positions). No RViz. Headless-friendly; suitable for `M1` validation.
- `display.launch.py` — `view_urdf.launch.py` + RViz2 with our `rviz_view.rviz`. `M2` validation.
- `demo.launch.py` — `display.launch.py` + `move_group` + `fake_joint_driver` + `rviz_virtual_joint_plugin` for InteractiveMarker. `M3`-`M4` validation.

### 7.7 Scripts

- `scripts/run_demo.sh` — copies the pattern from `boot/scripts/run_*.sh`:
  - exports `DISPLAY`, `XAUTHORITY`, `QT_QPA_PLATFORM`
  - exports `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, `ROS_DOMAIN_ID=45`, `CYCLONEDDS_URI`
  - sources `/opt/ros/jazzy/setup.bash` and `/home/xrak/rak/install/setup.bash` twice
  - `setsid nohup ros2 launch rak_description demo.launch.py < /dev/null > /tmp/demo_run.log 2>&1 &`

---

## 8. Validation Strategy

Four-milestone chain, each must succeed before continuing.

| Milestone | Steps | Failure cause |
|---|---|---|
| **M1 — URDF parses** | `ros2 run xacro xacro urdf/rak.urdf.xacro > /tmp/rak.urdf` succeeds (no xacro errors) AND a downstream parse check passes (use `urdfdom-tools` if available, else a Python `urdf_parser` script). | Missing closing tag, malformed xacro property, invalid inertia |
| **M2 — TF tree resolves** | `ros2 launch rak_description view_urdf.launch.py` + `ros2 run tf2_tools view_frames.py` produces 7-segment PDF (world→base→root→shoulder→upper→forearm→wrist→gripper) | Joint parent/child reversed, missing joint origin |
| **M3 — SRDF groups work in RViz** | `ros2 launch rak_description demo.launch.py` + drag in RViz; arm plans & moves | SRDF group names don't match config yaml; KDL cannot reach target |
| **M4 — fake_controller loop** | `ros2 topic echo /joint_states` shows values changing after demo | controllers.yaml action name mismatch; fake_joint_driver not loaded |

> M3 and M4 require Jetson local `:1` display (per memory K2). `view_urdf.launch.py` is headless-friendly; M1 + M2 can be validated over a pure SSH session without display.

---

## 9. Implementation Phases

| Phase | Deliverable | Approx time | Validation gate |
|---|---|---|---|
| P0 | Install MoveIt2 apt packages on xrak: `ros-jazzy-moveit` (metapackage), `ros-jazzy-moveit-planners-ompl`, `ros-jazzy-moveit-fake-controller-manager`, `ros-jazzy-joint-state-publisher` (C++ variant, NOT the Python GUI), `ros-jazzy-robot-state-publisher`, `ros-jazzy-xacro`, `ros-jazzy-tf2-tools`, `ros-jazzy-rviz2`, `ros-jazzy-rviz-visual-tools`. Explicitly DO NOT install `ros-jazzy-joint-state-publisher-gui` (Python; excluded per constraint 4.2). | 30 min | `ros2 pkg list | grep -c -i moveit` returns non-zero |
| P1 | `rak_description` package skeleton (CMakeLists, package.xml, README, LICENSE) builds cleanly | 30 min | `colcon build --packages-select rak_description` succeeds |
| P2 | `properties.xacro` + `inertial_macros.xacro` + `materials.xacro` + `rak.urdf.xacro` | 4–6 h | **M1 passes** |
| P3 | `rak.srdf` + `config/joint_limits.yaml` + `kinematics.yaml` + `ompl_planning.yaml` + `fake_controllers.yaml` | 1.5 h | **M2 passes** (headless) |
| P4 | `launch/view_urdf.launch.py` + `launch/display.launch.py` + `launch/demo.launch.py` + `scripts/run_demo.sh` | 1.5 h | **M3 passes** |
| P5 | `meshes/arm/README.md` placeholder; first commit | 30 min | `git log` shows scoped commit |
| **Total** | | **~10 h** | **M4 passes** (this spec's exit criterion) |

After M4 the package is "demo-ready": anyone with ssh access to xrak can `run_demo.sh` and see the arm plan in RViz. **Production-ready** status (real CAN integration, IK-fast solver, joint-limit measurement) is out of scope and tracked in §10.

---

## 10. Future Work (Out of Scope This Round)

These are **recorded**, not committed:

1. **`nerve` (Rust/CAN bridge).** When implemented, swap `fake_joint_driver` for a real controller that publishes `/joint_states` from CAN feedback and consumes joint targets as CAN frames. MoveIt config (`controllers.yaml`) needs only the controller manager swap.
2. **IKFast solver.** Replace KDL with IKFast once the arm geometry is final. IKFast requires a URDF → DH-parameters step; not needed for first-cut demo.
3. **Mesh files.** Replace placeholder `box` geometry with STL meshes once the mechanical design is finalized. Currently meshes are simplified boxes.
4. **Joint-limit calibration.** Replace placeholder velocity/acceleration bounds with measured values from the actual STM32F407 control loop.
5. **`brain/` Python decision layer integration.** Future: `brain` calls MoveIt via a C++ shim (`MoveItCommander.cpp`) rather than Python bindings. Out of scope; tracked for traceability.
6. **Multi-robot / multi-arm.** Not in scope; SRDF is single-arm.

---

## 11. Risks

| Risk | Probability | Impact | Mitigation |
|---|---|---|---|
| KDL fails to solve some reachable poses for 5-DOF arm | Medium | High (M3 fails) | Document the failing poses; flag for §10 IKFast work |
| Cyclone DDS + move_group memory pressure on Jetson | Medium | High | Profile; fall back to FastDDS only as last resort (team-leader mandated Cyclone) |
| `joint_state_publisher` C++ vs Python mismatch in Jazzy | Low | Medium | Verify Jazzy apt package metadata before P2 |
| Mesh STL files unavailable when mechanical design is in flux | Low | Low | Boxes are valid geometry; only visual quality suffers |
| Future `nerve` integration may invalidate controller-manager choices | Low | Medium | Keep `fake_controllers.yaml` separate from `controllers.yaml` so swap is one-file change |

---

## 12. References

- `CLAUDE.md` — robot topology (chassis + arm), CAN bus details, package layout
- `AGENTS.md` — colcon build order (core → eye → brain), engineering conventions
- `brain/rak_ai/types.py` — frozen schema constraint (not relevant to this spec but referenced for context)
- `boot/scripts/run_*.sh` — wrapper pattern for new `run_demo.sh`
- `boot/scripts/README.md` — documented pattern for setsid + Cyclone DDS + domain 45
- Memory `xrak-rak-project` K2 — Jetson local `:1` display path (RViz must use this)

---

## 13. Open Questions (to ask team leader before P3)

1. **`shoulder_pitch_joint` axis:** assumed `(0, 1, 0)`. If the arm is mounted sideways on the chassis, this is wrong. (Photograph of the arm would resolve.)
2. **`wrist_roll_joint` axis:** assumed `(1, 0, 0)`. Last link's continuous rotation axis — verify against mechanical drawing.
3. **Gripper:** placeholder `fixed` link. Is the gripper a separate motor (becomes a 5-DOF arm + gripper = 6-DOF) or a passive jaw (stays fixed link)?
4. **Joint limits (velocity/acceleration):** placeholders are conservative. Actual values need measurement.

If answers are not available, default values + `# TODO: confirm` markers will be used; spec exit criterion (M4) is unaffected.