# rak_description MoveIt2 Integration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Hand-write a MoveIt2 integration for the Rak robot's 5-DOF arm (base stepper yaw + 4 geared-motor joints), validated through a four-milestone chain (M1 xacro parse → M2 TF tree → M3 SRDF groups → M4 fake controller loop).

**Architecture:** Single ROS 2 Jazzy `ament_cmake` package `rak_description` under `rak/src/`. Contains hand-written URDF (xacro), SRDF, MoveIt config YAMLs, launch files, and a `run_demo.sh` that follows the existing `boot/scripts/run_*.sh` pattern (Cyclone DDS + domain 45 + setsid + nohup). No real CAN bridge (`nerve`, out of scope).

**Tech Stack:** ROS 2 Jazzy, MoveIt2 (apt), xacro, KDL kinematics, OMPL (RRT*, RRTConnect), `fake_joint_driver` (MoveIt-shipped C++, no CAN), Cyclone DDS, Jetson Orin Nano (aarch64), GNOME/Xorg on `:1`.

**Spec:** `docs/superpowers/specs/2026-10-03-rak-moveit2-integration-design.md` (commit `4c6f493`). Executors read BOTH this plan AND the spec.

---

## Global Constraints

- **ROS 2 distro:** Jazzy (Ubuntu 24.04, aarch64).
- **DDS:** Cyclone DDS (`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`), `ROS_DOMAIN_ID=45`, `CYCLONEDDS_URI=file:///home/xrak/rak/install/boot/share/boot/config/cyclonedds.xml` (verified exists at install).
- **Single repo scope:** No cross-repo deps. No git submodule.
- **No GUI:** No `moveit_setup_assistant`. All files hand-written.
- **Language split (§4.2 spec):** Low-level drivers C++, high-level Python. This package is config/description only — no executable nodes are added.
- **Explicitly NOT installed:** `ros-jazzy-joint-state-publisher-gui` (Python; use C++ `joint_state_publisher` instead).
- **Engineering hygiene:** Header comment on every file explaining purpose. Source citation comment on every non-obvious value. `# TODO: confirm` on placeholders.
- **Naming:** snake_case, full English, no abbreviations (matches `core`, `eye`, `boot`).
- **Commits:** Conventional Commits scope prefix (`feat:`, `fix:`, `docs:`, `chore:`); body explains *why*.

## Review Focus

Spec implies these failure modes; no task's unit test covers them directly. Each is pinned to the task that owns the code, with a test added in that task's step style:

1. **`properties.xacro` parameter drift.** Changing `link_length` must regenerate inertia in lock-step. Test: vary `link_length` across 3 values, assert inertia tensor entries scale proportionally. *(Task 3)*
2. **SRDF group name vs `kinematics.yaml` chain root typo.** Disables MoveIt planning silently. Test: load `rak.srdf` programmatically and assert group `arm_with_base` exists with chain `base_link` → `gripper_link`. *(Task 5)*
3. **`fake_joint_trajectory_controller` action name mismatch.** MoveIt may default to a different name in Jazzy. Test: `ros2 action list | grep` returns the expected controller name. *(Task 8)*
4. **RViz discovery on Jetson local `:1`.** Display requires `XAUTHORITY=/run/user/1000/gdm/Xauthority`. Test: `run_demo.sh` exports it; `test -S $XAUTHORITY` before launching. *(Task 9)*
5. **`link_length` semantics consistency.** Spec D7 says "per-link length = 20 cm"; Task 4 must use 0.20 m for all 4 arm links (not total arm length). Test: regenerated URDF has `upper_arm/forearm/wrist` lengths of 0.20 m each. *(Task 4)*

---

## File Map

```
rak/src/rak_description/
├── CMakeLists.txt                         [Task 2]
├── package.xml                            [Task 2]
├── README.md                              [Task 2]
├── LICENSE                                [Task 2]
├── urdf/
│   ├── rak.urdf.xacro                     [Task 4]
│   ├── rak.srdf                           [Task 5]
│   ├── macros/
│   │   ├── inertial_macros.xacro          [Task 3]
│   │   └── materials.xacro                [Task 3]
│   └── properties.xacro                   [Task 3]
├── meshes/arm/README.md                   [Task 9]
├── config/
│   ├── joint_limits.yaml                  [Task 6]
│   ├── kinematics.yaml                    [Task 6]
│   ├── ompl_planning.yaml                 [Task 6]
│   ├── fake_controllers.yaml              [Task 6]
│   └── rviz_view.rviz                     [Task 7]
├── launch/
│   ├── view_urdf.launch.py                [Task 7]
│   ├── display.launch.py                  [Task 7]
│   └── demo.launch.py                     [Task 8]
└── scripts/
    └── run_demo.sh                        [Task 9]
```

---

## Task 1: Install MoveIt2 apt packages (P0)

**Files:**
- Create: `/home/xrak/rak/src/rak_description/scripts/validate_moveit_installed.sh`

**Interfaces:**
- Consumes: nothing.
- Produces: xrak has MoveIt2 + supporting packages installed; `install/boot/share/boot/config/cyclonedds.xml` exists.

- [ ] **Step 1: Verify pre-state on xrak (moveit absent).**

```bash
ssh xrak 'bash -lc "source /opt/ros/jazzy/setup.bash && ros2 pkg list | grep -c -i moveit"'
```
Expected: `0`.

- [ ] **Step 2: Install MoveIt2 packages on xrak.**

```bash
ssh xrak "sudo apt update && sudo apt install -y \
  ros-jazzy-moveit \
  ros-jazzy-moveit-planners-ompl \
  ros-jazzy-moveit-fake-controller-manager \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-xacro \
  ros-jazzy-tf2-tools \
  ros-jazzy-rviz2 \
  ros-jazzy-rviz-visual-tools \
  liburdfdom-tools"
```

> Note: `ros-jazzy-joint-state-publisher-gui` is intentionally NOT installed (Python; excluded per spec §4.2).

- [ ] **Step 3: Write the validation script.**

Create `/home/xrak/rak/src/rak_description/scripts/validate_moveit_installed.sh` (the package dir doesn't exist yet — write it anywhere; move at Task 2):

```bash
#!/bin/bash
# validate_moveit_installed.sh — confirms MoveIt2 dependencies for rak_description.
# Spec §9 P0; review focus item #5.
set -e

fail() { echo "FAIL: $*" >&2; exit 1; }

# 1. MoveIt ROS 2 packages
n=$(bash -lc "source /opt/ros/jazzy/setup.bash; ros2 pkg list" | grep -c -i moveit)
[ "$n" -ge 5 ] || fail "ros2 pkg list returned only $n moveit packages (expected >=5)"
echo "OK: $n moveit packages installed"

# 2. C++ joint_state_publisher executable present (NOT the Python GUI variant)
[ -x /opt/ros/jazzy/lib/joint_state_publisher/joint_state_publisher ] \
  || fail "joint_state_publisher (C++) missing"
echo "OK: joint_state_publisher C++ present"

# 3. No joint_state_publisher_gui (Python) — confirm it was NOT installed
if [ -x /opt/ros/jazzy/lib/joint_state_publisher_gui/joint_state_publisher_gui ]; then
  fail "joint_state_publisher_gui (Python) is installed — should NOT be per §4.2"
fi
echo "OK: joint_state_publisher_gui (Python) correctly absent"

# 4. Cyclone DDS config exists at install path
[ -f /home/xrak/rak/install/boot/share/boot/config/cyclonedds.xml ] \
  || fail "cyclonedds.xml missing at install path"
echo "OK: cyclonedds.xml present at install path"

echo "ALL OK"
```

Make executable: `chmod +x /home/xrak/rak/src/rak_description/scripts/validate_moveit_installed.sh` (will be moved in Task 2).

- [ ] **Step 4: Run the validation script.**

```bash
ssh xrak "bash /home/xrak/rak/src/rak_description/scripts/validate_moveit_installed.sh"
```
Expected: `ALL OK` with 4 lines of `OK:` confirmation.

- [ ] **Step 5: Save validation script into repo and commit.**

```bash
ssh xrak "cd /home/xrak/rak && mkdir -p src/rak_description/scripts && mv src/rak_description/scripts/validate_moveit_installed.sh src/rak_description/scripts/validate_moveit_installed.sh.tmp 2>/dev/null; true"
# Write the file directly to its proper location from this session's tmp.
```

The validation script lives at `src/rak_description/scripts/` once Task 2 creates the dir; move then.

```bash
ssh xrak "cd /home/xrak/rak && git add src/rak_description/scripts/validate_moveit_installed.sh && git commit -m 'chore(rak_description): add MoveIt2 install validation script

Spec §9 P0 + review focus item #5. Asserts MoveIt2 packages
are present, joint_state_publisher_gui (Python) is correctly
absent, and the boot/config/cyclonedds.xml dependency exists
at its install path.'"
```
Expected: 1 file added, commit created.

---

## Task 2: Package skeleton (P1)

**Files:**
- Create: `src/rak_description/CMakeLists.txt`
- Create: `src/rak_description/package.xml`
- Create: `src/rak_description/README.md`
- Create: `src/rak_description/LICENSE`
- Modify: `src/rak_description/scripts/validate_moveit_installed.sh` (move into final location; already at scripts/ from Task 1)

**Interfaces:**
- Consumes: nothing.
- Produces: empty `rak_description` package that `colcon build` accepts.

- [ ] **Step 1: Write `package.xml`.**

Create `src/rak_description/package.xml`:

```xml
<?xml version="1.0" encoding="UTF-8"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<!--
  rak_description — MoveIt2 description + config package for the Rak arm.
  Hand-written URDF/SRDF/MoveIt-config per spec §3.1, §4.1. No executable
  nodes are added in this package — it is config + launch + scripts only.
-->
<package format="3">
  <name>rak_description</name>
  <version>0.1.0</version>
  <description>
    Hand-written MoveIt2 description and configuration for the Rak robot's
    5-DOF arm subsystem (base stepper yaw + 4 geared-motor joints). Includes
    URDF (xacro), SRDF, MoveIt config YAMLs, launch files, and a demo runner.
    See docs/superpowers/specs/2026-10-03-rak-moveit2-integration-design.md.
  </description>
  <maintainer email="xrak@example.com">xrak</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <exec_depend>moveit_ros_planning_interface</exec_depend>
  <exec_depend>moveit_ros_planning</exec_depend>
  <exec_depend>moveit_planners_ompl</exec_depend>
  <exec_depend>moveit_fake_controller_manager</exec_depend>
  <exec_depend>moveit_kinematics</exec_depend>
  <exec_depend>joint_state_publisher</exec_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>xacro</exec_depend>
  <exec_depend>rviz2</exec_depend>
  <exec_depend>rviz_visual_tools</exec_depend>
  <exec_depend>tf2_ros</exec_depend>
  <exec_depend>launch</exec_depend>
  <exec_depend>launch_ros</exec_depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_pep257</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [ ] **Step 2: Write `CMakeLists.txt`.**

```cmake
# rak_description — config + launch + scripts only, no compiled nodes.
# Spec §7.1 file layout.
cmake_minimum_required(VERSION 3.8)
project(rak_description)

find_package(ament_cmake REQUIRED)

# Install everything that ships with the package. Subdirs follow the spec
# file layout (urdf/, config/, launch/, scripts/, meshes/).
install(DIRECTORY
  urdf config launch scripts meshes
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_globals()
  ament_lint_auto(INSTALL)
endif()

ament_package()
```

- [ ] **Step 3: Write `LICENSE` (Apache-2.0).**

Use the standard Apache-2.0 text. Copy from `src/core/LICENSE` if convenient.

- [ ] **Step 4: Write `README.md`.**

```markdown
<!--
  rak_description — MoveIt2 integration for the Rak arm.
  Spec: docs/superpowers/specs/2026-10-03-rak-moveit2-integration-design.md
-->

# rak_description

Hand-written MoveIt2 description and configuration for the Rak robot's
5-DOF arm subsystem (base stepper yaw + 4 geared-motor joints).

## Why this package exists

The Rak robot is a three-recon product: this `rak` repo hosts perception
and decision on the Jetson Orin Nano; two STM32F407 leaf-node repos
(`vehical_rak_arm`, `vehical_rak_chassis`) handle real-time control over
CAN. This package provides the **single source of truth** for arm
geometry, kinematics, planning configuration, and demo launch glue so
that MoveIt2 motion planning can run on the Jetson.

## Layout

- `urdf/` — robot description (xacro + SRDF + macros)
- `config/` — MoveIt2 YAML configs (joint limits, kinematics, planning, controllers, RViz view)
- `launch/` — `view_urdf.launch.py`, `display.launch.py`, `demo.launch.py`
- `scripts/` — `validate_moveit_installed.sh`, `run_demo.sh`
- `meshes/` — STL placeholders (boxes until mechanical design finalizes)

## Usage

```bash
# M1-M2 (headless OK):
ros2 launch rak_description view_urdf.launch.py

# M2 visual:
ros2 launch rak_description display.launch.py

# M3-M4 (full demo, requires Jetson local :1):
./scripts/run_demo.sh
```

## Validation

This package targets four milestones:
- **M1** URDF parses (`xacro` exit code 0)
- **M2** TF tree resolves 7 links
- **M3** SRDF groups draggable in RViz
- **M4** `/joint_states` topic changes after planning

See spec §8 for details.
```

- [ ] **Step 5: Build the package.**

```bash
ssh xrak "cd /home/xrak/rak && source /opt/ros/jazzy/setup.bash && colcon build --packages-select rak_description"
```
Expected: build succeeds; `Finished <<< rak_description <<<` line.

- [ ] **Step 6: Commit.**

```bash
ssh xrak "cd /home/xrak/rak && git add src/rak_description/ && git commit -m 'feat(rak_description): package skeleton (CMakeLists, package.xml, README, LICENSE)

Per spec §7.1. ament_cmake, installs urdf/config/launch/scripts/meshes.
Empty subdirs at this point. M1 not yet exercised; build is the
gate this round.'"
```

---

## Task 3: xacro macros — properties, materials, inertias (P2a)

**Files:**
- Create: `src/rak_description/urdf/properties.xacro`
- Create: `src/rak_description/urdf/macros/inertial_macros.xacro`
- Create: `src/rak_description/urdf/macros/materials.xacro`
- Create: `src/rak_description/scripts/validate_macros.py` (Python validation; not a driver — fine under §4.2)

**Interfaces:**
- Consumes: nothing (these are leaf files).
- Produces:
  - `<xacro:property name="link_length" value="0.20"/>` (m) — single tunable for all arm segments.
  - `<xacro:macro name="box_inertial" params="mass Lx Ly Lz xyz:='0 0 0' rpy:='0 0 0'"/>` — solid-box inertia tensor.
  - `<xacro:macro name="cylinder_inertial" params="mass r L xyz:='0 0 0' rpy:='0 0 0'"/>` — solid cylinder inertia tensor.
  - `<material name="arm_aluminum">`, `<material name="base_link_color">`, `<material name="gripper_color">`.

- [ ] **Step 1: Write `properties.xacro`.**

```xml
<?xml version="1.0"?>
<!--
  properties.xacro — single source of tunable geometric parameters.
  Spec §7.3: changing link_length regenerates ALL dependent link
  dimensions and inertias in lock-step. Review focus item #1.
-->
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Per-link length of arm segments (m).
       Spec D7: team-leader mandate. -->
  <xacro:property name="link_length" value="0.20"/>

  <!-- Gripper placeholder length (m). TODO: confirm with team leader -->
  <xacro:property name="gripper_length" value="0.05"/>

  <!-- Approximate mass (kg) of each arm link.
       TODO: replace with measured mass once the mechanical build is weighed. -->
  <xacro:property name="link_mass" value="0.30"/>
  <xacro:property name="gripper_mass" value="0.10"/>

  <!-- Joint range limits (rad). TODO: confirm with mechanical drawing. -->
  <xacro:property name="shoulder_pitch_lower" value="${-pi/2}"/>
  <xacro:property name="shoulder_pitch_upper" value="${ pi/2}"/>
  <xacro:property name="elbow_pitch_lower"   value="${-3*pi/4}"/>
  <xacro:property name="elbow_pitch_upper"   value="${ 3*pi/4}"/>
  <xacro:property name="wrist_pitch_lower"   value="${-pi/2}"/>
  <xacro:property name="wrist_pitch_upper"   value="${ pi/2}"/>

</robot>
```

- [ ] **Step 2: Write `inertial_macros.xacro`.**

```xml
<?xml version="1.0"?>
<!--
  inertial_macros.xacro — solid-shape inertia tensors.
  Spec §7.3: NO raw `<inertia ixx="0.1"/>` anywhere. Every link's
  `<inertial>` block must call one of these macros.
  Sources: standard solid-body formulas (parallel-axis theorem not
  required because all shapes are centered at joint origin).
-->
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Solid box inertia tensor, centered.
       mass: total mass (kg)
       Lx, Ly, Lz: full side lengths (m) -->
  <xacro:macro name="box_inertial" params="mass Lx Ly Lz xyz:='0 0 0' rpy:='0 0 0'">
    <inertial>
      <origin xyz="${xyz}" rpy="${rpy}"/>
      <mass value="${mass}"/>
      <inertia ixx="${mass*(Ly*Ly + Lz*Lz)/12}"
               iyy="${mass*(Lx*Lx + Lz*Lz)/12}"
               izz="${mass*(Lx*Lx + Ly*Ly)/12}"
               ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </xacro:macro>

  <!-- Solid cylinder inertia tensor, axis along Z.
       mass: total mass (kg)
       r: radius (m); L: length along Z (m) -->
  <xacro:macro name="cylinder_inertial" params="mass r L xyz:='0 0 0' rpy:='0 0 0'">
    <inertial>
      <origin xyz="${xyz}" rpy="${rpy}"/>
      <mass value="${mass}"/>
      <inertia ixx="${mass*(3*r*r + L*L)/12}"
               iyy="${mass*(3*r*r + L*L)/12}"
               izz="${mass*r*r/2}"
               ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </xacro:macro>

</robot>
```

- [ ] **Step 3: Write `materials.xacro`.**

```xml
<?xml version="1.0"?>
<!--
  materials.xacro — shared RViz colors.
  Spec §7.3: one color per link family, named by link role.
-->
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <material name="base_link_color">
    <color rgba="0.30 0.30 0.30 1.0"/>  <!-- chassis gray -->
  </material>

  <material name="arm_aluminum">
    <color rgba="0.75 0.75 0.78 1.0"/>  <!-- brushed aluminum -->
  </material>

  <material name="gripper_color">
    <color rgba="0.20 0.40 0.80 1.0"/>  <!-- blue placeholder -->
  </material>

</robot>
```

- [ ] **Step 4: Write `validate_macros.py`.**

Create `src/rak_description/scripts/validate_macros.py`:

```python
#!/usr/bin/env python3
"""validate_macros.py — assertion tests for xacro macros.

Spec §7.3 review focus item #1: properties.xacro parameter drift.
Asserts that varying link_length regenerates inertias proportionally.
"""
import subprocess
import sys
import tempfile
from pathlib import Path

import xacro


def expand_urdf(urdf_path: Path, link_length_override: float | None = None) -> str:
    """Expand the URDF, optionally overriding link_length for the test."""
    mappings = {"link_length": str(link_length_override)} if link_length_override is not None else {}
    doc = xacro.process_file(str(urdf_path), mappings=mappings)
    return doc.toxml()


def test_inertia_scales_with_link_length(urdf_path: Path) -> None:
    """When link_length grows, ixx of the affected link must grow proportionally."""
    from urdf_parser_py.urdf import URDF

    base = URDF.from_xml_string(expand_urdf(urdf_path, link_length_override=0.20))
    bigger = URDF.from_xml_string(expand_urdf(urdf_path, link_length_override=0.40))

    def get_ixx(robot, link_name: str) -> float:
        for link in robot.links:
            if link.name == link_name:
                return link.inertial.inertia.ixx
        raise KeyError(link_name)

    base_ixx = get_ixx(base)
    bigger_ixx = get_ixx(bigger)
    # ixx scales with Lz^2 for a box (here Ly = Lz = link_length); expect ~4x
    ratio = bigger_ixx / base_ixx
    assert 3.5 < ratio < 4.5, f"ixx ratio {ratio:.2f} not ~4x when link_length doubles"


def test_link_length_consistency(urdf_path: Path) -> None:
    """Spec D7 / review focus item #5: 3 arm links must each be 0.20 m."""
    from urdf_parser_py.urdf import URDF

    robot = URDF.from_xml_string(expand_urdf(urdf_path))
    for link_name in ("upper_arm_link", "forearm_link", "wrist_link"):
        link = next((l for l in robot.links if l.name == link_name), None)
        assert link is not None, f"missing link {link_name}"
        # box geometry: size = "Lx Ly Lz"; Lx is along joint axis = link_length
        size_str = link.collision.geometry.box.size
        lx, ly, lz = (float(v) for v in size_str.split())
        assert abs(lx - 0.20) < 1e-6, f"{link_name} length {lx} != 0.20"


def main() -> int:
    pkg_root = Path(__file__).resolve().parents[1]
    urdf_path = pkg_root / "urdf" / "rak.urdf.xacro"
    if not urdf_path.exists():
        print(f"SKIP: {urdf_path} not yet created (Task 4); rerun after Task 4.")
        return 0

    test_link_length_consistency(urdf_path)
    test_inertia_scales_with_link_length(urdf_path)
    print("validate_macros: ALL OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
```

- [ ] **Step 5: Install Python test deps on xrak and run validation (expect SKIP).**

```bash
ssh xrak "sudo apt install -y python3-urdf-parser-py python3-pip && pip3 install --break-system-packages xacro"
ssh xrak "cd /home/xrak/rak && source /opt/ros/jazzy/setup.bash && source install/setup.bash && python3 src/rak_description/scripts/validate_macros.py"
```
Expected first run: `SKIP: .../urdf/rak.urdf.xacro not yet created`. That is correct — Task 4 creates the URDF that this test consumes.

- [ ] **Step 6: Commit.**

```bash
ssh xrak "cd /home/xrak/rak && git add src/rak_description/urdf/properties.xacro src/rak_description/urdf/macros/ src/rak_description/scripts/validate_macros.py && git commit -m 'feat(rak_description): xacro macros (properties, materials, inertias)

Spec §7.3. All link dimensions driven by properties.xacro; inertia
formulas in inertial_macros.xacro; materials centralized. Adds the
validation script that Task 4 will exercise.'"
```

---

## Task 4: Hand-write `rak.urdf.xacro` (P2b, M1 milestone)

**Files:**
- Create: `src/rak_description/urdf/rak.urdf.xacro`

**Interfaces:**
- Consumes: `properties.xacro`, `inertial_macros.xacro`, `materials.xacro` (all from Task 3).
- Produces: a URDF that, when expanded, contains 7 links (`base_link`, `arm_root_link`, `shoulder_link`, `upper_arm_link`, `forearm_link`, `wrist_link`, `gripper_link`) and 6 joints per spec §7.2.

- [ ] **Step 1: Write `rak.urdf.xacro`.**

```xml
<?xml version="1.0"?>
<!--
  rak.urdf.xacro — Rak arm kinematic tree.

  Topology (spec §7.2):
    base_link (virtual planar to /world, in SRDF)
      └─ arm_root_link          (base_pan_joint, continuous, base stepper, 1 kHz)
           └─ shoulder_link    (shoulder_pitch_joint, revolute ±90°)
                └─ upper_arm_link  (elbow_pitch_joint, revolute ±135°) — 0.20 m
                     └─ forearm_link (wrist_pitch_joint, revolute ±90°) — 0.20 m
                          └─ wrist_link (wrist_roll_joint, continuous) — 0.20 m
                               └─ gripper_link (gripper_mount_joint, fixed) — 0.05 m

  Total reach: ~0.65 m from arm_root_link + gripper.
  Sources: CLAUDE.md (arm is "4 joints 25 kHz + base stepper 1 kHz");
           team leader for per-link length = 0.20 m.
-->
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="rak">

  <!-- ============== includes ============== -->
  <xacro:include filename="$(find rak_description)/urdf/properties.xacro"/>
  <xacro:include filename="$(find rak_description)/urdf/macros/inertial_macros.xacro"/>
  <xacro:include filename="$(find rak_description)/urdf/macros/materials.xacro"/>

  <!-- ============== link 1: base_link (chassis) ==============
       Physical source: mecanum chassis origin; nav2 owns TF to /world.
       Geometric source: placeholder box (0.4 x 0.4 x 0.1 m). TODO: replace.
       TODO: confirm base_link dimensions and mass with chassis team. -->
  <link name="base_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><box size="0.40 0.40 0.10"/></geometry>
      <material name="base_link_color"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><box size="0.40 0.40 0.10"/></geometry>
    </collision>
    <xacro:box_inertial mass="5.0" Lx="0.40" Ly="0.40" Lz="0.10"/>
  </link>

  <!-- ============== link 2: arm_root_link ==============
       Physical source: top of the base stepper mount. Fixed-stepper,
       so this link is at the top of the rotating column.
       Geometric source: thin cylinder placeholder (r=0.05, h=0.05). TODO: confirm. -->
  <link name="arm_root_link">
    <visual>
      <origin xyz="0 0 0.025" rpy="0 0 0"/>
      <geometry><cylinder radius="0.05" length="0.05"/></geometry>
      <material name="arm_aluminum"/>
    </visual>
    <collision>
      <origin xyz="0 0 0.025" rpy="0 0 0"/>
      <geometry><cylinder radius="0.05" length="0.05"/></geometry>
    </collision>
    <xacro:cylinder_inertial mass="0.20" r="0.05" L="0.05" xyz="0 0 0.025"/>
  </link>

  <!-- base_pan_joint: continuous rotation around Z. Driven by base stepper.
       Source: CLAUDE.md "base stepper on a 1 kHz position loop". -->
  <joint name="base_pan_joint" type="continuous">
    <parent link="base_link"/>
    <child link="arm_root_link"/>
    <origin xyz="0 0 0.10" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit effort="20.0" velocity="5.0"/>  <!-- TODO: confirm with stepper driver -->
  </joint>

  <!-- ============== link 3: shoulder_link ==============
       Joint origin only — no actual extension. Hosts shoulder_pitch_joint.
       Geometric source: small block placeholder. TODO: confirm. -->
  <link name="shoulder_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><box size="0.06 0.06 0.06"/></geometry>
      <material name="arm_aluminum"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><box size="0.06 0.06 0.06"/></geometry>
    </collision>
    <xacro:box_inertial mass="0.10" Lx="0.06" Ly="0.06" Lz="0.06"/>
  </link>

  <!-- shoulder_pitch_joint: revolute around Z (shrug). Spec §7.2 axis (0,1,0)
       is the team's typical mounting; reviewer focus item #1 to confirm. -->
  <joint name="shoulder_pitch_joint" type="revolute">
    <parent link="arm_root_link"/>
    <child link="shoulder_link"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="${shoulder_pitch_lower}" upper="${shoulder_pitch_upper}"
           effort="30.0" velocity="3.0"/>  <!-- TODO: confirm -->
  </joint>

  <!-- ============== link 4: upper_arm_link (0.20 m) ==============
       Physical source: spec D7 per-link length = 0.20 m. -->
  <link name="upper_arm_link">
    <visual>
      <origin xyz="${link_length/2} 0 0" rpy="0 0 0"/>
      <geometry><box size="${link_length} 0.04 0.04"/></geometry>
      <material name="arm_aluminum"/>
    </visual>
    <collision>
      <origin xyz="${link_length/2} 0 0" rpy="0 0 0"/>
      <geometry><box size="${link_length} 0.04 0.04"/></geometry>
    </collision>
    <xacro:box_inertial mass="${link_mass}" Lx="${link_length}" Ly="0.04" Lz="0.04"
                        xyz="${link_length/2} 0 0"/>
  </link>

  <joint name="elbow_pitch_joint" type="revolute">
    <parent link="shoulder_link"/>
    <child link="upper_arm_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="${elbow_pitch_lower}" upper="${elbow_pitch_upper}"
           effort="30.0" velocity="3.0"/>  <!-- TODO: confirm -->
  </joint>

  <!-- ============== link 5: forearm_link (0.20 m) ============== -->
  <link name="forearm_link">
    <visual>
      <origin xyz="${link_length/2} 0 0" rpy="0 0 0"/>
      <geometry><box size="${link_length} 0.04 0.04"/></geometry>
      <material name="arm_aluminum"/>
    </visual>
    <collision>
      <origin xyz="${link_length/2} 0 0" rpy="0 0 0"/>
      <geometry><box size="${link_length} 0.04 0.04"/></geometry>
    </collision>
    <xacro:box_inertial mass="${link_mass}" Lx="${link_length}" Ly="0.04" Lz="0.04"
                        xyz="${link_length/2} 0 0"/>
  </link>

  <joint name="wrist_pitch_joint" type="revolute">
    <parent link="upper_arm_link"/>
    <child link="forearm_link"/>
    <origin xyz="${link_length} 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="${wrist_pitch_lower}" upper="${wrist_pitch_upper}"
           effort="20.0" velocity="5.0"/>  <!-- TODO: confirm -->
  </joint>

  <!-- ============== link 6: wrist_link (0.20 m) ============== -->
  <link name="wrist_link">
    <visual>
      <origin xyz="${link_length/2} 0 0" rpy="0 0 0"/>
      <geometry><box size="${link_length} 0.04 0.04"/></geometry>
      <material name="arm_aluminum"/>
    </visual>
    <collision>
      <origin xyz="${link_length/2} 0 0" rpy="0 0 0"/>
      <geometry><box size="${link_length} 0.04 0.04"/></geometry>
    </collision>
    <xacro:box_inertial mass="${link_mass}" Lx="${link_length}" Ly="0.04" Lz="0.04"
                        xyz="${link_length/2} 0 0"/>
  </link>

  <joint name="wrist_roll_joint" type="continuous">
    <parent link="forearm_link"/>
    <child link="wrist_link"/>
    <origin xyz="${link_length} 0 0" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit effort="10.0" velocity="10.0"/>  <!-- TODO: confirm -->
  </joint>

  <!-- ============== link 7: gripper_link (0.05 m placeholder) ==============
       Physical source: gripper not yet designed; placeholder fixed link.
       TODO: replace with actual gripper joint (would make this a 6-DOF arm). -->
  <link name="gripper_link">
    <visual>
      <origin xyz="${gripper_length/2} 0 0" rpy="0 0 0"/>
      <geometry><box size="${gripper_length} 0.03 0.03"/></geometry>
      <material name="gripper_color"/>
    </visual>
    <collision>
      <origin xyz="${gripper_length/2} 0 0" rpy="0 0 0"/>
      <geometry><box size="${gripper_length} 0.03 0.03"/></geometry>
    </collision>
    <xacro:box_inertial mass="${gripper_mass}" Lx="${gripper_length}" Ly="0.03" Lz="0.03"
                        xyz="${gripper_length/2} 0 0"/>
  </link>

  <joint name="gripper_mount_joint" type="fixed">
    <parent link="wrist_link"/>
    <child link="gripper_link"/>
    <origin xyz="${link_length} 0 0" rpy="0 0 0"/>
  </joint>

</robot>
```

- [ ] **Step 2: Run xacro + parse validation (M1 milestone).**

```bash
ssh xrak "cd /home/xrak/rak && source /opt/ros/jazzy/setup.bash && source install/setup.bash && \
  ros2 run xacro xacro src/rak_description/urdf/rak.urdf.xacro > /tmp/rak.urdf && \
  check_urdf /tmp/rak.urdf"
```
Expected: `Successfully Parsed XML` and zero errors.

- [ ] **Step 3: Run validate_macros.py — exercises macros AND link_length (review focus #1, #5).**

```bash
ssh xrak "cd /home/xrak/rak && source /opt/ros/jazzy/setup.bash && source install/setup.bash && \
  python3 src/rak_description/scripts/validate_macros.py"
```
Expected: `validate_macros: ALL OK`.

- [ ] **Step 4: Commit.**

```bash
ssh xrak "cd /home/xrak/rak && git add src/rak_description/urdf/rak.urdf.xacro && \
  git commit -m 'feat(rak_description): hand-write 5-DOF arm URDF (M1 passes)

Spec §7.2 link/joint table. 7 links, 6 joints: base_pan (continuous,
base stepper) + 3 revolute pitch + 1 continuous wrist roll + 1 fixed
gripper mount. All inertias via box_inertial/cylinder_inertial macros;
no raw inertia tags. Properties.xacro drives all link lengths so
changing one value regenerates the whole chain. xacro parses clean;
check_urdf passes; validate_macros.py confirms link_length=0.20 m
consistency and proportional inertia scaling.'"
```

---

## Task 5: Write `rak.srdf` (P3a)

**Files:**
- Create: `src/rak_description/urdf/rak.srdf`

**Interfaces:**
- Consumes: `rak.urdf.xacro` link names (must match exactly).
- Produces: SRDF with 1 planning group (`arm_with_base`), 1 end-effector (`gripper_eef`), 1 virtual joint (`world_to_base`), 6 disable_collisions (adjacent pairs only).

- [ ] **Step 1: Write `rak.srdf`.**

```xml
<?xml version="1.0"?>
<!--
  rak.srdf — MoveIt2 semantic description.
  Spec §7.4.
  Review focus item #2: group name and chain must match kinematics.yaml
  in Task 6; mistyping either breaks M4 silently.
-->
<robot name="rak">

  <!-- virtual_joint: chassis is planar in /world, driven by nav2 (not MoveIt). -->
  <virtual_joint name="world_to_base" type="planar"
                 parent_frame="world" child_link="base_link"/>

  <!-- planning group: full 5-DOF arm chain -->
  <group name="arm_with_base">
    <chain base_link="base_link" tip_link="gripper_link"/>
  </group>

  <!-- end effector: gripper mount -->
  <end_effector name="gripper_eef" parent_link="gripper_link"
                group="arm_with_base"/>

  <!-- self-collision matrix: disable adjacent pairs only.
       Intentionally NOT disabled: base_link ↔ upper_arm_link
       (must be checked at planning time, may collide at extreme configs). -->
  <disable_collisions link1="base_link"       link2="arm_root_link"  reason="adjacent"/>
  <disable_collisions link1="arm_root_link"   link2="shoulder_link"  reason="adjacent"/>
  <disable_collisions link1="shoulder_link"   link2="upper_arm_link" reason="adjacent"/>
  <disable_collisions link1="upper_arm_link"  link2="forearm_link"   reason="adjacent"/>
  <disable_collisions link1="forearm_link"    link2="wrist_link"     reason="adjacent"/>
  <disable_collisions link1="wrist_link"      link2="gripper_link"   reason="adjacent"/>

</robot>
```

- [ ] **Step 2: Validate SRDF — group + chain exist (review focus item #2).**

```bash
ssh xrak "cd /home/xrak/rak && source /opt/ros/jazzy/setup.bash && source install/setup.bash && \
  python3 -c \"
from srdf_parser_py.srdf import SRDF
srdf = SRDF.from_xml_file('src/rak_description/urdf/rak.srdf')
g = next((x for x in srdf.groups if x.name == 'arm_with_base'), None)
assert g is not None, 'group arm_with_base missing'
assert g.chains[0].base_link == 'base_link'
assert g.chains[0].tip_link == 'gripper_link'
print('SRDF OK: arm_with_base group present with correct chain')
\""
```
Expected: `SRDF OK: arm_with_base group present with correct chain`.

If `srdf_parser_py` is missing: `sudo apt install -y python3-srdf-parser-py`.

- [ ] **Step 3: Commit.**

```bash
ssh xrak "cd /home/xrak/rak && git add src/rak_description/urdf/rak.srdf && \
  git commit -m 'feat(rak_description): SRDF with arm_with_base group + EE

Spec §7.4. 5-DOF chain base_link->gripper_link. 6 disable_collisions
(adjacent pairs only); base_link<->upper_arm_link intentionally left
enabled to keep the planning-time check honest.'"
```

---

## Task 6: MoveIt config YAMLs (P3b)

**Files:**
- Create: `src/rak_description/config/joint_limits.yaml`
- Create: `src/rak_description/config/kinematics.yaml`
- Create: `src/rak_description/config/ompl_planning.yaml`
- Create: `src/rak_description/config/fake_controllers.yaml`

**Interfaces:**
- Consumes: joint names from `rak.urdf.xacro`, group name from `rak.srdf`.
- Produces: 4 YAML files that `move_group` loads at startup.

- [ ] **Step 1: Write `joint_limits.yaml`.**

```yaml
# joint_limits.yaml — joint velocity / acceleration / jerk bounds.
# Spec §7.5. Placeholders conservative; TODO: replace with measured limits
# from STM32F407 control loop.
joint_limits:
  base_pan_joint:
    has_velocity_limits: true
    max_velocity: 5.0           # rad/s, stepper ceiling. TODO: confirm.
    has_acceleration_limits: true
    max_acceleration: 10.0      # rad/s^2. TODO: confirm.
  shoulder_pitch_joint:
    has_velocity_limits: true
    max_velocity: 3.0
    has_acceleration_limits: true
    max_acceleration: 6.0
  wrist_pitch_joint:
    has_velocity_limits: true
    max_velocity: 3.0
    has_acceleration_limits: true
    max_acceleration: 6.0
  # elbow_pitch_joint / wrist_roll_joint: inherit group defaults below
```

- [ ] **Step 2: Write `kinematics.yaml`.**

```yaml
# kinematics.yaml — KDL solver config.
# Spec §7.5: KDL default; IKFast optional later (review focus item #2).
arm_with_base:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.05
  kinematics_solver_attempts: 5
```

- [ ] **Step 3: Write `ompl_planning.yaml`.**

```yaml
# ompl_planning.yaml — OMPL planner config.
# Spec §7.5: RRTConnect default, RRTstar for optimization requests.
arm_with_base:
  planner_configs:
    - RRTConnectkConfigDefault
  projection_evaluator: joints(base_pan_joint,shoulder_pitch_joint,elbow_pitch_joint)
  longest_valid_segment_fraction: 0.05
```

- [ ] **Step 4: Write `fake_controllers.yaml`.**

```yaml
# fake_controllers.yaml — MoveIt fake controller manager config.
# Spec D9: fake_joint_driver is the only controller until CAN bridge lands.
# Spec §7.5.
controller_names:
  - arm_with_base_controller
arm_with_base_controller:
  action_ns: follow_joint_trajectory
  type: FollowJointTrajectory
  default: true
  joints:
    - base_pan_joint
    - shoulder_pitch_joint
    - elbow_pitch_joint
    - wrist_pitch_joint
    - wrist_roll_joint
```

- [ ] **Step 5: Validate YAMLs parse.**

```bash
ssh xrak "cd /home/xrak/rak && source /opt/ros/jazzy/setup.bash && source install/setup.bash && \
  python3 -c \"
import yaml
for name in ('joint_limits','kinematics','ompl_planning','fake_controllers'):
    with open(f'src/rak_description/config/{name}.yaml') as f:
        d = yaml.safe_load(f)
    assert d, f'{name}.yaml empty or invalid'
    print(f'{name}.yaml: OK ({len(d)} top-level keys)')
\""
```
Expected: 4 lines, each `OK (N top-level keys)`.

- [ ] **Step 6: Commit.**

```bash
ssh xrak "cd /home/xrak/rak && git add src/rak_description/config/ && \
  git commit -m 'feat(rak_description): MoveIt config YAMLs (joint_limits, kinematics, ompl, fake_controllers)

Spec §7.5. KDL kinematics, RRTConnect planner, FollowJointTrajectory
action name; 5 active joints in fake controller. TODO: limits are
conservative placeholders; will be replaced with measured values.'"
```

---

## Task 7: view_urdf + display launch files (P4a, M2 milestone)

**Files:**
- Create: `src/rak_description/launch/view_urdf.launch.py`
- Create: `src/rak_description/launch/display.launch.py`
- Create: `src/rak_description/config/rviz_view.rviz` (minimal config for M2)

**Interfaces:**
- Consumes: `rak.urdf.xacro`, `rak.srdf`, the 4 YAMLs.
- Produces: 2 launch files that bring up `robot_state_publisher` + `joint_state_publisher` (+ RViz for display).

- [ ] **Step 1: Write `view_urdf.launch.py`.**

```python
"""view_urdf.launch.py — headless URDF + joint_state visualization.

Spec §7.6. No RViz; safe to run over plain SSH for M1/M2 validation.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("rak_description")
    urdf_path = os.path.join(pkg, "urdf", "rak.urdf.xacro")

    robot_description = Command(["xacro", urdf_path])

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    jsp = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": False}],
    )

    return LaunchDescription([rsp, jsp])
```

- [ ] **Step 2: Write `display.launch.py` (extends view_urdf with RViz).**

```python
"""display.launch.py — view_urdf + RViz (M2 visualization).

Spec §7.6. Requires Jetson local :1 display or X-forwarding.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg = get_package_share_directory("rak_description")
    urdf_path = os.path.join(pkg, "urdf", "rak.urdf.xacro")
    rviz_config = os.path.join(pkg, "config", "rviz_view.rviz")

    robot_description = Command(["xacro", urdf_path])

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    jsp = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[{"use_sim_time": False}],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
    )

    return LaunchDescription([rsp, jsp, rviz])
```

- [ ] **Step 3: Write `rviz_view.rviz` (minimal).**

Copy `boot/config/slam_rviz.rviz` as the starting template, then change:
- `Fixed Frame: world` (was `map`)
- Add `RobotModel` display with `Robot Description: robot_description`

(The exact YAML is long; adapt by editing a copy of `boot/config/slam_rviz.rviz` and adjusting the two fields above.)

- [ ] **Step 4: Build + smoke-launch view_urdf (no GUI, just verify it starts).**

```bash
ssh xrak "cd /home/xrak/rak && source /opt/ros/jazzy/setup.bash && \
  colcon build --packages-select rak_description"
```
Expected: build succeeds.

Then a 5s launch smoke-test in another ssh:
```bash
ssh xrak "cd /home/xrak/rak && timeout 5 ros2 launch rak_description view_urdf.launch.py 2>&1 | grep -E 'robot_state_publisher|joint_state_publisher|ERROR' | head -10"
```
Expected: both `robot_state_publisher` and `joint_state_publisher` start; no `ERROR` lines. (May show `Warning: ...` for missing transforms — that's normal during startup.)

- [ ] **Step 5: Verify TF tree resolves 7 segments (M2 validation, headless).**

```bash
ssh xrak "cd /home/xrak/rak && source /opt/ros/jazzy/setup.bash && source install/setup.bash && \
  setsid ros2 launch rak_description view_urdf.launch.py </dev/null >/tmp/view_urdf.log 2>&1 & \
  sleep 5 && \
  ros2 run tf2_tools view_frames -o /tmp/frames && \
  pkill -f view_urdf.launch.py"
```
Expected: `/tmp/frames.pdf` generated with 7 segments (world→base_link→arm_root_link→shoulder_link→upper_arm_link→forearm_link→wrist_link→gripper_link).

If `tf2_tools` not in apt: it's part of `ros-jazzy-tf2-tools`, which Task 1 installs.

- [ ] **Step 6: Commit.**

```bash
ssh xrak "cd /home/xrak/rak && git add src/rak_description/launch/ src/rak_description/config/rviz_view.rviz && \
  git commit -m 'feat(rak_description): view_urdf + display launch (M2 passes)

Spec §7.6. view_urdf is headless-friendly (M1/M2); display adds RViz
with RobotModel. tf2_tools view_frames produces 7-segment tree
world->...->gripper_link (verified).'"
```

---

## Task 8: demo.launch.py with move_group + fake_joint_driver (P4b, M3+M4 milestones)

**Files:**
- Create: `src/rak_description/launch/demo.launch.py`

**Interfaces:**
- Consumes: `display.launch.py` (Task 7), `rak.srdf` (Task 5), all 4 YAMLs (Task 6).
- Produces: a launch that brings up `move_group` + `fake_joint_driver` so the user can drag the InteractiveMarker in RViz (M3) and watch `/joint_states` change (M4).

- [ ] **Step 1: Write `demo.launch.py`.**

```python
"""demo.launch.py — full MoveIt2 demo with fake controller (M3 + M4).

Spec §7.6. Requires Jetson local :1 display (Xorg) for RViz + IK.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    pkg = get_package_share_directory("rak_description")
    urdf_path = os.path.join(pkg, "urdf", "rak.urdf.xacro")
    srdf_path = os.path.join(pkg, "urdf", "rak.srdf")

    # MoveItConfigsBuilder wraps the 4 YAMLs + SRDF into one config dict.
    moveit_config = (
        MoveItConfigsBuilder("rak", package_name="rak_description")
        .robot_description(file_path=urdf_path)
        .robot_description_semantic(file_path=srdf_path)
        .joint_limits(file_path=os.path.join(pkg, "config", "joint_limits.yaml"))
        .kinematics(file_path=os.path.join(pkg, "config", "kinematics.yaml"))
        .planning_pipelines(
            pipelines=["ompl"],
            default_planning_pipeline="ompl",
            load_all=False,
        )
        .moveit_controller_manager(
            file_path=os.path.join(pkg, "config", "fake_controllers.yaml")
        )
        .to_moveit_configs()
    )

    # move_group: planning server.
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    # RViz: InteractiveMarker for goal-pose dragging (M3).
    rviz_config = os.path.join(pkg, "config", "rviz_view.rviz")
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
    )

    # fake_joint_driver: serves FollowJointTrajectory, publishes /joint_states
    # (M4). Loaded via moveit_fake_controller_manager.
    fake_controller = Node(
        package="moveit_fake_controller_manager",
        executable="moveit_fake_controller_manager",
        name="moveit_fake_controller_manager",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"fake_controller_actuators": ["arm_with_base_controller"]},
        ],
    )

    # robot_state_publisher + joint_state_publisher (so move_group has TF).
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": Command(["xacro", urdf_path])}],
    )

    return LaunchDescription([rsp, move_group, fake_controller, rviz])
```

- [ ] **Step 2: Install `moveit_configs_utils` Python package on xrak.**

```bash
ssh xrak "sudo apt install -y ros-jazzy-moveit-common-tools python3-moveit-commander"
```

- [ ] **Step 3: Build + smoke-launch demo, check M3 evidence + controller action present (review focus #3).**

```bash
ssh xrak "cd /home/xrak/rak && source /opt/ros/jazzy/setup.bash && \
  colcon build --packages-select rak_description"
```
Expected: build succeeds.

Then in two ssh sessions:

Session A — launch demo:
```bash
ssh xrak "cd /home/xrak/rak && source /opt/ros/jazzy/setup.bash && source install/setup.bash && \
  setsid nohup ros2 launch rak_description demo.launch.py \
  </dev/null >/tmp/demo_run.log 2>&1 & disown"
```

Session B — wait 8 s, then check:
```bash
sleep 8
ssh xrak "source /opt/ros/jazzy/setup.bash && source /home/xrak/rak/install/setup.bash && \
  ros2 action list | grep -E 'follow_joint_trajectory|arm_with_base'"
```
Expected: action listed (e.g., `/arm_with_base_controller/follow_joint_trajectory`).

- [ ] **Step 4: Verify M4 — `/joint_states` topic is being published.**

```bash
ssh xrak "source /opt/ros/jazzy/setup.bash && source /home/xrak/rak/install/setup.bash && \
  ros2 topic hz /joint_states --window 5"
```
Expected: a non-zero Hz value (even if just `1.0` from the fake driver publishing at rest).

Then publish a goal pose via the C++ shim or moveit_commander (Python — fine for the validation, not a driver):
```python
# Run via ssh heredoc:
python3 <<'PYEOF'
import rclpy
from moveit_commander import MoveGroupCommander, RobotCommander, PlanningSceneInterface
rclpy.init()
mg = MoveGroupCommander("arm_with_base")
print("planning frame:", mg.get_planning_frame())
print("end effector:", mg.get_end_effector_link())
print("joints:", mg.get_joints())
mg.go([0.0, 0.5, 0.3, 0.0, 0.0], wait=True)
print("motion OK")
PYEOF
```
Expected: `planning frame:`, `end effector:`, `joints:`, `motion OK` all print. (Requires `python3-moveit-commander` installed in Step 2.)

- [ ] **Step 5: Stop the demo process.**

```bash
ssh xrak "pkill -f demo.launch.py; pkill -f moveit_fake_controller_manager; pkill -f move_group"
```

- [ ] **Step 6: Commit.**

```bash
ssh xrak "cd /home/xrak/rak && git add src/rak_description/launch/demo.launch.py && \
  git commit -m 'feat(rak_description): demo.launch.py with move_group + fake_controller (M3+M4 pass)

Spec §7.6. move_group reads rak.srdf + 4 YAMLs via MoveItConfigsBuilder.
fake_controller_manager serves FollowJointTrajectory on
arm_with_base_controller. MoveIt-commander smoke-test reaches the
gripper end-effector and moves all 5 joints, proving the chain.'"
```

---

## Task 9: `run_demo.sh` + meshes placeholder (P5)

**Files:**
- Create: `src/rak_description/scripts/run_demo.sh`
- Create: `src/rak_description/meshes/arm/README.md`

**Interfaces:**
- Consumes: pattern from `boot/scripts/run_nav2.sh` (verified exists).
- Produces: an operator script that brings up the full demo on Jetson local `:1` + a placeholder mesh dir.

- [ ] **Step 1: Write `meshes/arm/README.md`.**

```markdown
<!--
  Placeholder for STL mesh files. URDF currently uses simple box/cylinder
  geometry (verified correct kinematics + collision; visual quality is
  intentionally placeholder).
  TODO: replace with real STL meshes once mechanical design finalizes.
-->
# meshes/arm

Placeholder mesh directory. The URDF uses simple box and cylinder geometry
until mechanical design is finalized. Replacement workflow:

1. Drop `link1.stl`, `link2.stl`, `link3.stl`, `link4.stl` here.
2. Edit `src/rak_description/urdf/rak.urdf.xacro` to swap
   `<geometry><box .../></geometry>` for
   `<geometry><mesh filename="package://rak_description/meshes/arm/linkN.stl"/></geometry>`.
4. Rebuild + relaunch — TF and inertia are unchanged; only visuals improve.
```

- [ ] **Step 2: Write `run_demo.sh`.**

```bash
#!/bin/bash
# run_demo.sh — operator launcher for the MoveIt2 demo on Jetson local :1.
# Pattern: copy of src/boot/scripts/run_nav2.sh (Cyclone DDS + domain 45 +
# setsid + nohup). Memory K2: SSH -Y forwarding breaks; always use :1.

# NOTE: no `set -u` — trips on AMENT_TRACE_SETUP_FILES in /opt/ros/jazzy/setup.bash
set -e

export DISPLAY=:1
export XAUTHORITY=/run/user/1000/gdm/Xauthority
export QT_QPA_PLATFORM=xcb

# DDS unification (team leader mandate, 2026-10-02): Cyclone DDS, domain 45.
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=45
export CYCLONEDDS_URI=file:///home/xrak/rak/install/boot/share/boot/config/cyclonedds.xml

# Review focus item #4: verify Xauth cookie is readable on this shell.
test -S "$XAUTHORITY" || { echo "FAIL: \$XAUTHORITY=$XAUTHORITY not a socket"; exit 1; }

source /opt/ros/jazzy/setup.bash
source /home/xrak/rak/install/setup.bash

cd /home/xrak/rak

exec setsid nohup ros2 launch rak_description demo.launch.py \
    </dev/null >/tmp/demo_run.log 2>&1
```

```bash
ssh xrak "chmod +x /home/xrak/rak/src/rak_description/scripts/run_demo.sh"
```

- [ ] **Step 3: Smoke-test `run_demo.sh` end-to-end (review focus item #4).**

```bash
ssh xrak "/home/xrak/rak/src/rak_description/scripts/run_demo.sh"
sleep 8
ssh xrak "tail -20 /tmp/demo_run.log"
```
Expected: log shows `robot_state_publisher`, `move_group`, `moveit_fake_controller_manager`, `rviz2` all started without `ERROR` lines.

Then stop:
```bash
ssh xrak "pkill -f demo.launch.py; pkill -f moveit_fake_controller_manager; pkill -f move_group"
```

- [ ] **Step 4: Commit.**

```bash
ssh xrak "cd /home/xrak/rak && git add src/rak_description/scripts/run_demo.sh src/rak_description/meshes/ && \
  git commit -m 'feat(rak_description): run_demo.sh + meshes/arm placeholder (P5)

Spec §7.7. run_demo.sh follows boot/scripts/run_nav2.sh pattern exactly
(Cyclone DDS, domain 45, setsid, nohup, XAUTHORITY socket pre-check).
meshes/arm/ holds a README documenting the future STL-swap workflow.'"
```

---

## Self-Review Checklist

After implementation, the executor must verify:

1. **All 9 task commits land cleanly on `main`** with scoped messages.
2. **M1 passes** (`check_urdf /tmp/rak.urdf` shows zero errors).
3. **M2 passes** (`/tmp/frames.pdf` shows 7 segments; `tf2_tools view_frames` succeeds).
4. **M3 passes** (`ros2 action list | grep arm_with_base_controller` returns a result).
5. **M4 passes** (`moveit_commander` reaches `gripper_link` and moves 5 joints without error).
6. **`validate_moveit_installed.sh` passes** on xrak.
7. **`validate_macros.py` passes** (link_length consistency + inertia scaling).
8. **No `joint_state_publisher_gui`** is installed (spec §4.2).
9. **`boot/config/cyclonedds.xml`** exists at install path.
10. **Commit count = 9** in `git log --oneline main HEAD~9..HEAD` after Tasks 1–9.

## Execution Handoff

Plan complete and saved to `docs/superpowers/plans/2026-10-03-rak-moveit2-integration.md`. Please review.

For this plan I recommend **Native execution** (I implement every task myself in this session), because:
- Tasks are tightly sequenced — each task's content references the prior task's file paths
- Spec is committed; machine-readable artifacts
- 9 commits is the right review granularity — one review at the end is sufficient

If you'd rather have **Subagent-driven** (fresh subagent per task + per review), say so and I'll switch.

Either way: please confirm the plan captures what you want, and pick an execution approach.