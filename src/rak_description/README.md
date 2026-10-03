<!--
  rak_description - MoveIt2 integration for the Rak arm.
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

- `urdf/` - robot description (xacro + SRDF + macros)
- `config/` - MoveIt2 YAML configs (joint limits, kinematics, planning, controllers, RViz view)
- `launch/` - `view_urdf.launch.py`, `display.launch.py`, `demo.launch.py`
- `scripts/` - `validate_moveit_installed.sh`, `run_demo.sh`
- `meshes/` - STL placeholders (boxes until mechanical design finalizes)

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

See spec section 8 for details.