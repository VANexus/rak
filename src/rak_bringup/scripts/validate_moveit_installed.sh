#!/usr/bin/env bash
# Check the installed planning resources without starting nodes.
set -e
source "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/environment.sh"
for package in moveit_ros_move_group moveit_configs_utils moveit_kinematics joint_state_publisher; do
  ros2 pkg prefix "$package"
done
test -f "$(ros2 pkg prefix --share rak_description)/urdf/rak.urdf.xacro"
test -f "$(ros2 pkg prefix --share rak_moveit_config)/config/rak.srdf"
test -f "$(ros2 pkg prefix --share rak_bringup)/config/cyclonedds.xml"
echo "Planning resources found; real arm execution is not validated."
