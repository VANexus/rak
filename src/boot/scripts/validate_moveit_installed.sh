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
  fail "joint_state_publisher_gui (Python) is installed - should NOT be per section 4.2"
fi
echo "OK: joint_state_publisher_gui (Python) correctly absent"

# 4. Cyclone DDS config exists at install path
[ -f /home/xrak/rak/install/boot/share/boot/config/cyclonedds.xml ] \
  || fail "cyclonedds.xml missing at install path"
echo "OK: cyclonedds.xml present at install path"

echo "ALL OK"