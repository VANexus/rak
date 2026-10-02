#!/bin/bash
# /tmp/run_nav2.sh — launch the full SLAM + nav2 chain on Jetson local display.
# Same X11 + setsid pattern as run_slam.sh.

# NOTE: no `set -u` — trips on AMENT_TRACE_SETUP_FILES in /opt/ros/jazzy/setup.bash
set -e

export DISPLAY=:1
export XAUTHORITY=/run/user/1000/gdm/Xauthority
export QT_QPA_PLATFORM=xcb

# DDS unification (team leader mandate, 2026-10-02): Cyclone DDS, domain 45.
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=45
export CYCLONEDDS_URI=file:///home/xrak/rak/install/boot/share/boot/config/cyclonedds.xml

source /opt/ros/jazzy/setup.bash
source /home/xrak/rak/install/setup.bash

# ament_python packages (motor_stub) are skipped by colcon's setup.bash
# AMENT_PREFIX_PATH injection -- PYTHONPATH gets set, but ros2 pkg can't
# find them. Patch it manually so ros2 launch sees motor_stub.
# (colcon bug; remove this once the team moves to a colcon version that
#  prepends AMENT_PREFIX_PATH for ament_python packages too.)
export AMENT_PREFIX_PATH="/home/xrak/rak/install/motor_stub:${AMENT_PREFIX_PATH}"

cd /home/xrak/rak

exec setsid nohup ros2 launch boot nav2.launch.py \
    </dev/null >/tmp/nav2_run.log 2>&1
