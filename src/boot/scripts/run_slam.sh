#!/bin/bash
# /tmp/run_slam.sh — launch full SLAM chain on Jetson local display.
#
# Pattern locked by K2 (xrak-rak-project memory):
#   - source ROS env twice (base + workspace)
#   - DISPLAY=:1, XAUTHORITY=/run/user/1000/gdm/Xauthority (touchscreen)
#   - QT_QPA_PLATFORM=xcb (no Wayland)
#   - setsid + nohup + disown + </dev/null (true detach)
#
# Usage: bash /tmp/run_slam.sh
# Stop :  pkill -9 -f 'ros2 launch boot slam'   (or kill the listed PIDs)

# NOTE: no `set -u` — it trips on AMENT_TRACE_SETUP_FILES in /opt/ros/jazzy/setup.bash
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

exec setsid nohup ros2 launch boot slam.launch.py \
    </dev/null >/tmp/slam_run.log 2>&1
