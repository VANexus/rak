#!/bin/bash
# /tmp/run_lidar.sh — launch only the lidar driver + filter. No SLAM, no nav2.
# Same DDS + display pattern as run_slam.sh / run_nav2.sh.
#
# Topics produced:
#   /scan_raw : 108-byte N10_P packets at 10 Hz (raw vendor message)
#   /scan     : 525-bin re-binned LaserScan at 10 Hz (after layer-A drop + binning)
#
# Use case: while the team leader needs the lidar live without the full
# SLAM/nav2 stack weighing the system. The leader works on /scan directly.
#
# Stop:  pkill -9 -f lslidar_driver_node  pkill -9 -f scan_to_scan_filter_chain

# NOTE: no `set -u` -- trips on AMENT_TRACE_SETUP_FILES in /opt/ros/jazzy/setup.bash
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

cd /home/xrak/rak

exec setsid nohup ros2 launch boot lidar.launch.py \
    </dev/null >/tmp/lidar_only_run.log 2>&1
