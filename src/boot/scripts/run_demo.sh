#!/bin/bash
# run_demo.sh - operator launcher for the MoveIt2 demo on Jetson local :1.
# Pattern: copy of src/boot/scripts/run_nav2.sh (Cyclone DDS + domain 45 +
# setsid + nohup). Memory K2: SSH -Y forwarding breaks; always use :1.

# NOTE: no `set -u` - trips on AMENT_TRACE_SETUP_FILES in /opt/ros/jazzy/setup.bash
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