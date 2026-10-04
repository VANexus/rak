#!/usr/bin/env bash
# Lidar, laser odometry and SLAM, without a desktop dependency.
set -e
source "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/environment.sh"
exec setsid nohup ros2 launch rak_navigation slam.launch.py "$@" \
    </dev/null >/tmp/slam_run.log 2>&1
