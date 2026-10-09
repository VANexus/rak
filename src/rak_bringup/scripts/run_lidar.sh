#!/usr/bin/env bash
# Lidar driver and scan filter only.
set -e
source "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/environment.sh"
exec setsid nohup ros2 launch rak_bringup lidar.launch.py "$@" \
    </dev/null >/tmp/lidar_only_run.log 2>&1
