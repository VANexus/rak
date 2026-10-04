#!/usr/bin/env bash
# RViz for the navigation stack.
set -e
source "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/environment.sh"
rak_display
exec setsid nohup rviz2 -d "$(ros2 pkg prefix --share rak_navigation)/config/slam_rviz.rviz" "$@" \
    </dev/null >/tmp/rviz_slam_run.log 2>&1
