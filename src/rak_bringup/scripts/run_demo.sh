#!/usr/bin/env bash
# Legacy arm planning demo, not a real hardware controller.
set -e
source "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/environment.sh"
rak_display
exec setsid nohup ros2 launch rak_moveit_config demo.launch.py "$@" \
    </dev/null >/tmp/demo_run.log 2>&1
