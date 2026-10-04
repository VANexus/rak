#!/usr/bin/env bash
# Test navigation chain. No real hardware transport is started.
set -e
source "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/environment.sh"
exec setsid nohup ros2 launch rak_navigation nav2.launch.py "$@" \
    </dev/null >/tmp/nav2_run.log 2>&1
