#!/usr/bin/env bash
# One-command remote RViz on the workstation against topics published on the
# Jetson (ssh orin). See docs/perf/cross_host_dds.md.
#
#   bash src/rak_bringup/scripts/run_remote_rviz.sh [foo.rviz]
#
# Runs the LOCAL rviz2 (RViz is just a subscriber; the board stays headless).
# DDS env must match the board: rmw_cyclonedds_cpp + same ROS_DOMAIN_ID +
# CYCLONEDDS_URI. On the board these come from rak_bringup's env-hook; a
# workstation has no overlay, so we set them here explicitly.
#
# Overridable: RMW_IMPLEMENTATION, ROS_DOMAIN_ID (board default 0),
# CYCLONEDDS_URI (default: repo's team config file if present, else unset).

set -euo pipefail

RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:=rmw_cyclonedds_cpp}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:=0}"

this_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "$this_dir/../../.." && pwd)"
CFG="$repo_root/src/rak_bringup/config/cyclonedds.xml"
if [ -z "${CYCLONEDDS_URI:-}" ] && [ -f "$CFG" ]; then
  CYCLONEDDS_URI="file://$CFG"
fi

# Require a local ROS with rviz2. rcfile is distro-agnostic; lyrical here.
. /opt/ros/*/setup.bash 2>/dev/null || { echo "no local ROS setup.bash" >&2; exit 1; }
command -v rviz2 >/dev/null || { echo "rviz2 not installed locally" >&2; exit 1; }

rviz="${1:-$repo_root/src/rak_bringup/config/sensors_rviz.rviz}"
exec rviz2 -d "$rviz"