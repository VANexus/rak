#!/usr/bin/env bash
# Source this file; ROS setup scripts are not compatible with nounset.

if [[ -z "${RAK_WORKSPACE:-}" ]]; then
  RAK_WORKSPACE="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"
  while [[ "$RAK_WORKSPACE" != / && ! -f "$RAK_WORKSPACE/brain/pyproject.toml" ]]; do
    RAK_WORKSPACE="$(dirname "$RAK_WORKSPACE")"
  done
  if [[ "$RAK_WORKSPACE" == / ]]; then
    echo "Set RAK_WORKSPACE to the repository root." >&2
    return 1
  fi
fi

export RAK_WORKSPACE
export RAK_INSTALL_PREFIX="${RAK_INSTALL_PREFIX:-$RAK_WORKSPACE/install/architecture}"
if [[ ! -f "$RAK_INSTALL_PREFIX/setup.bash" ]]; then
  echo "Missing $RAK_INSTALL_PREFIX/setup.bash; build the workspace first." >&2
  return 1
fi

source /opt/ros/jazzy/setup.bash
# Only add this workspace, not an old overlay captured during its build.
source "$RAK_INSTALL_PREFIX/local_setup.bash"

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-45}"
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file://$(ros2 pkg prefix --share rak_bringup)/config/cyclonedds.xml}"

# Non-GUI chains work headless; only visualization launchers call this.
rak_display() {
  export DISPLAY="${DISPLAY:-:1}"
  export XAUTHORITY="${XAUTHORITY:-/run/user/$(id -u)/gdm/Xauthority}"
  export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
  if [[ ! -r "$XAUTHORITY" ]]; then
    echo "Cannot read Xauthority: $XAUTHORITY" >&2
    return 1
  fi
}
