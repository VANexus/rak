#!/usr/bin/env bash
# Source this file; ROS setup scripts are not compatible with nounset.

if [[ -z "${RAK_WORKSPACE:-}" ]]; then
  RAK_WORKSPACE="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"
  # [AI] 2026-10-10: ADR-0013 后 brain/ 已迁入 src/rak_agent，旧 marker 使本脚本
  # 探测失败 return 1（run_*.sh 一 source 即退出）。改用自身所在包作根 marker。
  while [[ "$RAK_WORKSPACE" != / && ! -f "$RAK_WORKSPACE/src/rak_bringup/package.xml" ]]; do
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
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-47}"
# CYCLONEDDS_URI 不在这里设：唯一定义点是上一行 source 的 overlay 里的
# rak_bringup 环境钩子 env-hooks/rak_cyclonedds.{sh,dsv}（set-if-unset，
# 显式 export 仍可覆盖）。以前这里的显式 export 是唯一接线，直接 source
# overlay 的操作入口会漏掉它（见 docs/perf/ipc_baseline.md §9）。

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
