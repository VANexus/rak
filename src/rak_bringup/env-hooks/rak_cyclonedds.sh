# generated from rak_bringup/env-hooks/rak_cyclonedds.sh
#
# [AI] 2026-10-14 统一 CYCLONEDDS_URI 的唯一定义点（docs/perf/ipc_baseline.md §9 遗留项闭环）。
# 任何 source 工作区 overlay（install/architecture/local_setup.bash 或 environment.sh）
# 的 shell、以及从它派生的所有节点进程，都会拿到团队 CycloneDDS 配置
# （config/cyclonedds.xml: domain 47 + MaxMessageSize）。以前只有 environment.sh 显式
# export，直接 `source …local_setup.bash && ros2 launch …` 的操作入口会漏掉（2026-10-14
# 实测两个 chassis 实例均未带此变量），于是配置文件休眠、运行时跑 Cyclone 默认值。
#
# set-if-unset 语义：先显式 export（bench 的 TEAMCFG、capture 脚本等）仍然优先。
# AMENT_CURRENT_PREFIX 由本包 local_setup.sh 在逐个 source 钩子期间提供；
# 直接 source 本文件时退回 COLCON_CURRENT_PREFIX。文件不存在则不设（宁缺勿坏）。
_rak_cyclonedds_prefix="${AMENT_CURRENT_PREFIX:-${COLCON_CURRENT_PREFIX:-}}"
if [ -z "${CYCLONEDDS_URI:-}" ] && [ -n "${_rak_cyclonedds_prefix}" ] &&
  [ -f "${_rak_cyclonedds_prefix}/share/rak_bringup/config/cyclonedds.xml" ]; then
  export CYCLONEDDS_URI="file://${_rak_cyclonedds_prefix}/share/rak_bringup/config/cyclonedds.xml"
fi
unset _rak_cyclonedds_prefix
