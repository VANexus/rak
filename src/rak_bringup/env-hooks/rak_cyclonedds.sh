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
#
# [AI] 2026-11-18: 网卡名是**机器相关**的（开发机 wlp0s20f3 / orin wlP1p1s0），而写死的
# NetworkInterface 在本机不存在时，Cyclone 会直接拒绝创建 participant —— 表现是
# **每一个** ROS 进程都 `rmw_create_node: failed to create domain` 然后 abort（本轮在
# orin 上实测：chassis_bringup 五个进程全灭，看起来像硬件故障）。
# 修法在构建期，不在这里：CMakeLists 按本机检测到的局域网口替换 NetworkInterface 后
# 再装进 share（见 cmake/dds_interface.cmake），所以本钩子只管指过去。
_rak_cyclonedds_prefix="${AMENT_CURRENT_PREFIX:-${COLCON_CURRENT_PREFIX:-}}"
if [ -z "${CYCLONEDDS_URI:-}" ] && [ -n "${_rak_cyclonedds_prefix}" ] &&
  [ -f "${_rak_cyclonedds_prefix}/share/rak_bringup/config/cyclonedds.xml" ]; then
  export CYCLONEDDS_URI="file://${_rak_cyclonedds_prefix}/share/rak_bringup/config/cyclonedds.xml"
fi
unset _rak_cyclonedds_prefix

# [AI] 2026-11-16: RMW 与 domain 也在这里兜底（与 rak_cyclonedds.dsv.in 的 set-if-unset 对齐）。
# 背景：orin 的 ~/.bashrc 只 source /opt/ros/jazzy，没有这两个变量；手工起节点时
# 漏掉 export 就会落到 FastRTPS + domain 0，和开发机（Cyclone + domain 47）完全
# 不可见——表现是 RViz 里"最后一帧留在屏幕上不动"，而不是报错。
# set-if-unset：显式指定的值（如 bench 想换 RMW）仍然优先。
if [ -z "${RMW_IMPLEMENTATION:-}" ]; then
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
fi
if [ -z "${ROS_DOMAIN_ID:-}" ]; then
  export ROS_DOMAIN_ID=47
fi
