#!/bin/bash
# 纯视觉寻路 —— 单机（相机 + 分割 + 控制全在板上）的一键入口。
#
# 这里只是 `rak_bringup/road_follow.launch.py role:=both` 的薄封装：所有节点
# 接线、参数、话题都在 launch 里，**不要再往本脚本加 ros2 run**。以前这里各自
# 手起 usb_cam / roadseg / road_follow，和 launch 文件重复维护了一遍，两边一
# 改就漂（2026-11-19 统一）。
#
# 真机让底盘动：另开 chassis_bringup.launch.py allow_motion:=true，并给本脚本
# 加 allow_motion:=true（两个门都要开）。单写者纪律（ADR-0011）：本链运行时
# 不得同起 Nav2 / rak_wbc / chassis_tui。想走 Nav2 执行层用 controller:=nav2。
#
#   ./start_road_follow.sh                        # 只解算，不动底盘（默认）
#   ./start_road_follow.sh allow_motion:=true     # 写执行器
#   ./start_road_follow.sh controller:=nav2       # RotationShim + MPPI（需 TF）
cd ~/rak || exit 1
source /opt/ros/jazzy/setup.bash
source install/architecture/local_setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-47}"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

pkill -9 -f '[u]sb_cam_node_exe' 2>/dev/null
pkill -9 -f '[r]oadseg_node' 2>/dev/null
pkill -9 -f '[r]oad_follow_node' 2>/dev/null
pkill -9 -f '[r]oad_goal_node' 2>/dev/null
sleep 2

exec ros2 launch rak_bringup road_follow.launch.py \
  role:=both enabled:=true rviz:=false "$@"
