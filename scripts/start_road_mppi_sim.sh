#!/bin/bash
# 纯视觉寻路 MPPI 桌面验证链（真相机 + 真分割，底盘是假体）。
#
# 这里只是 `rak_bringup/road_follow.launch.py controller:=nav2` 的薄封装，外加
# 两个**测试夹具**（假里程计 + 假底盘反馈）——都是 rak_motion_stub 的正式入口，
# 不再是裸脚本。所有节点接线在 launch 里，**不要再往本脚本加 ros2 run**。
#
#   camera -> roadseg -> road_goal(中心线->地面路径) -> nav2 FollowPath
#          -> RotationShim + MPPI(Omni) -> /cmd_vel -> cmd_vel_sim -> /base/telemetry
#          -> wheel_odom(假里程计+TF) -> MPPI 位姿反馈
#
# 单写者纪律(ADR-0011): 本链运行时 road_follow_node / chassis_tui / wbc 全关。
# 真机切换: 去掉这两个夹具，改用 chassis_bringup allow_motion:=true（轮式里程计+
# EKF 提供 odom->base_footprint），再传 odom_source:=ekf。
cd ~/rak || exit 1
source /opt/ros/jazzy/setup.bash
source install/architecture/local_setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-47}"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

pkill -9 -f '[u]sb_cam_node_exe' 2>/dev/null
pkill -9 -f '[r]oadseg_node' 2>/dev/null
pkill -9 -f '[r]oad_follow_node' 2>/dev/null
pkill -9 -f '[r]oad_goal_node' 2>/dev/null
pkill -9 -f '[w]heel_odom' 2>/dev/null
pkill -9 -f '[c]md_vel_sim' 2>/dev/null
pkill -9 -f '[l]ifecycle_manager' 2>/dev/null
sleep 2
rm -f /dev/shm/fastrtps_* 2>/dev/null

# 假里程计先起：Nav2 在 configure 期就要 odom->base_link TF，晚了 lifecycle 直接 abort。
nohup setsid ros2 run rak_motion_stub wheel_odom --ros-args \
  -p base_frame:=base_link -p odom_topic:=/odom \
  > /tmp/wheel_odom.log 2>&1 < /dev/null &
sleep 2

# 假底盘反馈：相机档没有 collision_monitor，速度链末端是 /cmd_vel_smoothed。
nohup setsid ros2 run rak_motion_stub cmd_vel_sim --ros-args \
  -p cmd_vel_topic:=/cmd_vel_smoothed \
  > /tmp/cmd_vel_sim.log 2>&1 < /dev/null &
sleep 2

exec ros2 launch rak_bringup road_follow.launch.py \
  role:=both controller:=nav2 enabled:=true rviz:=false \
  odom_source:=none motion_backend:=none "$@"
