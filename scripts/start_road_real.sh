#!/bin/bash
# 真机视觉寻路一键链：底盘(CANopen) + 前视相机 + 路面分割 + 跟随 + /cmd_vel 桥。
#
# 视觉侧只是 `rak_bringup/road_follow.launch.py role:=both allow_motion:=true` 的
# 薄封装；`/cmd_vel -> mecanum_drive_controller/reference` 的唯一桥
# `rak_motion_stub/cmd_vel_omni` 由该 launch 在 allow_motion:=true 时自动拉起
# （它带着实测的转向取反，别在别处再写一份）。
#
# 急停：Ctrl-C 本脚本**不会**停底盘 —— 用 scripts/stop_road_real.sh。
#
#   ./start_road_real.sh                 # direct 控制（默认）
#   ./start_road_real.sh controller:=nav2 # RotationShim + MPPI（需 EKF 的 TF）
ROS_SETUP=/opt/ros/jazzy/setup.bash
WS=/home/xrak/rak
INSTALL=$WS/install/architecture/local_setup.bash
DOMAIN=${ROS_DOMAIN_ID:-47}
MAX_VX=${MAX_VX:-0.08}

echo "[1/2] chassis (allow_motion:=true, slaves:=chassis)"
sudo -n bash -c "source $ROS_SETUP; source $INSTALL; \
  export ROS_DOMAIN_ID=$DOMAIN RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; \
  setsid nohup ros2 launch rak_bringup chassis_bringup.launch.py \
    allow_motion:=true slaves:=chassis arm_motion_writer:=none \
    > /tmp/chassis_real.log 2>&1 < /dev/null &"

sleep 32
grep -q "Driver booted and ready" /tmp/chassis_real.log || {
  echo "FAIL: device_container 没启动（看 /tmp/chassis_real.log）"; exit 1; }
grep -q "Configured and activated mecanum_drive_controller" /tmp/chassis_real.log || {
  echo "FAIL: mecanum_drive_controller 没激活"; exit 1; }

echo "[2/2] vision chain (camera + roadseg + follow + cmd_vel_omni), max_vx=$MAX_VX"
source "$ROS_SETUP"
source "$INSTALL"
export ROS_DOMAIN_ID=$DOMAIN RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 清理只按可执行名匹配，避免 pkill -f 把本脚本自己也匹配掉。
for pat in '[u]sb_cam_node_exe' '[r]oadseg_node' '[r]oad_follow_node' '[r]oad_goal_node' '[c]md_vel_omni'; do
  pkill -9 -f "$pat" 2>/dev/null
done
sleep 2

exec ros2 launch rak_bringup road_follow.launch.py \
  role:=both enabled:=true allow_motion:=true rviz:=false \
  max_vx:="$MAX_VX" "$@"
