#!/bin/bash
# Bring up the real chassis with motion enabled.
#   run_chassis.sh [slaves] [urdf]   slaves = both | chassis | arm
# 缺机械臂时用 slaves:=chassis，否则 arm 的 Boot Timeout 会让 CANopen 容器 abort。
# urdf 省略时用默认 rak_chassis.urdf.xacro；排障可用 /tmp/chassis_only.urdf
# 只加载底盘硬件接口，避免 arm 读失败把 joint_state_broadcaster 拖停。
source /opt/ros/jazzy/setup.bash
source /home/xrak/rak/install/local_setup.bash
export ROS_DOMAIN_ID=47
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
SLAVES=${1:-both}
URDF=${2:-}
if [ -n "$URDF" ]; then
  exec ros2 launch rak_bringup chassis_bringup.launch.py \
    allow_motion:=true slaves:="$SLAVES" arm_motion_writer:=none urdf:="$URDF"
fi
exec ros2 launch rak_bringup chassis_bringup.launch.py \
  allow_motion:=true slaves:="$SLAVES" arm_motion_writer:=none
