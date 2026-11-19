#!/bin/bash
# 真机视觉寻路急停：先杀**会发指令的**节点，再停底盘。
# 顺序很重要：留着 road_follow_node 就会继续发 vx，底盘停了它还在发。
#
# 顺序 = 指令源 -> 桥 -> 底盘：
#   1. road_follow_node / road_goal_node  停止产生 /cmd_vel
#   2. cmd_vel_omni                        停止转发到 reference
#   3. 底盘 / ros2_control / device_container
# 清理用括号模式（`[r]oad_...`），否则 pkill -f 会把调用本脚本的那一行也匹配掉。

for pat in '[r]oad_follow_node' '[r]oad_goal_node' '[c]md_vel_omni' \
           '[r]oadseg_node' '[u]sb_cam_node_exe'; do
  pkill -9 -f "$pat" 2>/dev/null
done
echo "vision chain stopped"

sudo -n bash -c \
  "pkill -9 -f 'mecanum_drive_controller|ros2_control_node|device_container|chassis_bringup'" \
  2>/dev/null
sleep 2
echo "chassis stopped. 检查: ps -eo args= | grep -c controller_manager"
