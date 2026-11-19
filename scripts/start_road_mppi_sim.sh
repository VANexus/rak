#!/bin/bash
# 纯视觉寻路 MPPI 桌面验证链：真相机 + 真分割，底盘用 cmd_vel_sim 假体闭环。
# 链路: camera -> roadseg -> road_goal(中心线->地面路径) -> nav2 FollowPath
#       -> MPPI(Omni) -> /cmd_vel -> cmd_vel_sim -> /base/telemetry
#       -> wheel_odom(假里程计+TF) -> MPPI 位姿反馈
# 单写者纪律(ADR-0011): 本链运行时 road_follow_node / chassis_tui / wbc 全关。
# 真机切换: cmd_vel_sim/wheel_odom 换成 chassis_bringup(allow_motion:=true, 轮式里程计)。
cd ~/rak || exit 1
source /opt/ros/jazzy/setup.bash
source install/architecture/local_setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-47}"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

pkill -9 -f 'usb_cam' 2>/dev/null
pkill -9 -f 'roadseg_node' 2>/dev/null
pkill -9 -f 'road_follow_node' 2>/dev/null
pkill -9 -f 'road_goal_node' 2>/dev/null
pkill -9 -f 'wheel_odom' 2>/dev/null
pkill -9 -f 'nav2' 2>/dev/null
pkill -9 -f 'cmd_vel_sim' 2>/dev/null
pkill -9 -f 'lifecycle_manager' 2>/dev/null
sleep 2
rm -f /dev/shm/fastrtps_* 2>/dev/null

ARC=""
for d in /dev/video*; do
  n=$(cat "/sys/class/video4linux/$(basename "$d")/name" 2>/dev/null)
  if [[ "$n" == *"AX USB2.0"* ]] && v4l2-ctl -d "$d" --list-formats 2>/dev/null | grep -q MJPG; then
    ARC=$d
    break
  fi
done
[ -z "$ARC" ] && ARC=/dev/video0
echo "front camera: $ARC"

nohup setsid ros2 run usb_cam usb_cam_node_exe --ros-args \
  -r __ns:=/camera -r __node:=camera \
  -p video_device:="$ARC" -p pixel_format:=mjpeg2rgb \
  -p image_width:=640 -p image_height:=480 -p framerate:=30.0 \
  -p camera_name:=front_camera -p frame_id:=camera_optical_frame \
  > /tmp/front_cam.log 2>&1 < /dev/null &
sleep 2

nohup setsid ros2 run rak_road_perception roadseg_node --ros-args \
  -p input_mode:=ros_image -p image_topic:=/camera/image_raw \
  > /tmp/roadseg.log 2>&1 < /dev/null &
sleep 3

# 假里程计（先起，Nav2 配置期就要 odom->base_link TF，晚了 lifecycle 直接 abort）
nohup setsid python3 "$HOME/rak/scripts/wheel_odom.py" --ros-args \
  -p base_frame:=base_link -p odom_topic:=/odom \
  > /tmp/wheel_odom.log 2>&1 < /dev/null &
sleep 2

# Nav2 相机档（无雷达无地图）。motion_backend:=none：相机档没有 collision_monitor，
# 速度链末端是 /cmd_vel_smoothed，所以假底盘自己订阅它（launch 里的 sim 只读 /cmd_vel）。
nohup setsid ros2 launch rak_bringup nav2.launch.py sensors:=camera \
  motion_backend:=none publish_robot_tf:=false odom_source:=none \
  > /tmp/nav_vision.log 2>&1 < /dev/null &
sleep 4
nohup setsid ros2 run rak_motion_stub cmd_vel_sim --ros-args \
  -p cmd_vel_topic:=/cmd_vel_smoothed \
  > /tmp/cmd_vel_sim.log 2>&1 < /dev/null &
sleep 6

# 视觉路径 -> FollowPath（MPPI 执行）
nohup setsid ros2 run rak_road_perception road_goal_node --ros-args \
  -p enabled:=true \
  > /tmp/road_goal.log 2>&1 < /dev/null &

echo "started"
echo "看: /tmp/nav_vision.log /tmp/road_goal.log /tmp/roadseg.log /tmp/wheel_odom.log"
echo "路径: ros2 topic echo /vision_path --once"
echo "速度: ros2 topic echo /cmd_vel"
