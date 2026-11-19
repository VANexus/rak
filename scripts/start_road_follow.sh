#!/bin/bash
# 纯视觉寻路最小链：前置相机 -> roadseg(TRT) -> road_follow -> /cmd_vel
# 只做这一条链，不起雷达/SLAM/Nav2/EKF（用户 2026-11-18 口径）。
# 真机让底盘动：另开 chassis_bringup.launch.py 并 allow_motion:=true，
# /cmd_vel 需按其控制器重映射（mecanum_drive_controller/cmd_vel_unstamped）。
# 单写者纪律（ADR-0011）：本链运行时不得同起 Nav2 / rak_wbc / chassis_tui。
cd ~/rak || exit 1
source /opt/ros/jazzy/setup.bash
source install/architecture/local_setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-47}"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

pkill -9 -f 'usb_cam' 2>/dev/null
pkill -9 -f 'roadseg_node' 2>/dev/null
pkill -9 -f 'road_follow_node' 2>/dev/null
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

# usb_cam 1280x720 只能 ~2 FPS（历史实测），640x480 MJPG 可用；roadseg 内部自己缩到 672x384。
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

nohup setsid ros2 run rak_road_perception road_follow_node --ros-args \
  -p enabled:=true -p max_vx:=0.12 \
  > /tmp/road_follow.log 2>&1 < /dev/null &

echo "started: front_cam /tmp/front_cam.log  roadseg /tmp/roadseg.log  follow /tmp/road_follow.log"
echo "看输出: ros2 topic echo /cmd_vel"
