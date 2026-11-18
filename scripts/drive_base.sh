#!/bin/bash
# Drive the real base forward through the mecanum controller, then stop.
#   drive_base.sh <speed_mps> <seconds>
source /opt/ros/jazzy/setup.bash
source /home/xrak/rak/install/local_setup.bash
export ROS_DOMAIN_ID=47
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
SPEED=${1:-0.12}
SECS=${2:-8}
TOPIC=/mecanum_drive_controller/reference
echo "base forward ${SPEED} m/s for ${SECS}s"
timeout "$SECS" ros2 topic pub -r 20 "$TOPIC" geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: base_link}, twist: {linear: {x: $SPEED}}}" > /dev/null 2>&1
ros2 topic pub --once "$TOPIC" geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: base_link}, twist: {linear: {x: 0.0}}}" > /dev/null 2>&1
echo "stopped"
