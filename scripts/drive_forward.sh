#!/bin/bash
# Drive the base forward at a fixed speed for N seconds, then stop.
#   drive_forward.sh <speed_mps> <seconds>
source /opt/ros/jazzy/setup.bash
source /home/xrak/rak/install/local_setup.bash
export ROS_DOMAIN_ID=47
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
SPEED=${1:-0.12}
SECS=${2:-6}
echo "forward ${SPEED} m/s for ${SECS}s"
timeout "$SECS" ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: $SPEED}, angular: {z: 0.0}}" > /dev/null 2>&1
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}}" > /dev/null 2>&1
echo "stopped"
