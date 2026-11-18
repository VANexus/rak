#!/usr/bin/env bash
# Start the front mono camera with the already-calibrated intrinsics.
#
# Why this wrapper exists:
#   - camera.launch.py defaults camera_info_url to empty, and its docstring
#     mentions ~/rak/data/calib/front_camera.yaml, which does not exist;
#   - the real calibration is ~/.ros/camera_info/front_camera.yaml;
#   - with the Gemini 335L connected, UVC enumeration can move the front camera
#     from /dev/video0 to /dev/video8. Install 99-front-camera.rules for a stable
#     /dev/front_camera, or pass the device as the first argument.
#
# Usage:
#   ./front_camera.sh                 # prefer /dev/front_camera, fall back to /dev/video8
#   ./front_camera.sh /dev/video8     # explicit device
set -euo pipefail

info="${HOME}/.ros/camera_info/front_camera.yaml"
dev="${1:-/dev/front_camera}"

if [[ ! -f "${info}" ]]; then
  echo "front camera calibration missing: ${info}" >&2
  exit 1
fi

if [[ ! -e "${dev}" ]]; then
  if [[ -e /dev/front_camera ]]; then
    dev=/dev/front_camera
  elif [[ -e /dev/video8 ]]; then
    dev=/dev/video8
  else
    echo "front camera device not found; install 99-front-camera.rules or pass a device" >&2
    exit 1
  fi
fi

echo "front camera: ${dev}, camera_info: ${info}"
exec ros2 launch rak_bringup camera.launch.py \
  video_device:="${dev}" \
  camera_info_url:="file://${info}"
