#!/usr/bin/env bash
# Build only the offline ROS/MoveIt subset, keeping outputs on WSL's ext4 disk.
set -eo pipefail
[[ $(uname -r) == *WSL2* && $(uname -m) == x86_64 ]] || {
  echo 'This entry point is for the x86 WSL development environment' >&2; exit 1;
}
repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
build_root="${RAK_GRASP_BUILD_ROOT:-$HOME/rak-grasp-workspace}"
mkdir -p "$build_root"
cd "$build_root"
source /opt/ros/jazzy/setup.bash
packages=(rak_interfaces rak_description rak_arm_simulation rak_grasp_perception rak_moveit_config)
paths=()
for package in "${packages[@]}"; do paths+=("$repo/src/$package"); done
colcon build --paths "${paths[@]}" --executor sequential \
  --build-base build --install-base install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/local_setup.bash
ROS_DOMAIN_ID=77 PYTHONPATH="$repo/src/rak_grasp_perception:$PYTHONPATH" \
  python3 -m pytest "$repo/src/rak_grasp_perception/test" -q
colcon test --paths "${paths[@]}" --build-base build --install-base install \
  --packages-select rak_description rak_moveit_config
colcon test-result --test-result-base build --verbose
PYTHONPATH="$repo/brain" python3 "$repo/brain/tests/test_ai.py"
PYTHONPATH="$repo/brain" python3 "$repo/brain/tests/test_task_intent.py"
printf '\nSource %s/install/local_setup.bash before running RViz or simulation.\n' "$build_root"
