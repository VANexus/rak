#!/usr/bin/env bash
set -euo pipefail
# Local WSL development only. Run explicitly as root; never run on Orin.
[[ $(id -u) == 0 && $(uname -r) == *WSL2* && $(uname -m) == x86_64 ]] || {
  echo 'Requires root in x86_64 WSL2 Ubuntu 24.04' >&2; exit 1;
}
source /etc/os-release
[[ $VERSION_ID == 24.04 ]] || exit 1
export DEBIAN_FRONTEND=noninteractive
curl() { command curl -4 --retry-all-errors --connect-timeout 20 --max-time 180 "$@"; }
exec > >(tee /mnt/d/car/tools/graspnet/wsl-bootstrap.log) 2>&1
# IPv6 DNS lookups stalled on this host's WSL/VPN path. Applies to this distro only.
cat > /etc/apt/apt.conf.d/80rak-network <<'EOF'
Acquire::ForceIPv4 "true";
Acquire::Retries "3";
Acquire::http::Timeout "30";
Acquire::https::Timeout "30";
EOF
apt-get -o Acquire::Retries=3 update
apt-get install -y ca-certificates curl gnupg locales git build-essential python3-pip
locale-gen en_US.UTF-8
update-locale LANG=en_US.UTF-8
install -m 0755 -d /etc/apt/keyrings
curl --retry 3 -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
cat > /etc/apt/sources.list.d/docker.sources <<'EOF'
Types: deb
URIs: https://download.docker.com/linux/ubuntu
Suites: noble
Components: stable
Architectures: amd64
Signed-By: /etc/apt/keyrings/docker.asc
EOF
curl --retry 3 -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | gpg --dearmor --yes -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl --retry 3 -fsSL https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' > /etc/apt/sources.list.d/nvidia-container-toolkit.list
ros_source_version=$(curl --retry 3 -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | python3 -c 'import json,sys; print(json.load(sys.stdin)["tag_name"])')
curl --retry 3 -fL "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ros_source_version}/ros2-apt-source_${ros_source_version}.noble_all.deb" -o /tmp/ros2-apt-source.deb
dpkg -i /tmp/ros2-apt-source.deb
# Optional signed-index mirror; the official ROS signing key is retained.
if [[ -n ${ROS_APT_MIRROR:-} ]]; then
  [[ $ROS_APT_MIRROR == https://* ]] || { echo 'ROS_APT_MIRROR requires HTTPS' >&2; exit 1; }
  python3 - "$ROS_APT_MIRROR" <<'PY'
from pathlib import Path
import sys
source = Path('/usr/share/ros-apt-source/ros2.sources').read_text()
source = source.replace('Types: deb deb-src', 'Types: deb').replace(
    'http://packages.ros.org/ros2/ubuntu', sys.argv[1].rstrip('/'))
Path('/etc/apt/sources.list.d/rak-ros2-mirror.sources').write_text(source)
p = Path('/etc/apt/sources.list.d/ros2.sources')
if p.exists():
    p.rename(p.with_suffix('.sources.disabled'))
PY
fi
apt-get -o Acquire::Retries=3 update
apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin nvidia-container-toolkit
nvidia-ctk runtime configure --runtime=docker
systemctl restart docker
docker info --format '{{.DockerRootDir}}'
apt-get install -y ros-jazzy-desktop ros-dev-tools ros-jazzy-moveit ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-interactive-markers ros-jazzy-vision-msgs python3-pytest python3-scipy python3-opencv python3-defusedxml python3-pydantic python3-pil xvfb fonts-noto-cjk
printf '\nBOOTSTRAP COMPLETE\n'
