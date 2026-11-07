#!/usr/bin/env bash
# One-shot setup for Ubuntu 24.04 (including WSL2): ROS 2 Jazzy + Gazebo Harmonic + ros_gz
# + this project's Python environment.   Usage:  bash setup_linux.sh
set -euo pipefail

if ! grep -q 'VERSION_ID="24.04"' /etc/os-release; then
  echo "This script targets Ubuntu 24.04 (needed for ROS 2 Jazzy). Aborting."; exit 1
fi
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "==> Base packages and UTF-8 locale"
sudo apt-get update
sudo apt-get install -y locales software-properties-common curl gnupg lsb-release unzip
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

echo "==> ROS 2 apt source"
sudo add-apt-repository -y universe
ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
if [ -z "${ROS_APT_SOURCE_VERSION}" ]; then
  echo "Could not look up the ros-apt-source version (GitHub rate limit?). Wait a few minutes and re-run."; exit 1
fi
CODENAME="$(. /etc/os-release && echo "${UBUNTU_CODENAME:-${VERSION_CODENAME}}")"
curl -L -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${CODENAME}_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

echo "==> Gazebo apt source"
sudo curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
  --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

echo "==> Installing ROS 2 Jazzy, Gazebo Harmonic, ros_gz bridge (this takes a while)"
sudo apt-get update
sudo apt-get install -y ros-jazzy-desktop ros-dev-tools gz-harmonic ros-jazzy-ros-gz \
  python3-tk python3-venv libportaudio2

echo "==> Optional: Gazebo Python bindings for the trail effect"
sudo apt-get install -y python3-gz-transport13 python3-gz-msgs10 || echo "(trail packages not available; the trail effect will be skipped)"

echo "==> Source ROS in every new terminal"
grep -qxF 'source /opt/ros/jazzy/setup.bash' ~/.bashrc || echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc

echo "==> Python environment for the voice app"
python3 -m venv --system-site-packages "$PROJECT_DIR/.venv"
"$PROJECT_DIR/.venv/bin/pip" install --upgrade pip
"$PROJECT_DIR/.venv/bin/pip" install faster-whisper sounddevice numpy

cat <<MSG

Done. Open a NEW terminal, then:
  gz sim -r empty.sdf                      # a Gazebo window should appear
  cd "$PROJECT_DIR" && source .venv/bin/activate && python3 app.py --no-ros
Then follow the four-terminal steps in README.md.
MSG
