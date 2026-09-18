#!/usr/bin/env bash
# NIDAR AirMouse dev environment for Ubuntu 24.04 (WSL2):
#   ROS 2 Jazzy + Gazebo Harmonic + slam_toolbox + MAVROS + ArduPilot SITL + ardupilot_gazebo
#
# Run as your normal user (NOT root):   bash /mnt/c/Users/mvn/Documents/dinar/setup/wsl_setup.sh
# It asks for your password once, then runs unattended (~30-60 min). Log: ~/airmouse_setup.log
#
# Network note: this network blocks raw.githubusercontent.com and https://packages.ros.org,
# so ROS packages come from the official UMD mirror and rosdistro files from the jsDelivr CDN.
set -euo pipefail
exec > >(tee -a "$HOME/airmouse_setup.log") 2>&1

if [ "$(id -u)" -eq 0 ]; then echo "Run this as your normal user, not root."; exit 1; fi

ROS_MIRROR=https://mirror.umd.edu/packages.ros.org/ros2/ubuntu
ROSDISTRO_CDN=https://cdn.jsdelivr.net/gh/ros/rosdistro@master
ROS_KEY_FPR=C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
APT="sudo DEBIAN_FRONTEND=noninteractive apt-get install -y"

sudo -v
# keep sudo alive for the whole run
while true; do sudo -n true; sleep 50; kill -0 "$$" 2>/dev/null || exit; done &

echo "=== [1/7] Base packages"
sudo apt-get update
sudo DEBIAN_FRONTEND=noninteractive apt-get upgrade -y
$APT locales curl gnupg lsb-release git build-essential cmake python3-pip python3-venv \
     software-properties-common wget unzip
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
sudo add-apt-repository -y universe

echo "=== [2/7] ROS 2 + Gazebo apt sources"
curl -fsSL "$ROSDISTRO_CDN/ros.key" -o /tmp/ros.key
gpg --show-keys --with-colons /tmp/ros.key | grep -q "$ROS_KEY_FPR" \
  || { echo "ROS key fingerprint mismatch - aborting"; exit 1; }
sudo gpg --dearmor --yes -o /usr/share/keyrings/ros-archive-keyring.gpg /tmp/ros.key
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] $ROS_MIRROR noble main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list

sudo curl -fsSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable noble main" \
  | sudo tee /etc/apt/sources.list.d/gazebo-stable.list
sudo apt-get update

echo "=== [3/7] ROS 2 Jazzy, Gazebo Harmonic, SLAM, MAVROS"
$APT ros-jazzy-desktop ros-dev-tools python3-rosdep python3-colcon-common-extensions \
     gz-harmonic ros-jazzy-ros-gz \
     ros-jazzy-slam-toolbox ros-jazzy-mavros ros-jazzy-mavros-extras \
     ros-jazzy-foxglove-bridge ros-jazzy-rosbridge-server ros-jazzy-tf-transformations \
     rapidjson-dev libopencv-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
     gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl
# MAVROS needs GeographicLib datasets even indoors (it crashes without them)
sudo /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh || echo "WARN: geographiclib datasets failed (retry later)"

echo "=== [4/7] rosdep (pointed at jsDelivr mirror of rosdistro)"
sudo mkdir -p /etc/ros/rosdep/sources.list.d
sudo tee /etc/ros/rosdep/sources.list.d/20-default.list >/dev/null <<EOF
yaml $ROSDISTRO_CDN/rosdep/base.yaml
yaml $ROSDISTRO_CDN/rosdep/python.yaml
yaml $ROSDISTRO_CDN/rosdep/ruby.yaml
EOF
export ROSDISTRO_INDEX_URL=$ROSDISTRO_CDN/index-v4.yaml
rosdep update --rosdistro jazzy || echo "WARN: rosdep update failed (retry later)"

echo "=== [5/7] ArduPilot source + prerequisites"
if [ ! -d "$HOME/ardupilot" ]; then
  git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git "$HOME/ardupilot"
else
  git -C "$HOME/ardupilot" pull && git -C "$HOME/ardupilot" submodule update --init --recursive
fi
cd "$HOME/ardupilot"
Tools/environment_install/install-prereqs-ubuntu.sh -y

echo "=== [6/7] Build ArduCopter SITL"
# install-prereqs creates a python venv and PATH entries in ~/.profile
set +u; source "$HOME/.profile"; set -u
./waf configure --board sitl
./waf copter

echo "=== [7/7] ardupilot_gazebo plugin"
if [ ! -d "$HOME/ardupilot_gazebo" ]; then
  git clone https://github.com/ArduPilot/ardupilot_gazebo.git "$HOME/ardupilot_gazebo"
fi
cd "$HOME/ardupilot_gazebo"
mkdir -p build && cd build
GZ_VERSION=harmonic cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j"$(nproc)"

mkdir -p "$HOME/airmouse_ws/src"

# ~/.bashrc additions (only once)
if ! grep -q "# >>> airmouse >>>" "$HOME/.bashrc"; then
cat >> "$HOME/.bashrc" <<'EOF'
# >>> airmouse >>>
source /opt/ros/jazzy/setup.bash
export ROSDISTRO_INDEX_URL=https://cdn.jsdelivr.net/gh/ros/rosdistro@master/index-v4.yaml
export GZ_VERSION=harmonic
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}
export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH:-}
[ -f $HOME/airmouse_ws/install/setup.bash ] && source $HOME/airmouse_ws/install/setup.bash
# <<< airmouse <<<
EOF
fi

echo
echo "=== DONE. Setup finished successfully. Tell Claude to verify. ==="
