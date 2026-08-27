#!/usr/bin/env bash
set -euo pipefail
# Install ROS 2 on Ubuntu (ARM64).
# Jazzy is the Tier-1 distro for 24.04 (noble); Humble is 22.04-only and has
# no packages on noble at all. Override with ROS_DISTRO=... on a different release.
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
sudo apt update
sudo apt -y install software-properties-common
sudo add-apt-repository universe -y || true
sudo apt update

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" |       sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt -y install "ros-${ROS_DISTRO}-ros-base" \
  "ros-${ROS_DISTRO}-rmw-cyclonedds-cpp" \
  "ros-${ROS_DISTRO}-ros2launch" \
  "ros-${ROS_DISTRO}-teleop-twist-keyboard"

# Default to CycloneDDS (good on Wi-Fi)
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" | tee -a ~/.bashrc
echo "source /opt/ros/${ROS_DISTRO}/setup.bash" | tee -a ~/.bashrc

# rosdep
sudo apt -y install python3-rosdep
sudo rosdep init || true
rosdep update

echo "ROS 2 ${ROS_DISTRO} installed."
