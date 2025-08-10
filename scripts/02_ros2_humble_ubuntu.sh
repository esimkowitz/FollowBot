#!/usr/bin/env bash
set -euo pipefail
# Install ROS 2 Humble on Ubuntu 22.04 (ARM64)
sudo apt update
sudo apt -y install software-properties-common
sudo add-apt-repository universe -y || true
sudo apt update

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" |       sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt -y install ros-humble-ros-base         ros-humble-rmw-cyclonedds-cpp         ros-humble-ros2launch         ros-humble-teleop-twist-keyboard

# Default to CycloneDDS (good on Wi-Fi)
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" | tee -a ~/.bashrc
echo "source /opt/ros/humble/setup.bash" | tee -a ~/.bashrc

# rosdep
sudo apt -y install python3-rosdep
sudo rosdep init || true
rosdep update

echo "ROS 2 Humble installed."
