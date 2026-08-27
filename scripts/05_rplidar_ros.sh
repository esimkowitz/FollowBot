#!/usr/bin/env bash
set -euo pipefail
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
# Install RPLIDAR ROS 2 driver (try apt, else build from source)
source /opt/ros/${ROS_DISTRO}/setup.bash
if sudo apt -y install ros-${ROS_DISTRO}-rplidar-ros; then
  echo "rplidar_ros installed from apt."
else
  echo "Building rplidar_ros from source (ros2 branch if available)..."
  mkdir -p ~/rplidar_ws/src
  cd ~/rplidar_ws/src
  if [ ! -d rplidar_ros ]; then
    git clone https://github.com/Slamtec/rplidar_ros.git
  fi
  cd ..
  rosdep install --from-paths src -y --ignore-src || true
  colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
  echo "Add to your session: source ~/rplidar_ws/install/setup.bash"
fi
