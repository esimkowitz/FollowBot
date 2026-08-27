#!/usr/bin/env bash
set -euo pipefail
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
# Install Foxglove Bridge (apt when available, else from source)
source /opt/ros/${ROS_DISTRO}/setup.bash
if sudo apt -y install ros-${ROS_DISTRO}-foxglove-bridge; then
  echo "foxglove_bridge installed from apt."
else
  echo "Apt package not found, building from source..."
  mkdir -p ~/src && cd ~/src
  if [ ! -d foxglove_bridge ]; then
    git clone https://github.com/foxglove/ros-foxglove-bridge.git foxglove_bridge
  fi
  cd foxglove_bridge
  colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
  source install/setup.bash
fi
echo "To run: ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8765 -p address:=0.0.0.0 -p client_publish:=true"
