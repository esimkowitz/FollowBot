#!/usr/bin/env bash
set -euo pipefail
source /opt/ros/humble/setup.bash
WS_DIR="${1:-$HOME/ros2_follow_me_ws}"
source "$WS_DIR/install/setup.bash"
RPLIDAR_PORT="${2:-/dev/ttyUSB0}"
ros2 launch bringup follow_me_nav2.launch.yaml use_foxglove:=true rplidar_port:="$RPLIDAR_PORT"
