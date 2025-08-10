#!/usr/bin/env bash
set -euo pipefail
# Build this workspace
WS_DIR="${1:-$HOME/ros2_follow_me_ws}"
mkdir -p "$WS_DIR"
rsync -a --delete ./ "$WS_DIR"/
source /opt/ros/humble/setup.bash
cd "$WS_DIR"
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
echo "Build complete. Source: source $WS_DIR/install/setup.bash"
