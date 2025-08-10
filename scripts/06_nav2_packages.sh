#!/usr/bin/env bash
set -euo pipefail
# Install Nav2 stack on Humble
sudo apt -y install       ros-humble-nav2-bringup       ros-humble-nav2-controller       ros-humble-nav2-planner       ros-humble-nav2-bt-navigator       ros-humble-nav2-behavior-tree       ros-humble-nav2-waypoint-follower       ros-humble-nav2-costmap-2d       ros-humble-nav2-msgs
echo "Nav2 packages installed."
