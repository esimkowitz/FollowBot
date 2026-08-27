#!/usr/bin/env bash
set -euo pipefail
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
# Install the Nav2 stack.
# nav2-behaviors (plural) holds the runtime behavior plugins and is distinct
# from nav2-behavior-tree; the behavior server needs both.
sudo apt -y install \
  "ros-${ROS_DISTRO}-nav2-bringup" \
  "ros-${ROS_DISTRO}-nav2-controller" \
  "ros-${ROS_DISTRO}-nav2-planner" \
  "ros-${ROS_DISTRO}-nav2-bt-navigator" \
  "ros-${ROS_DISTRO}-nav2-behavior-tree" \
  "ros-${ROS_DISTRO}-nav2-behaviors" \
  "ros-${ROS_DISTRO}-nav2-waypoint-follower" \
  "ros-${ROS_DISTRO}-nav2-costmap-2d" \
  "ros-${ROS_DISTRO}-nav2-msgs"
echo "Nav2 packages installed."
