# FollowBot
A robot that can follow you around, based on ROS 2 Humble, Nav2, Sphero RVR, and RPLIDAR

## Contents
- `rvr_ros/`        – Rust: base driver skeleton (cmd_vel subscriber)
- `leg_detector/`   – Rust: LaserScan subscriber publishing legs (PoseArray) – placeholder logic
- `bringup/`        – Launch files to bring up RVR + RPLIDAR + leg detector (+ optional Foxglove bridge)
- `tracker_fuser/`  – Rust: Fuses leg detections with RPLIDAR scans to create a `PoseArray` of detected legs
- `foxglove_layouts/` – Foxglove Studio dashboard layout JSON

## Scripts (run on the Pi)
1. `scripts/01_os_prep.sh` – base packages, performance tweaks, zram, tools.
2. `scripts/02_ros2_humble_ubuntu.sh` – ROS 2 Humble apt install and rosdep init.
3. `scripts/03_ros2_rust_setup.sh` – rustup + ros2-rust scaffolding notes (build from source).
4. `scripts/04_foxglove_bridge.sh` – install & test `foxglove_bridge`.
5. `scripts/05_rplidar_ros.sh` – install RPLIDAR driver (apt first; fallback to source).
6. `scripts/06_nav2_packages.sh` – Nav2 stack (apt) on Humble.
7. `scripts/07_csi_camera_ubuntu.sh` – enable CSI camera on Ubuntu (libcamera + rpicam-apps from source).
8. `scripts/20_build_workspace.sh` – build this workspace with colcon.
9. `scripts/21_launch_minimal.sh` – launch LiDAR+RVR+leg_detector(+Foxglove).
10. `scripts/22_launch_nav2.sh` – launch Nav2 + Rust tracker_fuser.

> These are scaffolds: read them, tweak variables at the top (device names, camera model),
> and run with `bash -x` if you want to see each step.
