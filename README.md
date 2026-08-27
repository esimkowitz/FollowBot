# FollowBot
A robot that can follow you around, based on ROS 2 Jazzy, Nav2, Sphero RVR, and RPLIDAR.
Runs on a Raspberry Pi 5 with Ubuntu 24.04 (noble) arm64.

## Contents
- `rvr_ros/`        – Rust: base driver skeleton (cmd_vel subscriber)
- `leg_detector/`   – Rust: LaserScan subscriber publishing legs (PoseArray) – placeholder logic
- `bringup/`        – Launch files to bring up RVR + RPLIDAR + leg detector (+ optional Foxglove bridge)
- `tracker_fuser/`  – Rust: Fuses leg detections with RPLIDAR scans to create a `PoseArray` of detected legs
- `foxglove_layouts/` – Foxglove Studio dashboard layout JSON

## Scripts

### Run on your Mac, before first boot
- `scripts/00_prepare_sd_macos.sh` – write cloud-init user/WiFi/SSH settings onto a
  freshly-flashed Ubuntu card, and enable the UART for the RVR. Raspberry Pi Imager
  cannot customise Ubuntu images, so this does that job instead. Prompts for the
  password and WiFi passphrase; writes only a SHA-512 hash and a derived PSK.

### Run on the Pi, after first boot
1. `scripts/01_os_prep.sh` – base packages, performance tweaks, zram, tools.
2. `scripts/02_ros2_ubuntu.sh` – ROS 2 apt install and rosdep init.
3. `scripts/03_ros2_rust_setup.sh` – rustup + ros2-rust scaffolding notes (build from source).
4. `scripts/04_foxglove_bridge.sh` – install & test `foxglove_bridge`.
5. `scripts/05_rplidar_ros.sh` – install RPLIDAR driver (apt first; fallback to source).
6. `scripts/06_nav2_packages.sh` – Nav2 stack (apt).
7. `scripts/07_csi_camera_ubuntu.sh` – enable CSI camera on Ubuntu (libcamera + rpicam-apps from source).
8. `scripts/20_build_workspace.sh` – build this workspace with colcon.
9. `scripts/21_launch_minimal.sh` – launch LiDAR+RVR+leg_detector(+Foxglove).
10. `scripts/22_launch_nav2.sh` – launch Nav2 + Rust tracker_fuser.

The on-Pi scripts default to `ROS_DISTRO=jazzy`, which is the Tier-1 distro for
Ubuntu 24.04. Override it (`ROS_DISTRO=kilted ./scripts/02_ros2_ubuntu.sh`) if you
move to a different release.

> These are scaffolds: read them, tweak variables at the top (device names, camera model),
> and run with `bash -x` if you want to see each step.

## Hardware notes
- The RVR is on `uart2-pi5` (GPIO 4/5 — physical pins 7 and 29), reached through a
  `/dev/rvr` udev symlink. The Pi 5 has five GPIO-header UARTs and keeps Bluetooth on a
  dedicated one, so there is no need to disable Bluetooth as on earlier Pis.
- Wiring: pin 7 (GPIO 4, TXD) → RVR RX, pin 29 (GPIO 5, RXD) → RVR TX, plus a common
  ground. Both sides are 3.3 V.
- The `uartN` → `/dev/ttyAMAn` mapping is not officially documented for the Pi 5. After
  first boot, check `ls -l /dev/serial* /dev/ttyAMA*` and adjust the udev rule in
  `/etc/udev/rules.d/99-followbot.rules` if the node is not `ttyAMA2`.
