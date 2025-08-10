# FollowBot
A robot that can follow you around, based on ROS 2 Humble, Nav2, Sphero RVR, and RPLIDAR

## Contents
- `rvr_ros/`        – Rust: base driver skeleton (cmd_vel subscriber)
- `leg_detector/`   – Rust: LaserScan subscriber publishing legs (PoseArray) – placeholder logic
- `bringup/`        – Launch files to bring up RVR + RPLIDAR + leg detector (+ optional Foxglove bridge)
- `tracker_fuser/`  – Rust: Fuses leg detections with RPLIDAR scans to create a `PoseArray` of detected legs
- `foxglove_layouts/` – Foxglove Studio dashboard layout JSON

## Build (colcon) – placeholder
1) Install ROS 2 Humble, `ament_cargo`, and ros2-rust per https://github.com/ros2-rust/ros2_rust
2) From the workspace root:
   ```bash
   source /opt/ros/humble/setup.bash
   colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
   source install/setup.bash
   ```

## Launch
```bash
ros2 launch bringup follow_me_minimal.launch.py use_foxglove:=true rplidar_port:=/dev/ttyUSB0
```

---
## Nav2 + Tracker Fuser (pre-wired)
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch bringup follow_me_nav2.launch.py use_foxglove:=true rplidar_port:=/dev/ttyUSB0
```
