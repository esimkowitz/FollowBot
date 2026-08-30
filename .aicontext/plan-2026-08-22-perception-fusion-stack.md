# FollowBot: Perception Stack and Sensor Fusion

## Context

FollowBot is a Sphero RVR-based robot that follows a person, built on ROS 2 Humble on a Raspberry Pi 5 (Ubuntu 22.04 arm64). Today the repo is a scaffold: `rvr_protocol` is a real, tested Rust crate (70 passing tests) implementing the RVR UART protocol and its client layer, but the three ROS packages (`rvr_ros`, `leg_detector`, `tracker_fuser`) are 12–38 line stubs that log "alive" and publish nothing. There is no TF tree, no odometry, no camera integration, and the Rust packages don't build.

This plan wires the working protocol crate into a functioning robot and adds a camera-based perception layer, so the robot can identify and follow a specific person rather than the nearest leg-shaped object.

**Hardware:** RVR base over UART `/dev/ttyAMA0`; RPLIDAR A1 (360°, ~10 Hz, ~1° resolution); 2× Arducam B0309 (IMX708, ~102° wide, fixed focus, CSI-2) front and rear; Hailo-8L AI HAT+ (13 TOPS) on the M.2 slot.

**Core architectural idea — division of labor by sensor strength:**
- **Camera** → bearing (sub-degree) + identity (person class + ReID embedding). Poor range.
- **Lidar** → range (±2 cm) + 360° coverage. No identity.
- **Odom** → the frame everything is fused in.

Camera says "person at bearing −12°"; lidar says "solid object at −12°, range 2.34 m." Fused, that's a person at a known position. Neither sensor classifies alone.

### Decisions made during planning

| Decision | Choice | Why |
|---|---|---|
| Language split | Rust: driver, clusters, tracker, controller. Python: camera + Hailo | Boundary is the ROS message contract. HailoRT is C++/Python; FFI from Rust is a pointless detour |
| Leg detection | **Dropped.** Replaced by generic `scan_clusters` | At 1° resolution a leg is ~2 points at 3 m — below the existing `min_pts: 3`. No geometry left to classify |
| Follow control | `follow_controller` (20 Hz reactive) + costmap veto; Nav2 on demand | `NavigateToPose` at a 1.5 s reissue = 2.25 m stale goal; recovery behaviors spin away from the person |
| Obstacle avoidance | `nav2_costmap_2d` as a standalone veto layer | Costmap is an independent node; it senses obstacles without a planner downstream |
| Messages | Custom `followbot_msgs` (6 types) | `vision_msgs` can't carry a 512-float ReID embedding cleanly |
| odom→base_link TF | `robot_localization` EKF owns it, **not** the driver | Makes state-dependent covariance meaningful; prevents two TF publishers |
| Camera nodes | One node, both cameras | The Hailo is a single resource; the tracker wants both cameras batched |

---

## Node graph

```
/dev/ttyAMA0 ─▶ rvr_ros (Rust) ─▶ /odom, /rvr/imu, /rvr/battery, /diagnostics
                     ◀── /cmd_vel                        (no TF from here)

/dev/ttyUSB0 ─▶ rplidar_composition ─▶ /scan

CSI cam0 ─┐
CSI cam1 ─┴──▶ vision_node (Python, both cams, one Hailo)
                     ─▶ /perception/detections   PersonDetectionArray  @12 Hz
                     ─▶ /perception/camera_info_{front,rear}           latched

/scan ──▶ scan_clusters (Rust) ─▶ /perception/clusters  ClusterArray   @10 Hz

/perception/detections ─┐
/perception/clusters ───┼─▶ tracker_fuser (Rust) ─▶ /tracker/tracks, /tracker/target,
/odom, /tf ─────────────┘                            /tracker/status, /tracker/markers
/tracker/designate ─────┘

/tracker/target ──▶ follow_controller (Rust) ──▶ /cmd_vel  @20 Hz
/local_costmap/costmap ──┘  (veto layer)

robot_state_publisher ─▶ /tf_static     ekf_node ─▶ /tf (odom→base_link)
nav2_costmap_2d (local, standalone)     slam_toolbox ─▶ /tf (map→odom), /map
```

`tracker_fuser` and `follow_controller` are **separate nodes**: the tracker runs event-driven on sensor arrival (jittery); the controller runs a hard 20 Hz timer that must never miss a tick because of the RVR deadman.

---

## Work by area

### A. Build system — fix first, everything depends on it

All three Rust packages contain `find_package(ament_cargo REQUIRED)`. **`ament_cargo` is not a CMake package** — it's a colcon build type. Every Rust package currently fails at configure time.

- Delete `CMakeLists.txt` from `rvr_ros`, `leg_detector`, `tracker_fuser`
- Set `<export><build_type>ament_cargo</build_type></export>` in each `package.xml`
- Extend `scripts/03_ros2_rust_setup.sh` to actually install ros2-rust (it currently only *prints* instructions): clone `ros2_rust`, install `cargo-ament-build` and `colcon-ros-cargo`, run the vendor step, build the overlay
- Add a `.gitignore` (none exists). Remove the committed `src/tracker_fuser/target/` directory
- `rvr_protocol` needs a `package.xml` to be visible to colcon, and `rvr_ros` must add it as a path dependency

Files: `src/*/CMakeLists.txt`, `src/*/package.xml`, `scripts/03_ros2_rust_setup.sh`, `scripts/20_build_workspace.sh`

### B. `followbot_msgs` — new package

Six messages. Reuse standard types inside them (`std_msgs/Header`, `geometry_msgs/Point`, `Vector3`).

- **`PersonDetection`** — bearing (rad, CCW+), bearing_sigma, bbox, score, `range_hint` (−1 if invalid), camera enum, `embedding` (float32[]), `embed_quality`
- **`PersonDetectionArray`** — header + detections + `inference_latency`
- **`Cluster`** — centroid, `min_range`, bearing, width, radius, point_count, `occluded_left/right`
- **`ClusterArray`**, **`Track`** (id, pose, velocity, covariance, state enum, confidence, hits/misses, `lidar_backed`, `rear_originated`), **`TrackArray`**, **`TrackerStatus`**

### C. `rvr_ros` driver — wrap the existing client

`src/rvr_protocol/src/client.rs` already provides `RvrClient` with an RX thread, sequence counter, pending-request map, notification dispatch, and slot registry. This is a **ROS wrapper, not a rewrite**.

**Three fixes to `rvr_protocol` first:**
1. `receive_loop` treats every read error as transient and sleeps forever — a yanked adapter spins at 100 Hz with nothing upstream learning. Add a consecutive-error counter, a `link_failed` flag, and `is_link_healthy()`
2. Rename `on_notification` → `set_notification_handler` (it replaces rather than adds — silent footgun)
3. Add `enable_motor_fault_notifications` (`classify()` handles the notification but nothing enables it)

**Thread architecture** (keep ROS callbacks off the serial path — rclrs 0.3 is single-threaded by default):
- **A** `rclrs::spin` — subscription callbacks write to a shared `Mutex<CommandCell>`, never touch serial
- **B** `rvr-rx` — already exists inside `RvrClient`
- **C** `rvr-tx` — hard 20 Hz loop, the *only* thread issuing drive commands
- **D** `rvr-housekeeping` — 1 Hz battery, link health, diagnostics

Publish `/odom` and `/rvr/imu` directly from the notification handler (rclrs publishers are `Send + Sync`); queue only the low-rate notifications.

**cmd_vel → `drive_tank_si_units`, not `drive_rc_si_units`.** RC drive looks like a closer match to `Twist` and that's the trap: it takes deg/s, carries gamepad-oriented directional latching flags, and activates an opaque control system tuned for human input. Tank SI is unambiguous arithmetic against the simplest onboard controller, and is already unit-tested.

Scale both wheels by a common factor at saturation rather than clipping independently — independent clipping silently changes commanded curvature.

**Three nested watchdogs:** host deadman 200 ms (matches the existing `deadman: 0.2` param) → RVR control-system timeout set to 1000 ms → link watchdog with 1 Hz reconnect.

**Odometry conversion — the highest-risk code in the driver.** RVR streams in an X-right/Y-forward frame; REP-103 is X-forward/Y-left. Both right-handed, so it's a pure 90° rotation:

```rust
fn rvr_to_rep103(x: f32, y: f32) -> (f32, f32) { (y, -x) }   // one function, used everywhere
```

RVR yaw is clockwise-positive (consistent with `drive_with_heading`); REP-103 is CCW-positive, so `yaw_ros = -yaw_rvr.to_radians()`, wrapped to (−π, π]. **A sign error here looks fine driving straight and diverges violently on turns** — Phase 2 gates for it explicitly.

Stream two slots on `Target::Secondary` at 33 ms: (Quaternion, Imu) and (Locator, Velocity, Gyroscope). Bandwidth ≈ 2620 B/s of 11520 available — 23%, comfortable. **Do not use streamed `Encoders`** — they're `u32` and wrap on reverse; the RVR already fuses encoders+IMU into Locator.

REP-103 subtlety: `pose` is in `header.frame_id` (odom), `twist` is in `child_frame_id` (base_link). Getting that wrong breaks the EKF on turns.

**Covariance is state-dependent** — inflate position variance on motor stall, yaw variance during fast rotation, and skip publishing entirely on `StreamInvalid`. Use `1e6` for unused DOFs, not `1e9` (some code paths invert the matrix).

**No TF broadcaster.** Add a `publish_tf` param defaulting false for early bring-up, logging a WARN while true.

Files: `src/rvr_protocol/src/client.rs`, `src/rvr_ros/src/{main,lib}.rs`

### D. `scan_clusters` — replaces `leg_detector`

Rename the package. The name is load-bearing documentation: `leg_detector` implies a capability the hardware cannot deliver.

Per scan: filter to [range_min+0.02, 6.0] m → segment with an **adaptive** threshold (`thresh = C0 + C1·min(r_i, r_i+1)`, Dietmayer) rather than the current fixed `jump_thresh: 0.2`, because a fixed threshold over-segments near objects and under-segments far ones → reject only `point_count < 2` or `width > 1.2 m` → emit centroid, `min_range`, bearing, width, radius, and `occluded_left/right`.

Deliberately **no** radius filter and **no** `min_pts: 3` — those are the leg-classification filters that cause range blindness. `occluded_*` lets the tracker distinguish "person moved away" from "person behind that chair."

Emit in `frame_id: "laser"` with the scan's stamp verbatim; the tracker does one time-aware transform.

Files: `src/leg_detector/` → `src/scan_clusters/`

### E. `vision_node` — one Python node, both cameras

One node because the Hailo is a single resource and the tracker wants both cameras batched. Alternate cameras (front-weighted, e.g. 2:1) rather than running two competing processes.

libcamera → 640×640 letterboxed → Hailo YOLOv8 person detection → crop each box → Hailo ReID embedding (`repvgg_a0_person_reid_512`, ~3% of the accelerator) → publish `PersonDetectionArray`.

**Timestamp the frame at capture, not at publish**, and report `inference_latency` separately — downstream prediction depends on knowing when the photons arrived. Publish `CameraInfo` latched from a real calibration.

**Compute bearing with the correct formula:**

```
bearing = atan2(cx - u, fx)          # from CameraInfo — NOT a hardcoded FOV
```

The naive `(cx-u)/(W/2) · (HFOV/2)` is wrong because angle is not linear in pixel position (`tan θ ∝ pixel offset`). Its error is **zero at center and at the edges, peaking ~4° mid-frame** — the nastiest possible shape, since centering a person and driving forward looks perfect. At 3 m, 4° is 21 cm of lateral error: enough to mis-associate two people walking side by side, and it injects a phantom velocity that reverses sign as the target crosses frame center.

Set `range_hint = -1` when the bbox touches a vertical edge (the height-based range estimate is only valid for a fully-visible standing person, and is ±20% at best — a gate-widening prior, never a primary measurement).

**Verify the actual hFOV empirically.** Arducam lists B0309 as "102°" ambiguously between horizontal and diagonal, and vendors habitually publish diagonal because it's the larger number. Calibrate rather than trust the spec sheet.

### F. `tracker_fuser` — the core

**Association** (on each detection batch): predict all tracks to the detection stamp → lift each detection to a 2D measurement by finding lidar clusters within a bearing gate → build a cost matrix → **Hungarian assignment**.

- Gate: `3·sqrt(σ_bearing² + σ_cluster²) + atan2(width/2, range)`, floored at 0.10 rad and capped at 0.35 rad
- Multiple clusters pass → take the **smallest `min_range`** (nearest surface is the person; farther is wall seen past them), plus ~0.15 m to reach body center
- Propagate polar→Cartesian covariance through the Jacobian. The measurement is a **thin banana** (~8 cm radial, ~2 cm tangential at 3 m); an isotropic circle throws away the bearing precision that is the whole point of the camera
- Cost = Mahalanobis + `W_REID`·(1 − cosine) + small cross-camera penalty, with a hard χ² gate
- **Hungarian, not greedy** — greedy's failure mode is exactly the two-people-close-together case

**Leftovers:** unmatched lidar-backed front-camera detections spawn TENTATIVE tracks; rear-originated tracks are flagged and can never be auto-selected as target. Before dropping an unmatched track, check for a lidar-only cluster consistent with its velocity — **this is what keeps a track alive when the person leaves the camera FOV but is still visible to the 360° lidar.** Unmatched clusters are ignored entirely; never spawn a track from a cluster alone.

**Lifecycle:** `TENTATIVE →(3 hits in 5 frames)→ CONFIRMED ⇄ OCCLUDED →(3 s)→ LOST →(ReID cos ≥ 0.80)→ CONFIRMED`, deleting LOST after 20 s. Re-acquisition threshold is deliberately strict — silently following a stranger is much worse than failing to re-acquire.

**Stop predicting motion after ~1.5 s of occlusion** (zero the velocity, keep growing covariance). A constant-velocity extrapolation is credible for about a second of human motion; beyond that a confidently-wrong position 4 m from truth is worse than an uncertain position at last-seen. Small detail, large effect on re-acquisition rate.

**Filter:** constant-velocity Kalman (4-state) in the **odom frame**. ReID gallery per track — a small set of embeddings rather than a single running average, so appearance variation across viewpoints is captured.

**Target selection:** operator designates via `/tracker/designate`; the target survives occlusion by ReID; auto-selection only from front-camera, lidar-backed, CONFIRMED tracks.

Files: `src/tracker_fuser/src/main.rs` (current stub has the wrong architecture — `base_link` frame, 1.5 s NavigateToPose reissue)

### G. `follow_controller` — new package

20 Hz pure-pursuit-style control. Goal point at standoff distance along the line from target back toward the robot, led by `target_vel × ~0.4 s`. Proportional on bearing-to-target (keeps the person centered in the 102° FOV) and on standoff error, with linear speed scaled by `align²` so the robot turns-then-goes rather than driving wide arcs that swing the person out of frame.

**Standoff:** 1.2 m with a deadband (~1.0–1.5 m) to stop 20 Hz oscillation; limited reverse when too close; hard stop at ~0.6 m.

**Costmap veto** — the obstacle-avoidance mechanism. Each tick, forward-simulate the commanded arc ~0.8 s against `/local_costmap/costmap`: LETHAL → zero linear velocity but keep angular (rotate away); INSCRIBED → scale down. This is a **veto, not a planner** — the target proposes a direction, the costmap can block it, but never invents a route. Sufficient here because a person-following robot has a continuously-updated human-provided reference trajectory: the person walked through that space seconds ago.

**Escalation for the case a veto can't solve** (concave obstacles, doorways): if vetoed >2 s while the target is still tracked, send **one** `NavigateToPose` goal to the full Nav2 stack, cancelled on regaining line-of-sight. Same path serves "go to last known position" when the target is LOST.

### H. TF, localization, Nav2

- **URDF + `robot_state_publisher`**: `base_link` at the RVR's rotation center, `laser`, `camera_{front,rear}_link` plus their optical frames (fixed joint, rpy `-π/2 0 -π/2`), wheels
- **`robot_localization` `ekf_node`** in 2D mode owns `odom→base_link`. Fuse **velocities** from `/odom` (vx, vy, vyaw) and yaw from `/rvr/imu` — fusing absolute position from a dead-reckoning source is double-integration and drifts on drift
- **`slam_toolbox`** online async for `map→odom`
- **Nav2 rework** in `bringup`: local costmap as a standalone node (4×4 m rolling, `observation_persistence: 0.0`); fix the costmap param nesting (currently top-level, so the params never reach the nodes); add the missing `nav2_lifecycle_manager`; `behavior_server` comes from `nav2_behaviors`, not `nav2_behavior_tree`. Full Nav2 launched but **inactive**, activated on demand

Files: `src/bringup/launch/*.launch.yaml`, `src/bringup/config/nav2_params.yaml`, new `urdf/` and `config/ekf.yaml`

---

## Phasing and verification gates

Ordered so each phase yields a testable artifact and the Hailo isn't on the critical path until Phase 5.

| # | Phase | Gate — you know it works when… |
|---|---|---|
| 0 | Build fix + `followbot_msgs` | A throwaway Rust node publishes a `TrackArray`; `ros2 topic echo` shows correct fields; a Python subscriber reads the same topic. **Highest-risk item in the plan** — ros2-rust message generation on Humble/arm64 is the least-trodden path. Find out day one |
| 1 | URDF + TF | `view_frames` shows one connected tree, no warnings. A `PointStamped` at (1,0,0) in `camera_front_optical_frame` transforms to 1 m directly in front of `base_link` — not above or beside. Ruler-measured lidar↔camera offset matches TF within 1 cm |
| 2 | `rvr_ros` driver | (a) `rvr_probe` prints both firmware versions + battery. (b) Track-width calibration: `ω=1.0` for 6.283 s ends within ±10° of start; `v=0.3` for 10 s covers 3.0 m ±5 cm. (c) Push 1 m forward → `odom.x` +1.0, `y`≈0; push 1 m left → `y` increases. (d) **Rotate 90° CCW by hand → yaw reads +1.571, not −1.571.** (e) Rotate 90° then drive → `twist.linear.x` positive, `y`≈0. (f) Kill the `/cmd_vel` publisher → stop within 250 ms; kill the driver mid-drive → stop within 1.2 s. (g) 2 m teleop square → <0.3 m position and <15° yaw error |
| 3 | `scan_clusters` | Person at 2 m → cluster `min_range` within 10 cm of tape, `width` 0.15–0.45 m. At 4 m → still present, `point_count ≥ 2`. Walk a full circle → tracks continuously through 360° with <10% dropouts at 3 m |
| 4 | Camera pipeline, **CPU-only** (ONNX/YOLOv8n @320, 3–5 fps) | Too slow to follow, plenty to validate contracts. (a) `inference_latency` stable; stamps always in the past. (b) **Bearing accuracy: person at 0°, 20°, 40° measured with a floor protractor → within 2° at all three.** This is the gate that catches the naive-linear-bearing bug — its signature is *perfect at center, ~17° reading at 20°*. (c) Rear camera bearings near ±π with correct sign. (d) Front camera holds 15 fps in a dim room |
| 5 | Hailo swap-in | Same node, Hailo backend. Detection rate ≥12 Hz, latency <80 ms, bearing gate from Phase 4 still passes unchanged |
| 6 | `tracker_fuser`, no control | Tracks in Foxglove: stable IDs while walking; ID survives a 1–2 s occlusion behind a doorway; walking out of view and returning re-acquires the same ID; two people get two IDs that don't swap when they cross |
| 7 | `follow_controller`, tethered | Robot maintains 1.2 m ±0.3 m; stops for an obstacle in its path; stops within 250 ms when the target is lost |
| 8 | Failure drills | Occlusion, FOV exit, two people crossing, Hailo node killed, serial unplugged — each produces the documented degraded behavior, not a crash or a runaway |
| 9 | Endurance | 15 min continuous follow: no track-ID leak, no memory growth, odom drift bounded by SLAM correction |

---

## Notes and risks

- **Phase 0 is the real risk.** If ros2-rust message generation doesn't work on Humble/arm64, the Rust/Python split needs rethinking. Prove it before building anything on top.
- **B0309 is listed as discontinued** by Arducam (successors B0638/B0639, same IMX708 family). Confirm availability.
- **The `avoid_warnings=2` in `scripts/01_os_prep.sh` suppresses undervoltage warnings.** With a Hailo HAT and two cameras drawing power, that's now actively hiding a signal worth seeing. Consider reverting it, and monitor `vcgencmd get_throttled` in the diagnostics node instead.
- **The Hailo AI HAT+ occupies the sole PCIe M.2 slot** — no NVMe alongside it without a switch board that forces Gen 2. Use USB 3.0 storage.
- **Foxglove layout topics are stale.** `follow_me_debug.json` references `/legs` and `/camera/image_raw`; update to `/perception/clusters`, `/tracker/markers`, and the new image topics.
- **Rear camera may prove unnecessary** — the RPLIDAR already covers 360°, and the lidar-only track-continuation path handles most FOV exits. It's cheap to defer; the architecture treats cameras as a list with known extrinsics.
