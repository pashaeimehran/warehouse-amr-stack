# Implementation plan

Maps the remaining work onto the roadmap (`Warehouse Mobile Manipulator
Roadmap.md`), in dependency order. Each item names its done-when criterion
from the roadmap; an item is ticked only when that criterion was verified by
a run. Status of individual components: `docs/implementation-status.md`.

Legend: [x] verified, [~] in progress or partial, [ ] not started.

## v0.1-core (tagged)

- [x] everything in the roadmap row; GitHub Actions green, tag `v0.1-core`

## v0.2-platform

- [x] `qos_lab`: 3 profiles x 2 payloads x 2 RMWs, loss and latency counters (3 runs)
- [x] `executor_lab`: slow plus 50 Hz callback under 4 configurations (3 runs)
- [x] both tables with a written explanation in `docs/02-ros2-architecture.md`
- [x] `robot_interfaces` (all listed messages)
- [x] lifecycle skeleton and component container; manual `configure` and `activate` work
- [x] base URDF with 2D LiDAR, RGB-D, IMU, wheel encoders; gz_ros2_control diff drive and joint_state_broadcaster
- [x] `/cmd_vel` drives the robot through ros2_control; `/odom`, `/joint_states` publish
- [x] warehouse world incl. walking actor (SHA256-pinned Fuel mesh in the images)
- [x] ros_gz_bridge, ground-truth pose publisher (OdometryPublisher)
- [x] layered launch: sim, robot, navigation, perception, system (+ mapping)
- [x] launch tests: topics within 20 s, TF complete, `/cmd_vel` moves the wheels (headless)
- [~] launch tests in CI: `ros` job written (actionlint clean); first GitHub run needs a push (author)
- [x] map with slam_toolbox (saved to `warehouse_bringup/maps/`; measured against true geometry)
- [x] Nav2 with AMCL; waypoint script with the simple commander; dock to Station A autonomously (launch test)
- [x] apriltag_ros detections as TF; tags detected (launch test, error against the landmark map)
- [x] `docs/02-ros2-architecture.md` (QoS and executor tables, TF tree, launch layers, topics, navigation); node graph image pending (needs a display)
- [x] switch the stack to Cyclone DDS (decided in docs/02) and re-run the ROS suite (88 tests green)
- [ ] RViz config; navigation GIF (needs a display; author)

## v0.3-perception (week 2)

- [ ] 30 Gazebo frames saved; pretrained YOLO11n baseline latency (`baseline_latency.json`), in `tools/`
- [~] `perception_core` (Python, no ROS): letterbox, IoU, own NMS, `DetectorBackend`, `MockBackend`, YOLO head decoder done (27 pytest); `TorchBackend` open (needs PyTorch in a tools environment)
- [ ] `perception` lifecycle node: `Detection2DArray` with source stamps, drop-old-frames, `PerceptionDiagnostics`; node tests with `MockBackend`
- [ ] evaluation harness on 50 hand-checked frames; `docs/03-perception.md`
- Blocker: person detection needs the walking actor's mesh; GPU inference needs the WSL NVIDIA libraries (CPU inference is possible but slow)

## v0.4-localization

- [ ] `localization_core` (C++, Eigen, no ROS): EKF x, y, yaw, v, yaw rate; odometry and IMU prediction; AprilTag update from `warehouse_sim/config/tag_map.yaml`; Mahalanobis gating; Joseph form; GTests
- [ ] EKF ROS component (map to odom TF, pose with covariance); robot_localization baseline; rosbags; evo ATE and RPE
- [ ] GTSAM iSAM2 factor graph (needs GTSAM: not in the Jazzy apt set; check `ros-jazzy-gtsam` or a pinned source build in the image)
- [ ] slip, IMU bias, tag dropout runs; four-way comparison; Nav2 on the own EKF; `docs/04-localization.md`

## v0.5-world-model

- [ ] `camera_geometry` (C++, no ROS): back projection, robust median depth ROI, covariance; GTests within 1 mm
- [ ] `perception_3d` real implementation (replaces the v0.2 skeleton)
- [ ] `lidar_processing` (pcl_ros ground removal, clustering); Open3D offline figure
- [ ] `tracking_core` (Kalman CV, gating, association, track lifecycle); `tracking` real implementation; fusion; `docs/05-3d-perception.md`

## v0.6-intelligent-navigation

- [ ] `nav2_tracked_obstacle_layer`; MPPI tuning with before and after table
- [ ] `mission_manager` (BehaviorTree.CPP v4) with BT node tests; `safety_supervisor` FSM gating `/cmd_vel_nav` to `/cmd_vel`
- [ ] `fleet_interface` (MQTT, VDA 5050 subset); scenario runner; scenarios 1 and 2; `docs/06-navigation-integration.md`

## v0.7-manipulation

- [ ] UR5e and gripper on the base; arm controllers; MoveIt 2 config; planning scene
- [ ] grasp generator; pick node; DetachableJoint; `cartesian_velocity_controller`; arm interlock
- [ ] 20 pick trials (at least 18 succeed); scenario 9; `docs/07-manipulation.md`

## v0.8-optimized-perception

- [ ] seeded capture, auto-labeller, randomization, dataset; YOLO11n fine-tuning (TinyGPU: needs the author's authorisation)
- [ ] ONNX export and parity; ONNX Runtime backends; TensorRT engine and `perception_trt` (need the GPU)
- [ ] four-backend benchmark; `docs/08-model-and-deployment.md`

## v0.9-reliability and v1.0

- [ ] `docs/09-fmea.md` before any fault code; `health_monitor`; `fault_injector` (out of band)
- [ ] scenarios 3 to 8 and 10; 10 consecutive green runs; `scenarios.yml`
- [ ] tracing, profiling, `performance.yml` gate; release workflow, runtime image, Makefile, GitLab mirror
- [ ] final README, `docs/11` and `docs/12`, interview notes, demo video

## Decisions recorded

- ROS 2 and Gazebo run only in Docker images; the host has no ROS. `tools/ci/check_ros.sh` is the ROS gate (build, test); colcon trees live under `build/` (`build/colcon`, `build/install`, `build/log`), inside the per-service overlay.
- The ROS packages share one apt list, `docker/install_ros_packages.sh` (`ci` and `dev` variants).
- Velocity commands are `geometry_msgs/TwistStamped` end to end: Jazzy's diff_drive_controller accepts only stamped commands, so Nav2 and the safety supervisor will use the stamped variant.
- The world is generated (`warehouse_sim/scripts/generate_world.py`) and the committed output is checked for drift by a test. The Gazebo world frame is the map frame, origin at the dock.
- Ground truth comes from Gazebo's OdometryPublisher on `/ground_truth/odom`; it is for evaluation only and never feeds control.
- `perception_3d` and `tracking` exist as lifecycle skeletons since v0.2, explicitly marked as such in their sources, so that composition and lifecycle are tested from week 1.
