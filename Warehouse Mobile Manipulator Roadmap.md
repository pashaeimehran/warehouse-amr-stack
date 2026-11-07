# Warehouse AMR and Mobile Manipulator: 6-Week Combined Roadmap

Oct 15, 2026 · @Mehran Pashaei

## Overview

In six weeks (Mon 19 Oct to Sun 29 Nov 2026) you build one public repository, `warehouse-amr-stack`, that combines every skill from the August 8-week AMR roadmap with the October mobile manipulator plan. Nothing is removed; a few items are added where the two plans meet.

**What the robot does.** A warehouse AMR with a 6-DOF arm receives a VDA 5050 order over MQTT. It navigates with Nav2, localizes on its own EKF and a GTSAM factor graph, and detects people, pallets, KLT boxes and shelf items with a fine-tuned detector deployed through TensorRT. It tracks moving people in a fused world model, slows or stops through a deterministic safety supervisor, picks the ordered item with MoveIt 2 and a custom ros2\_control controller, and delivers it. Ten automated scenarios in headless Gazebo prove it degrades safely when sensors, models, localization or the arm fail.

**The engineering question it answers**

> How do you build an autonomous mobile manipulator in which learned, probabilistic, occasionally wrong perception drives deterministic, bounded, provably safe behaviour, and how do you prove with automated tests and measurements that it degrades safely when any component fails?

**Time budget**

| Item | Value |
| --- | --- |
| Weeks | 6 (42 days, one release or more per week) |
| Weekday and Saturday sessions | about 6 hours |
| Sunday sessions | about 4 hours (release, docs, overflow) |
| Weekly total | about 40 hours |
| Program total | about 240 hours (the August plan was about 155) |
| Daily split | Learn 60 min, build 3.5 h, test and measure 60 min, document and commit 30 min |

**Rule when behind:** nothing gets dropped. Overflow moves to the next Sunday or evening in the same order, and a release tag waits until its definition of done holds.

**Constraints**

- Simulation only, no physical robot.
- NVIDIA RTX GPU with 6 to 12 GB VRAM: small detectors, Gazebo instead of Isaac Sim as the simulator.
- Finishes before the December thesis crunch.
- No code, rosbags or data from your LHFT student job go into this public repo. Your real-robot experience stays on the CV as work experience, separate from the project.

## Timeline at a glance

&#91;embedded content: six-week timeline · ten releases, 19 Oct to 29 Nov 2026\]

Four weeks ship two releases each, week 4 and week 5 one each; the order follows dependencies, so perception exists before tracking, tracking before navigation, and the arm before the final data and reliability work.

## Skills coverage

Every skill from both plans has a place in the six weeks. "From" shows the source: August (8-week AMR roadmap), October (mobile manipulator plan), Both, or Added (new where the two plans meet).

| Skill | From | Where it is built | Week |
| --- | --- | --- | --- |
| Modern C++17/20: RAII, rule of zero/three/five, ownership, move, const, optional, variant, chrono | Both | `robot_core`, every C++ package | 1 |
| Concurrency: thread, mutex, condition\_variable, atomic | August | `TaskQueue`, `LatestSample` | 1 |
| Interfaces and dependency injection | August | `IClock`, `INavigationClient`, `IManipulationClient` (added) | 1 |
| CMake presets, ament\_cmake | Both | all packages | 1 |
| GoogleTest, GoogleMock, pytest, launch\_testing | Both | all packages | 1 to 6 |
| ASan, UBSan, TSan, clang-format, clang-tidy, gcovr coverage | Both | CI | 1 |
| Docker multi-stage images, Compose profiles | Both | `docker/` | 1, 6 |
| GitHub Actions (4 workflows), GitLab CI mirror, conventional commits | August | `.github/`, `.gitlab-ci.yml` | 1, 6 |
| UML/C4 architecture diagrams, Doxygen headers | August | `docs/` | 6 |
| RMW, DDS, QoS measured on Fast DDS and Cyclone DDS; rmw\_zenoh awareness | August | `qos_lab` | 1 |
| Executors, callback groups, callback starvation | August | `executor_lab` | 1 |
| Lifecycle nodes, composition, intra-process communication | August | perception, tracking, safety | 1 to 3 |
| Custom interfaces | Both | `robot_interfaces` | 1 |
| TF2, REP 105 and REP 103 frames | August | whole stack | 1, 3 |
| Layered launch architecture, parameter files | August | `warehouse_bringup` | 1 |
| ros2\_control: diff drive with gz\_ros2\_control | Both | base | 1 |
| ros2\_control: arm joint\_trajectory\_controller, gripper, controller switching | October | arm | 4 |
| Custom ros2\_control controller plugin (Cartesian velocity, pluginlib) | October | `cartesian_velocity_controller` | 4 |
| Gazebo Harmonic world authoring, ros\_gz\_bridge, actors | Both | `warehouse_sim` | 1 |
| Nav2, slam\_toolbox, AMCL, MPPI tuning, Nav2 BT and recoveries | Both | navigation | 1, 3 |
| Custom Nav2 costmap layer plugin | August | `TrackedObstacleLayer` | 3 |
| BehaviorTree.CPP v4, Groot2 | Both | `mission_manager` | 3, 4 |
| Explicit FSM for safety, ISO 3691-4 vocabulary | August | `safety_supervisor` | 3 |
| Safety supervisor also gates arm motion | Added | `safety_supervisor` | 4 |
| MQTT and VDA 5050 subset (order, state, connection) | August | `fleet_interface` | 3 |
| VDA 5050 pick and drop actions mapped to the arm | Added | `fleet_interface`, `mission_manager` | 4 |
| Own C++ EKF for robot pose with AprilTag landmarks and Mahalanobis gating | October | `localization_core` | 2 |
| robot\_localization EKF (odometry and IMU) | Both | baseline | 2 |
| GTSAM iSAM2 factor graph with IMU preintegration and landmark factors | October | `factor_graph_localization` | 2 |
| apriltag\_ros | October | landmarks | 1 |
| Trajectory evaluation with evo (ATE, RPE), slip, bias and tag dropout runs | October | `tools/analysis` | 2 |
| Own constant velocity Kalman tracker, gating, data association, track lifecycle | August | `tracking_core` | 3 |
| Track-level camera and LiDAR fusion with per-sensor noise R | August | `tracking` | 3 |
| OpenCV, cv\_bridge, CameraInfo and intrinsics | August | `perception_core` | 2 |
| PyTorch inference, IoU, NMS, precision, recall, mAP | Both | `perception_core` | 2, 5 |
| Detector backend abstraction, mock backend, GPU-free tests | August | `perception_core` | 2 |
| Frame dropping policy, perception diagnostics, source timestamps | August | `perception` | 2 |
| Camera geometry: back projection, robust depth ROI, published covariance | August | `camera_geometry`, `perception_3d` | 3 |
| 3D item poses for grasping | October | `perception_3d` | 4 |
| PCL (pcl\_ros ground removal, Euclidean clustering) and Open3D | August | `lidar_processing`, `tools/analysis` | 3 |
| Synthetic data: capture node, projection auto-labeller, domain randomization, split by episode | August | `tools/dataset_tools` | 5 |
| Gazebo bounding box and segmentation cameras; labels cross-checked against your labeller | October + Added | `tools/dataset_tools` | 5 |
| Randomization ablation | October | `tools/training` | 5 |
| Fine tuning YOLO11n, per-class mAP against the COCO baseline | Both | `tools/training` | 5 |
| ONNX, ONNX Runtime (CUDA and TensorRT execution providers) | August | `perception_core` | 5 |
| TensorRT FP16 engines and a native C++ TensorRT inference node | Both | `perception_trt` | 5 |
| CUDA concepts: host and device memory, streams, batching | August | benchmarks | 5 |
| Ultralytics licensing separation | August | `tools/` vs `src/` | 5 |
| Kinematics: forward, velocity, Jacobian, damped least squares | October | notes, controller | 4 |
| Mobile manipulator URDF/xacro (UR5e and gripper on the base) | October | `warehouse_description` | 4 |
| MoveIt 2: SRDF, planning scene, MoveGroupInterface pick in C++ | October | `manipulation` | 4 |
| Gazebo DetachableJoint, KDL or Pinocchio | October | `manipulation` | 4 |
| Pick success rate over 20 trials | October | `tools/analysis` | 4 |
| FMEA with 12+ failure modes, including localization and arm | August + Added | `docs/` | 6 |
| Health monitor: watchdogs, QoS deadline and liveliness, diagnostic\_msgs | August | `health_monitor` | 6 |
| Out-of-band fault injection, plus tag dropout and arm joint freeze | August + Added | `fault_injector` | 6 |
| Degraded modes, hysteresis, controlled stop | August | `safety_supervisor`, `mission_manager` | 6 |
| SiL scenario runner and 10 scenarios (8 from August, 2 added for the arm) | Both + Added | `scenarios/` | 3, 4, 6 |
| Flake-free suite over 10 consecutive runs | August | CI | 6 |
| ros2\_tracing and LTTng | August | `tools/analysis` | 6 |
| End-to-end latency instrumentation and per-stage budget | August | whole stack | 6 |
| perf, flame graphs, heaptrack or massif | August | profiling | 6 |
| Jitter of perception, base and arm control loops | August + Added | benchmarks | 6 |
| Executor and QoS configuration comparison | August | benchmarks | 6 |
| CI performance regression gate | August | `performance.yml` | 6 |
| Real-time vocabulary, PREEMPT\_RT trade-offs | August | `docs/` | 6 |
| Isaac Sim and Isaac Lab architecture reading | Both | notes | 6 |
| Jazzy to Lyrical migration notes | August | notes | 6 |
| Informed view on Physical AI, VLA and RL | August | README future work | 6 |
| README, demo video, limitations doc, interview notes | Both | repo | 6 |

## Technology stack

One choice per category; know the alternatives well enough to defend your choice in an interview.

| Category | Chosen | Alternatives to discuss | Why |
| --- | --- | --- | --- |
| OS | Ubuntu 24.04 LTS | 22.04, 26.04 | Required pairing for Jazzy |
| ROS 2 | Jazzy Jalisco (LTS) | Humble, Kilted, Lyrical Luth | Mature ecosystem; postings name Humble or Jazzy |
| Simulator | Gazebo Harmonic (LTS) | Isaac Sim, Gazebo Jetty, MuJoCo, Webots | Pairs with Jazzy, runs headless in CI, fits your GPU |
| Runtime language | C++17 with selected C++20 | Rust | What German robotics codebases compile |
| Tooling language | Python 3.12 |  | Training, data, scenarios, analysis |
| Build | CMake presets, ament\_cmake | Bazel, Meson | Universal in ROS |
| Deep learning | PyTorch | TensorFlow, JAX | Named in German postings |
| Detector | Ultralytics YOLO11n/s (training only) | RT-DETR, RF-DETR, YOLOX, torchvision Faster R-CNN | Fast to fine-tune; runtime depends only on the ONNX file |
| Inference, Python path | ONNX Runtime (CUDA EP, TensorRT EP) | raw PyTorch, OpenVINO | Decouples robot from training framework |
| Inference, C++ path | Native TensorRT FP16 engine | INT8, torch.compile | Lowest latency; ARX names TensorRT |
| Computer vision | OpenCV | Kornia, scikit-image | Universally requested |
| 3D processing | Open3D (offline), Eigen (runtime), PCL via pcl\_ros (LiDAR) | PCL everywhere | Covers all three names in postings |
| Robot localization | Own C++ EKF, GTSAM iSAM2, robot\_localization baseline | UKF, particle filter, Ceres, g2o | Built, factor graph and standard tool side by side |
| Object tracking | Own C++ constant velocity Kalman filter | UKF, IMM | Writing it is the point |
| Landmarks | AprilTags via apriltag\_ros | ArUco | Known map, clean factor graph |
| Navigation | Nav2, MPPI controller, custom costmap layer | DWB, Regulated Pure Pursuit, TEB | Extend Nav2, not just configure it |
| Arm | UR5e description, simple parallel gripper | Franka FR3 | Well-supported ROS 2 descriptions |
| Motion planning | MoveIt 2 (OMPL, MoveGroupInterface) | MoveIt Task Constructor, cuRobo | Standard in manipulation postings |
| Kinematics library | KDL or Pinocchio | Drake | Jacobian for the custom controller |
| Mission | BehaviorTree.CPP v4, Groot2 | SMACC2, YASMIN | Mission layer; safety stays an explicit FSM |
| Actuation | ros2\_control, gz\_ros2\_control | direct cmd\_vel plugin | Base and arm through the same framework |
| Middleware | Cyclone DDS, measured against Fast DDS; rmw\_zenoh awareness | Connext | The measurement is the point |
| Fleet interface | MQTT (Mosquitto, paho) with a VDA 5050 subset | REST, ROS action | German intralogistics standard |
| C++ tests | GoogleTest, GoogleMock | Catch2 | Named in postings |
| Python tests | pytest | unittest | Standard |
| Integration tests | launch\_testing plus a custom scenario runner | robot framework | ROS 2 native, readable scenarios |
| Static analysis | clang-format, clang-tidy, ASan, UBSan, TSan, gcovr | cppcheck, SonarQube | Free and CI friendly |
| Tracing and profiling | ros2\_tracing with LTTng, perf, heaptrack | Tracy, VTune | ROS 2 native |
| Trajectory evaluation | evo | rpg\_trajectory\_evaluation | Standard ATE and RPE |
| CI | GitHub Actions, GitLab CI mirror | Jenkins | Recruiters read GitHub; employers use GitLab |
| Containers | Docker, Compose profiles dev, sim, ci, runtime | Podman | Reproducibility |
| Docs | Markdown, Mermaid, one C4 diagram, Doxygen headers | Sphinx | Light and readable |

## Architecture

Learned components answer perceptual questions that have no closed-form solution; every decision that affects safety, timing or system state is deterministic, bounded and unit tested. The learned output is an untrusted, timestamped, confidence-weighted input that the deterministic layer must validate.

&#91;embedded content: component architecture · control stack left, perception chain right\]

Perception reaches Nav2 and the safety supervisor only as validated tracks; 3D item poses also feed the pick node, and the fault injector never runs in the production launch.

**Learned versus deterministic**

| Function | Implementation | Why |
| --- | --- | --- |
| Detecting people, pallets, KLT boxes, AMRs, shelf items | Learned (CNN detector) | No practical closed-form algorithm |
| 3D position from a 2D box plus depth | Deterministic (pinhole geometry, TF2) | Exact and testable against ground truth |
| Robot pose | Deterministic (own EKF, GTSAM factor graph) | Covariance you can reason about and gate on |
| Object state and velocity | Deterministic (Kalman filter) | Optimal under its assumptions, unit testable |
| Data association | Deterministic (gating plus assignment) | Tunable and explainable |
| Is an obstacle safety relevant | Deterministic (geometric zones, thresholds) | Must be auditable |
| Speed limiting, stopping, arm interlock | Deterministic FSM | Bounded worst case; never learned |
| Mission sequencing | Deterministic (behavior tree) | Inspectable, versioned in XML |
| Stale or missing sensor data | Deterministic (timeouts, QoS deadline and liveliness) | The failure detector must not be probabilistic |
| Coordinate transforms | Deterministic (TF2) | Mathematics, not inference |
| Base planning and control | Deterministic (Nav2 planner, MPPI) | Mature and explainable |
| Grasp pose generation | Deterministic (geometry from the 3D item pose) | Testable offsets, no hidden failure modes |
| Arm motion planning | Deterministic (MoveIt 2, OMPL, collision checking) | Collision-checked against the planning scene |
| Final approach control | Deterministic (Jacobian controller) | Analytical, tested against finite differences |
| Fault recovery | Deterministic (FSM with hysteresis) | Predictable under partial failure |

**Safety and latency contract** (the README headline, each line enforced by an automated test)

| Property | Target | Enforced by |
| --- | --- | --- |
| Person visible to commanded velocity reduced | p95 under 400 ms | Latency gate in CI, scenario 2 |
| Minimum distance to a person during a crossing | at least 0.8 m | Scenario 2 |
| Perception data older than 300 ms | rejected | Unit test, scenario 5 |
| Camera stops publishing to DEGRADED | under 500 ms | QoS deadline and watchdog, scenario 4 |
| Any hard fault to controlled stop | under 1 s | Scenarios 6, 7, 10 |
| Arm moves only with the base stopped and no person within 1.5 m | always | Safety FSM unit tests, scenario 9 |
| Arm joint freeze during a pick to arm stopped and mission aborted safely | under 1 s | Scenario 10 |
| Robot never navigates on a pose flagged as untrusted | always | Scenario 6 |
| Pick success in nominal conditions | at least 18 of 20 trials | Pick benchmark |
| Collisions in any scenario | zero | Contact sensor assertion in every scenario |

## Week 1 (19 to 25 Oct): C++ core, ROS 2 internals, simulated platform

By Sunday the ROS-free C++ core is tested under sanitizers in CI, and the simulated AMR navigates the warehouse with measured QoS and executor behaviour. Releases: **v0.1-core** (Wed) and **v0.2-platform** (Sun).

| Day | Learn | Build and test | Done when |
| --- | --- | --- | --- |
| Mon 19 Oct | RAII, rule of zero/three/five, `unique_ptr` vs `shared_ptr` vs observing pointers, move semantics, const correctness, `optional`, `variant`, `chrono`, dependency injection | Repo skeleton, `CMakePresets.json`, GPU Docker dev image on `ros:jazzy`; `robot_core` value types (`Pose2D`, `Waypoint`, `Mission`, `Detection`, `TrackedObject`, `Fault`, `MissionResult`, `GraspTarget`, `PickResult`); `IClock` with `SystemClock` and `FakeClock` | `cmake --preset debug && ctest` passes in Docker with 15+ tests; time logic tested with `FakeClock`, no `sleep`; README stub pushed |
| Tue 20 Oct | State machine design, transition tables, `variant` FSM vs enum and switch; `thread`, `mutex`, `condition_variable`, `atomic`, memory ordering, data races | `MissionStateMachine` (IDLE, NAVIGATING, AT\_STATION, PICKING, PLACING, RETURNING, FAULT, RECOVERING, ABORTED); thread-safe `TaskQueue<T>` and `LatestSample<T>` | 25+ FSM tests including every illegal transition; multi-producer test passes 1000 iterations under TSan; Mermaid state diagram drafted |
| Wed 21 Oct | GoogleMock and test doubles, ASan/UBSan/TSan, gdb basics, clang-format and clang-tidy configuration, GitHub Actions caching and artifacts | Mock `INavigationClient` and `IManipulationClient`; mission logic tests against them; sanitizer presets; `.clang-format`, `.clang-tidy`; `ci.yml` (format, tidy, build, ctest, gcovr, artifacts); `docs/01-core-design.md` | Green in all three sanitizer presets; one sanitizer bug documented; coverage above 85%; zero clang-tidy warnings; CI badge green. **Tag v0.1-core** |
| Thu 22 Oct | RMW and DDS, QoS (reliability, durability, history, depth, deadline, liveliness), silent incompatibility, Fast DDS vs Cyclone vs Connext vs rmw\_zenoh; executors and callback groups | `qos_lab`: 3 profiles x 2 payload sizes x 2 RMWs, loss and latency counters; `executor_lab`: one slow and one 50 Hz callback under 4 configurations | Both tables committed with a written explanation of the starvation result |
| Fri 23 Oct | Lifecycle nodes, composition, intra-process communication, interface design; URDF/xacro; ros2\_control (controller manager, resource manager, command vs state interfaces, update loop) | `robot_interfaces` (`DetectedObject3D`, `TrackedObject`, `TrackedObjectArray`, `HealthStatus`, `MissionCommand`, `SafetyState`, plus `LocalizationStatus`, `ItemPose`, `PickCommand`); lifecycle skeleton and component container; base URDF with 2D LiDAR, RGB-D camera, IMU, wheel encoders; gz\_ros2\_control with `diff_drive_controller` and `joint_state_broadcaster` | `ros2 lifecycle set` configure and activate work; two components share one container; `/cmd_vel` drives the robot through ros2\_control; `/odom` and `/joint_states` publish |
| Sat 24 Oct | TF2 tree design, REP 105, extrapolation errors; launch layering, parameter files; launch\_testing; headless Gazebo | Warehouse world (aisles, shelves with items, Stations A and B, dock, drop-off, AprilTags on shelf ends, walking actor); ros\_gz\_bridge; ground truth pose publisher; layered launch (`sim`, `robot`, `perception`, `system`); launch tests: topics within 20 s, TF complete, `/cmd_vel` moves the wheels | `view_frames` clean; RViz shows LiDAR, camera, TF; launch tests pass headless in CI with `gz sim -s` |
| Sun 25 Oct | Nav2 structure, AMCL, slam\_toolbox, costmaps; apriltag\_ros | Map with slam\_toolbox; Nav2 with AMCL; waypoint script with the Nav2 simple commander; apriltag\_ros detections as TF; `docs/02-ros2-architecture.md` (QoS and executor tables, TF tree, node graph); navigation GIF | Robot drives dock to Station A autonomously; tags detected. **Tag v0.2-platform** |

**Measure:** `robot_core` coverage, clean build time, clang-tidy warning count, QoS loss and latency, callback latency per executor configuration.

**Interview answers you now own:** why RAII; `unique_ptr` vs `shared_ptr` in a robot; what a data race is and how TSan finds it; when atomics are enough; why inject a clock; why the core has no ROS dependency; what the RMW does; reliable vs best effort for a camera; transient local durability; silent QoS incompatibility; lifecycle nodes for perception; why composition cuts latency; callback starvation; ros2\_control command vs state interfaces.

**Mistakes to avoid:** ROS types inside `robot_core`; `shared_ptr` out of habit; `sleep` in tests; CI that only works on your machine; reliable QoS on a high-rate camera; one monolithic launch file; ignoring TF timestamp tolerance; a direct `/cmd_vel` Gazebo plugin instead of gz\_ros2\_control.

## Week 2 (26 Oct to 1 Nov): perception in the loop, robot localization

By Sunday a pretrained detector runs live in a lifecycle node with measured latency, and four localization methods are compared against ground truth. Releases: **v0.3-perception** (Wed) and **v0.4-localization** (Sun).

| Day | Learn | Build and test | Done when |
| --- | --- | --- | --- |
| Mon 26 Oct | Detection heads, anchor-free vs anchor-based, objectness vs class confidence, NMS, IoU, mAP; OpenCV for robots (colour conversion, letterbox, normalization); cv\_bridge, CameraInfo | Save 30 Gazebo frames, run pretrained YOLO11n, write `baseline_latency.json`; `perception_core` (no ROS): preprocessing, `DetectorBackend` interface, `TorchBackend`, `MockBackend`, your own NMS | pytest proves preprocessing determinism, shapes, letterbox padding and NMS on hand-built boxes; mean, p50, p95 latency recorded |
| Tue 27 Oct | vision\_msgs, image transport, sensor data QoS, lifecycle callbacks; latency from timestamps, queue discipline, newest-frame processing | `perception` lifecycle node: `Detection2DArray` with the source image stamp, debug image, drop-old-frames policy, `PerceptionDiagnostics` (input FPS, output FPS, inference ms, dropped frames, queue depth); node tests with `MockBackend` in CI | Live detections in RViz while driving; FPS and dropped frames measured with Nav2 running; perception testable without GPU or Gazebo |
| Wed 28 Oct | Precision vs recall and why recall dominates for people; EKF theory (Probabilistic Robotics ch. 3, Stachniss lectures) | Evaluation harness on 50 hand-checked frames (precision, recall for person and box); `docs/03-perception.md` with pipeline diagram and GIF; state, process and measurement models written in `docs/04-localization.md` | Precision and recall table; one failure case screenshotted. **Tag v0.3-perception** |
| Thu 29 Oct | EKF linearization and Jacobians, Mahalanobis gating, numerical stability (Joseph form) | `localization_core` (C++, Eigen, no ROS): state x, y, yaw, v, yaw rate; prediction from wheel odometry and IMU; AprilTag landmark update with a landmark map in YAML; gating; GTests | Covariance stays symmetric and positive definite; known inputs give known trajectories; gated outlier rejected; tests in CI |
| Fri 30 Oct | robot\_localization sensor matrices, odom vs map frame; TUM trajectory format; evo | EKF ROS 2 component publishing map to odom TF and pose with covariance; robot\_localization baseline (odometry, IMU, tag poses); 3 standard runs recorded as rosbags; ground truth export; evo ATE and RPE scripts | Own EKF and baseline run live; first ATE numbers |
| Sat 31 Oct | Factor graphs, iSAM2, IMU preintegration (GTSAM tutorials, prototype in Python first) | C++ `factor_graph_localization`: odometry between factors, IMU preintegration, AprilTag landmark factors, keyframe strategy | Runs in real time; trajectory compared with ground truth |
| Sun 1 Nov | Designing hard evaluation cases | Slip (low-friction patches or injected odometry bias), IMU bias and tag dropout runs; batch evaluation of AMCL, own EKF, robot\_localization and GTSAM; plots and results in `docs/04-localization.md`; Nav2 switched to your EKF, CPU and latency measured | Four-way comparison table with plots; Nav2 navigates on your EKF. **Tag v0.4-localization** |

**Measure:** inference mean, p50, p95, p99; input vs output FPS; dropped frames; precision and recall; ATE and RPE per estimator and scenario; EKF and factor graph CPU time per update.

**Interview answers you now own:** how single-stage detection works; NMS and IoU; why recall matters more for people; why process only the newest frame; why perception is a lifecycle node; what an EKF estimates and what its covariance means; how you chose Q and R; why gate measurements; when an EKF breaks; filter vs factor graph trade-offs; what IMU preintegration solves; ATE vs RPE.

**Mistakes to avoid:** a growing image queue; losing the source timestamp; ROS-coupled detection logic; judging accuracy by looking at pictures; tuning Q and R by feel without writing it down; evaluating localization on a single run; comparing trajectories that are not time-aligned.

## Week 3 (2 to 8 Nov): 3D world model, navigation, safety, fleet interface

By Sunday detections become fused 3D tracks that reshape the costmap, a deterministic safety FSM gates velocity, and a VDA 5050 order starts a full mission tested in two scenarios. Releases: **v0.5-world-model** (Thu) and **v0.6-intelligent-navigation** (Sun).

| Day | Learn | Build and test | Done when |
| --- | --- | --- | --- |
| Mon 2 Nov | Pinhole model, intrinsics K, distortion, depth encodings, back projection, invalid depth; TF2 in C++, `lookupTransform` timeouts, image-time synchronization | `camera_geometry` (C++, no ROS): `pixelToCameraPoint`, median depth over an inner ROI with invalid rejection, GTests; `perception_3d` node publishing `DetectedObject3D` in the map frame at the image stamp, with covariance | Unit tests within 1 mm; a person at a known pose localized within 0.2 m; transform failures handled, not thrown |
| Tue 3 Nov | Point clouds, voxel downsampling, RANSAC ground plane, Euclidean clustering; Open3D vs PCL | Open3D offline analysis of a saved cloud plus a README figure; `lidar_processing` (C++, pcl\_ros) ground removal and clustering; comparison paragraph | Ground-free cloud and clusters in RViz; Open3D figure saved |
| Wed 4 Nov | Kalman filter with constant velocity model, Q and R, Mahalanobis distance, greedy and Hungarian association, track lifecycle | `tracking_core` (C++, no ROS): `KalmanTracker` (4 states), `TrackManager` (birth, confirmation after N hits, coasting, death after M misses, stable IDs); GTests | Covariance shrinks on update; synthetic trajectory recovered within bounded RMSE; outlier gated; one stable track across a 60 s crossing, ID switches counted |
| Thu 5 Nov | Sensor complementarity; track-level vs early fusion | Camera and LiDAR fusion at track level with per-sensor R and a `source` field; `tracking` lifecycle node composed with `perception` and `perception_3d` in one container; integration test against Gazebo ground truth; `docs/05-3d-perception.md` | Position error mean and p95; counts of camera-only, LiDAR-only and fused tracks. **Tag v0.5-world-model** |
| Fri 6 Nov | Layered costmaps, `nav2_costmap_2d` plugin interface and registration, MPPI critics and weights, Nav2 BT and recoveries | `TrackedObstacleLayer`: velocity-aware inflation, wider for uncertain tracks, expiry of stale tracks, unit tests; MPPI tuned for aisle width; recovery set configured | Tracked person appears and disappears in the local costmap within one update; before and after table (time to goal, path length, recoveries, clearance) |
| Sat 7 Nov | BehaviorTree.CPP v4 (ports, blackboard, subtrees, reactive vs sequence), Groot2; safety supervisor patterns, speed zones, hysteresis, ISO 3691-4 vocabulary | `mission_manager` with nodes `Dock`, `NavigateTo`, `WaitForLocalization`, `WaitForClearance`, `PickAtStation` and `PlaceAtStation` (stubs until week 4), `AbortMission`; BT node tests with mocked Nav2 clients; `safety_supervisor` FSM gating `/cmd_vel_nav` to `/cmd_vel` (zone 2 slow, zone 1 stop, hysteresis), `/safety_state`, unit tests | Full mission runs from one BT XML, visible in Groot2; robot slows, stops and resumes without oscillation; every transition logged |
| Sun 8 Nov | MQTT, Mosquitto, VDA 5050 order, state, connection; scenario testing with deterministic seeds | `fleet_interface` (schema validation, mapping to `MissionCommand`, state at 1 Hz, connection on change) with malformed-order tests; scenario runner (world setup, actor scripting, assertions, timeout, JSON results); scenarios 1 and 2; `docs/06-navigation-integration.md`; crossing GIF | A `mosquitto_pub` order starts the mission; both scenarios pass headless in about 2 min each; detection to `/cmd_vel` latency recorded. **Tag v0.6-intelligent-navigation** |

**Measure:** 3D error against ground truth, ID switch count, tracking latency, share of fused tracks, detection to costmap latency, detection to `/cmd_vel` latency (headline number), minimum clearance, mission time with and without the crossing, recovery count.

**Interview answers you now own:** pixel plus depth to a 3D point; what wrong intrinsics do; why robust depth sampling; why transform at the image timestamp; what TF2 buys you; what camera and LiDAR each contribute; Kalman covariance and gating; when constant velocity breaks; Nav2 end to end; what a costmap layer does; behavior tree vs FSM, with both in your project; why safety is its own node; what VDA 5050 is for.

**Mistakes to avoid:** depth from a single centre pixel; transforms looked up at "now"; untracked Q and R tuning; churning track IDs; safety logic inside the behavior tree; stale obstacles that never expire; MPPI tuning without before and after numbers; attempting the full VDA 5050 spec instead of a documented subset.

## Week 4 (9 to 15 Nov): manipulation

By Sunday the AMR carries a UR5e arm, picks detected shelf items with MoveIt 2 and a custom Cartesian controller, and completes a VDA 5050 fetch order. Release: **v0.7-manipulation** (Sun).

| Day | Learn | Build and test | Done when |
| --- | --- | --- | --- |
| Mon 9 Nov | Modern Robotics ch. 4 to 6: forward kinematics, velocity kinematics, space vs body Jacobian, singularities, damped least squares | Kinematics notes in `docs/`; UR5e and a simple parallel gripper mounted on the base in xacro, with collision geometry | Mobile manipulator spawns in Gazebo; TF includes arm frames; launch tests updated |
| Tue 10 Nov | ros2\_control for arms (joint\_trajectory\_controller, gripper controller, controller switching); MoveIt 2 architecture (move\_group, OMPL, kinematics plugins) | Arm and gripper controllers in gz\_ros2\_control; MoveIt Setup Assistant (SRDF, planning groups, KDL kinematics, named poses) | Plan and execute to named poses from RViz in simulation |
| Wed 11 Nov | Planning scene, collision and attached objects; MoveGroupInterface C++ API | Shelves added to the planning scene; combined Nav2 and MoveIt launch; arm stowed while driving; `IManipulationClient` adapter over MoveGroupInterface with mock-based tests | Robot drives with a stowed arm and plans around shelves |
| Thu 12 Nov | Grasp pose generation, approach and retreat; Gazebo DetachableJoint | `perception_3d` publishes `ItemPose` for shelf items; grasp generator (pre-grasp, grasp, lift) with unit tests; C++ pick node; DetachableJoint attach and detach; attached object in the planning scene | First successful pick in simulation |
| Fri 13 Nov | KDL or Pinocchio Jacobian API; pluginlib; `controller_interface` lifecycle and real-time-safe `update()` | `cartesian_velocity_controller` plugin: twist to joint velocities through a damped least squares Jacobian inverse, joint position and velocity limits; GTests: Jacobian against finite differences, damping near singularities | Controller loads through the controller\_manager; tests green |
| Sat 14 Nov | Controller switching, visual servoing basics, interlocks for arm motion | Switch from trajectory to Cartesian control for the final approach; real `PickAtStation` and `PlaceAtStation` BT nodes; `safety_supervisor` arm interlock (base stopped, no person within 1.5 m) with tests; VDA 5050 pick and drop actions mapped to the BT | A VDA 5050 order with a pick action runs the full fetch mission |
| Sun 15 Nov | Designing success-rate experiments | 20 pick trials with failure reasons logged; fix the top failure; scenario 9 (nominal pick and deliver); `docs/07-manipulation.md`; pick GIF | At least 18 of 20 picks succeed; scenario 9 passes headless. **Tag v0.7-manipulation** |

**Measure:** pick success rate, planning time, execution time, controller update time and jitter, final approach error against ground truth.

**Interview answers you now own:** forward kinematics; what the Jacobian maps; singularities and damped least squares; the MoveIt 2 pipeline; what the planning scene is for; why trajectory control in free space and Cartesian control for the approach; real-time rules inside a controller `update()`; why the arm interlock lives in the deterministic supervisor; what DetachableJoint simplifies and what it hides.

**Mistakes to avoid:** relying on friction grasps in simulation; allocating memory in the controller update; ignoring joint limits; planning without collision objects; moving the arm while the base drives; reporting one lucky pick instead of a success rate.

## Week 5 (16 to 22 Nov): synthetic data, fine tuning, deployment

By Sunday a detector fine-tuned on auto-labeled synthetic data runs through TensorRT, both from Python and from a native C++ node, with a four-backend latency comparison. Release: **v0.8-optimized-perception** (Sun).

| Day | Learn | Build and test | Done when |
| --- | --- | --- | --- |
| Mon 16 Nov | Dataset design, class taxonomy, split hygiene; projecting 3D boxes into the image, visibility, occlusion, truncation | Seeded capture node (RGB, depth, CameraInfo, camera pose, all model poses); `auto_labeler.py` projecting 3D boxes with a visibility filter; Gazebo bounding box camera; per-box IoU cross-check between both labelling methods; split-by-episode test; QA overlay tool | 300 frames reproducible from a seed; 200 labels spot-checked; mean IoU between methods and label error rate recorded |
| Tue 17 Nov | Domain randomization (lighting intensity and colour temperature, textures, placement, camera height and pitch, distractors); segmentation masks | Randomized capture loop; about 3,000 train, 400 validation, 300 test images split by episode; classes person, pallet, klt\_box, amr plus 3 to 5 shelf item classes; segmentation masks saved; dataset and seed published as a release asset | Dataset built; class balance charted |
| Wed 18 Nov | Transfer learning, layer freezing, learning rate schedules, augmentation, overfitting curves, mAP@0.5 vs mAP@0.5:0.95, PR curves | Fine-tune YOLO11n from COCO weights; training curves; per-class evaluation of baseline vs fine-tuned on the held-out set; randomization ablation (train without randomization) | Comparison table with per-class gains and one honest weakness; ablation result |
| Thu 19 Nov | ONNX graphs and opsets, ONNX Runtime execution providers, TensorRT engine building, FP16, workspace, warm-up, static vs dynamic shapes; CUDA host and device memory, streams, batching | ONNX export with a parity test against PyTorch; `OnnxRuntimeBackend` (CUDA EP and TensorRT EP) behind `DetectorBackend`; trtexec FP16 engine build script with caching | Parity within tolerance; backend equivalence test green; engine cached, not rebuilt on start |
| Fri 20 Nov | TensorRT C++ runtime API, CUDA buffers, GPU vs CPU pre and post processing | `perception_trt` C++ lifecycle node: preprocessing, inference, NMS, `Detection2DArray` with source stamps, engine from cache; unit tests for pre and post processing | Node runs live with the same interface as the Python node |
| Sat 21 Nov | Benchmark hygiene: warm-up, percentiles, pinned configuration, repeated runs | `benchmarks/inference_benchmark.py`: PyTorch vs ORT CUDA vs ORT TensorRT FP16 vs native C++ TensorRT, batch 1, 200 runs after warm-up, 3 repeats; VRAM, GPU utilization, model size, FP16 accuracy delta; charts | Latency table (p50, p95, p99) and accuracy chart committed |
| Sun 22 Nov | Reviewing the backend abstraction | Robot switched to the fastest backend; scenarios 1, 2 and 9 re-run; `docs/08-model-and-deployment.md` (dataset method, randomization parameters, training config, accuracy and latency tables, FP16 trade-off, Ultralytics licensing separation, synthetic-only statement) | Behaviour unchanged, latency improved. **Tag v0.8-optimized-perception** |

**Measure:** dataset size and class balance, label error rate, labelling method agreement, baseline vs fine-tuned mAP per class, ablation delta, latency per backend, model size, VRAM, GPU utilization, FP16 accuracy delta.

**Interview answers you now own:** training vs inference; how you spotted overfitting; why transfer learning works; why synthetic data and its limits; what domain randomization buys (with your ablation number); why split by episode; FP32 vs FP16 with your measurements; why ONNX and why the robot never imports PyTorch; what TensorRT does to a graph; why the first inference is slow; native TensorRT vs ONNX Runtime trade-offs.

**Mistakes to avoid:** letting data work eat the week; random frame splits; mAP without the synthetic-only caveat; rebuilding the engine on every start; changing the detector interface and breaking earlier weeks; benchmarking without warm-up.

## Week 6 (23 to 29 Nov): reliability, performance, release

By Sunday ten fault-tolerance scenarios pass flake-free in CI, a stage-by-stage latency budget is enforced by a performance gate, and the repository is presentable. Releases: **v0.9-reliability** (Wed) and **v1.0** (Sun).

| Day | Learn | Build and test | Done when |
| --- | --- | --- | --- |
| Mon 23 Nov | FMEA, safe states, fail-safe vs fail-operational, detection budgets; watchdogs, heartbeats, staleness, QoS deadline and liveliness, diagnostic\_msgs | `docs/09-fmea.md` written before any code: 12+ failure modes (camera, inference, LiDAR, localization, tags, perception crash, wheel joint, arm joint, gripper, MQTT link) with detection method, time budget, response and recovery; `SafetyState` enum; `health_monitor` (per-topic freshness, deadline and liveliness callbacks, node liveness, `/health`, diagnostics aggregator) | DEGRADED within 500 ms of a stalled camera, asserted in a unit test |
| Tue 24 Nov | Out-of-band fault injection; degraded modes, LiDAR-only fallback, hysteresis, controlled stop vs emergency stop | `fault_injector` services: `drop_camera`, `delay_inference`, `corrupt_image`, `drop_lidar`, `degrade_localization`, `drop_tags`, `freeze_joint` (wheel or arm), `kill_node`; off by default and excluded from the runtime launch; degraded handling in `safety_supervisor` and `mission_manager` (LiDAR-only at reduced speed, stop and relocalize, arm stop and safe abort, controlled stop and hold) | Every fault is one service call and reversible; the robot survives each with no collision or deadlock |
| Wed 25 Nov | Assertions that fail for the right reason; flakiness from physics, timing and startup races | Scenarios 3 to 8 and 10; whole suite run 10 times in a row; `scenarios.yml` nightly and on tags; `docs/10-reliability.md` (FMEA table, state diagram, detection latency table, scenario results) | 10 scenarios, 10 consecutive green runs, zero flakes, zero collisions. **Tag v0.9-reliability** |
| Thu 26 Nov | ros2\_tracing and LTTng; latency budgets, percentiles, jitter, throughput vs latency | 60 s trace during scenario 2 analysed in a notebook; source image stamp carried through every stage to `/cmd_vel`; `/perf_metrics` with per-stage durations | Stage-by-stage latency distribution from image capture to velocity change |
| Fri 27 Nov | perf, flame graphs, heaptrack or massif; how executors and QoS affect latency | Profile `tracking`, `perception_3d`, `perception_trt`; fix the largest hotspot; latency under 2 executor x 2 QoS configurations; jitter of the perception loop, diff drive controller and Cartesian arm controller | Before and after CPU numbers; configuration table against the 400 ms p95 budget; jitter table |
| Sat 28 Nov | Choosing regression thresholds; reproducibility; real-time vocabulary (hard, firm, soft, priority inversion, what PREEMPT\_RT would change) | `performance.yml` gate, proven to fail with an injected 200 ms delay; `release.yml`; runtime image; Compose profiles dev, sim, ci, runtime; `Makefile` (build, test, sim, demo, scenarios, bench); pinned versions and seeds; GitLab CI mirror | Gate goes red, then green again; a fresh clone reaches a running demo in under 10 minutes |
| Sun 29 Nov | 60 min on Isaac Sim and Isaac Lab architecture; Jazzy to Lyrical migration notes | Final README (badges, video, headline numbers, architecture and C4 diagrams, benchmark tables, future work on Physical AI, VLA and RL); `docs/11-performance.md`, `docs/12-limitations.md`, interview notes; demo video; CV bullets from real numbers | **Tag v1.0** |

**Measure:** detection latency per failure mode against its budget, time to safe state, time to recover, collisions (zero), suite runtime and flake rate, per-stage p50, p95, p99 and jitter, CPU per node, memory, GPU utilization.

**Interview answers you now own:** what happens when perception fails; three ways to detect stale data and when each fits; degraded state vs emergency stop; why the fault detector is deterministic; why hysteresis; why fault injection stays out of band; how to test without a robot and what that does not prove; how you killed flakiness; how you measure callback latency; what tracing shows that logs cannot; a case where throughput and latency traded off; why report p95 and p99; how the performance gate works.

**Mistakes to avoid:** writing the injector before the FMEA; fault code inside production nodes; equal fault and recovery thresholds; accepting a flaky scenario; reporting mean latency only; measuring on a loaded desktop; a README without diagram and video; calling a simulation project "real time" or "production ready".

## Testing and fault injection

Testing starts on day 1 and runs at five levels; anything slow, nondeterministic or external is mocked in unit tests and exercised for real once at system level.

**Test pyramid**

| Level | Tool | Target count | Runtime | Runs in CI | Covers |
| --- | --- | --- | --- | --- | --- |
| C++ unit | GoogleTest, GoogleMock | 150+ | under 10 s | every push | geometry, filters, factor graph helpers, association, state machines, containers, kinematics, controller math |
| Python unit | pytest | 50+ | under 15 s | every push | preprocessing, NMS, auto-labeller, label cross-check, scenario assertions, VDA 5050 parsing |
| Node level | pytest with rclpy, GoogleTest with rclcpp | 25+ | under 90 s | every push | adapters with mocked backends, Nav2 and MoveIt clients; no Gazebo |
| Launch integration | launch\_testing | 10+ | under 4 min | every push | bringup, topics, lifecycle transitions, TF tree, controllers loaded |
| SiL scenarios | scenario runner, headless Gazebo | 10 | about 20 min | nightly and on tags | full behaviour including faults |
| Benchmarks | pytest-benchmark, custom | 6 | about 8 min | nightly | inference, end-to-end latency, tracking, localization, pick rate, controller jitter |
| Performance gate | custom | 1 | about 3 min | every push to main | fails the build if p95 latency exceeds budget |

**Mock vs real**

| Mock in unit and node tests | Run for real at system level |
| --- | --- |
| `MockBackend` with scripted detections | the detector in scenarios and benchmarks |
| `FakeClock` | real time in scenarios |
| Nav2 action clients | full Nav2 |
| MoveGroupInterface behind `IManipulationClient` | full MoveIt 2 |
| Injected TF transforms | full TF tree |
| MQTT broker | Mosquitto container in the integration test |
| Synthetic depth arrays with known geometry | Gazebo depth camera |
| Synthetic odometry, IMU and tag streams | Gazebo sensors for localization |
| GPU (mock backend needs none) | your RTX for training and benchmarks |

**The ten scenarios** (each with a fixed seed, a hard timeout and a zero-collision assertion)

| # | Scenario | Setup | Pass criteria |
| --- | --- | --- | --- |
| 1 | Nominal mission | MQTT order: dock, Station A, Station B, dock | SUCCEEDED; under 180 s sim time; `/health` OK throughout; no recoveries |
| 2 | Person crossing | Actor crosses 4 m ahead at 1.2 m/s | Confirmed track before 3 m; speed below 0.3 m/s within p95 400 ms of first detection; clearance at least 0.8 m; mission completes |
| 3 | Blocked route | Pallet spawned across the committed corridor | In costmap within 1 s; replan within 10 s; goal reached by another route; at most 2 recoveries |
| 4 | Camera stops | Camera frames halted mid-leg with a person present | DEGRADED within 500 ms; LiDAR-only at 0.3 m/s cap; completes or aborts cleanly; full mode back within 2 s, no oscillation |
| 5 | Inference timeout | Inference delayed to 900 ms per frame | Detections older than 300 ms rejected; DEGRADED; no phantom obstacle beyond 2 s; speed limited |
| 6 | Localization degradation | Wheel slip, half LiDAR field of view and tag dropout inflate covariance | Detected within 1 s of the threshold; controlled stop within 1 s; relocalization attempted; never navigates on an untrusted pose |
| 7 | Perception crash | Perception process killed mid-mission | Liveliness loss within 500 ms; SAFE\_STOP within 1 s; lifecycle restart; nominal within 15 s; mission resumes |
| 8 | LiDAR dropout with person | LiDAR returns dropped with a person in the path | Camera-only track kept; DEGRADED; stops before the person; clearance at least 0.8 m |
| 9 | Pick and deliver (added) | VDA 5050 order with pick and drop actions | Item picked and delivered; arm moves only with base stopped and no person within 1.5 m; mission SUCCEEDED |
| 10 | Arm joint freeze (added) | Arm joint frozen through ros2\_control during a pick | Detected within 500 ms; arm stopped within 1 s; mission aborts to a safe hold; base never moves with the arm extended |

Each scenario writes a JSON result; CI aggregates them into a markdown table committed to `docs/`.

**Fault injection rules**

1. Out of band: a separate node with a service API, never compiled into production nodes or included in the runtime launch.
2. Reversible: every fault can be cleared, so recovery is tested too.
3. Derived from the FMEA, not from what is easy to inject.
4. Detection budgets written before implementation, then met and measured.
5. Hysteresis everywhere: entry and exit thresholds differ, and the oscillation case has its own test.

## Performance benchmarks

Twelve measurements carry the engineering story; each is reported as percentiles under a stated load, never as a bare mean.

| # | Measurement | Report |
| --- | --- | --- |
| 1 | End-to-end reaction latency, image capture to first reacting `/cmd_vel` (headline) | p50, p95, p99 |
| 2 | Per-stage breakdown: preprocessing, inference, NMS, 3D localization, TF, tracking, costmap, controller | p50 and p95 per stage |
| 3 | Inference across PyTorch, ORT CUDA, ORT TensorRT FP16, native C++ TensorRT | p50, p95, p99 at batch 1 after warm-up, 200 runs |
| 4 | Perception throughput: input FPS, output FPS, dropped frames | under Nav2 and Gazebo load |
| 5 | Fault detection latency per FMEA failure mode | actual vs budget |
| 6 | CPU per node, total memory, GPU utilization | nominal scenario |
| 7 | Jitter of perception loop, diff drive controller, Cartesian arm controller | standard deviation and p99 of the period |
| 8 | 3D object localization against ground truth | mean and p95 |
| 9 | Robot localization ATE and RPE per estimator and scenario (added from October) | table and plots |
| 10 | Pick success rate, planning and execution time (added from October) | rate over 20 trials, p50 and p95 times |
| 11 | Detection accuracy: baseline vs fine-tuned mAP, randomization ablation, FP16 delta | per class |
| 12 | p95 end-to-end latency across release tags | trend chart in the README |

**Skip:** network throughput, GPU power draw, disk I/O, serialization size alone, and anything not tied to a decision you made.

**Measurement hygiene**

- Warm up first and say so.
- Percentiles only; a 40 ms mean with a 400 ms p99 is a robot that sometimes hits someone.
- Pin world, seed, scenario and machine; close everything else; record hardware and software versions in the report header.
- Run each benchmark at least three times and report the variation.
- State the load: "45 ms p95 with Nav2 and Gazebo running".
- Keep dated JSON in `benchmarks/results/`.

**The performance gate:** a CI job runs a fixed scenario and fails if p95 end-to-end latency exceeds 400 ms. Prove it by injecting a 200 ms delay, watching CI go red, reverting, and screenshotting both.

## Repository structure and CI/CD

ROS-free domain libraries sit beside thin ROS adapters, and everything that never runs on the robot lives in `tools/`. That boundary is the strongest structural signal in the repo.

```
warehouse-amr-stack/
  README.md  LICENSE (Apache-2.0)  CHANGELOG.md  Makefile  CMakePresets.json
  .clang-format  .clang-tidy  .gitignore  .dockerignore  .gitlab-ci.yml
  docs/
    00-overview.md                 C4 context and container diagrams
    01-core-design.md   02-ros2-architecture.md   03-perception.md
    04-localization.md  05-3d-perception.md        06-navigation-integration.md
    07-manipulation.md  08-model-and-deployment.md 09-fmea.md (before code)
    10-reliability.md   11-performance.md          12-limitations.md
    13-interview-notes.md  images/
  src/
    robot_core/                    C++17, no ROS
    camera_geometry/               C++17, no ROS
    tracking_core/                 C++17, no ROS (Kalman, association)
    localization_core/             C++17, no ROS (robot pose EKF)
    perception_core/               Python, no ROS (preprocess, backends)
    robot_interfaces/              msg, srv, action
    warehouse_description/         URDF/xacro: base, sensors, UR5e, gripper, ros2_control
    warehouse_sim/                 worlds, items, tags, actors
    warehouse_bringup/             layered launch and parameters
    perception/                    lifecycle adapter around perception_core
    perception_trt/                native C++ TensorRT node
    perception_3d/                 adapter around camera_geometry
    lidar_processing/              pcl_ros ground removal, clustering
    tracking/                      adapter around tracking_core
    localization/                  EKF adapter, factor_graph_localization (GTSAM)
    nav2_tracked_obstacle_layer/   custom costmap plugin
    manipulation/                  MoveIt config, grasp generator, pick node
    cartesian_velocity_controller/ ros2_control plugin
    mission_manager/               BehaviorTree.CPP app and BT XML
    safety_supervisor/             deterministic FSM, base and arm gating
    health_monitor/                watchdogs, deadline, liveliness, diagnostics
    fault_injector/                OUT OF BAND, excluded from runtime launch
    fleet_interface/               MQTT and VDA 5050 subset
    qos_lab/  executor_lab/
  tools/
    dataset_tools/                 capture, auto-labeller, randomization, QA
    training/                      fine-tuning, evaluation, ONNX export, TRT build
    analysis/                      evo scripts, trace notebooks, reports
  test/integration/  test/fixtures/
  scenarios/runner/  scenarios/s01 ... s10  scenarios/results/
  benchmarks/  benchmarks/results/
  docker/Dockerfile.dev (GPU)  Dockerfile.runtime  Dockerfile.ci (headless)
  docker-compose.yml               profiles: dev, sim, ci, runtime
  .github/workflows/ci.yml  scenarios.yml  performance.yml  release.yml
```

**Workflows**

| Workflow | Trigger | Jobs | Target |
| --- | --- | --- | --- |
| `ci.yml` | every push and PR | format checks, C++ domain build and GTest, clang-tidy, ASan and UBSan (TSan scheduled), pytest, ROS workspace build, node tests, launch\_testing headless, gcovr coverage | under 12 min |
| `scenarios.yml` | nightly, tags, manual | 10 SiL scenarios in headless Gazebo, aggregated results table, failure videos as artifacts | under 30 min |
| `performance.yml` | push to main, nightly | fixed-scenario latency benchmark, p95 gate, updated trend chart | under 8 min |
| `release.yml` | tag push | runtime image, release notes from the changelog, dataset and weights as release assets | under 10 min |

**Practical points**

- Build `Dockerfile.ci` from `ros:jazzy-ros-base` once and cache it; never install ROS per run.
- Headless Gazebo: `gz sim -s`, with `LIBGL_ALWAYS_SOFTWARE=1` if needed; budget time for it in week 1.
- Free runners have no GPU, so the mock backend keeps perception tests in CI; GPU jobs run on your machine as a self-hosted runner, mentioned in the README.
- README badges: build, scenarios, coverage, latency p95.
- Mirror to GitLab with `.gitlab-ci.yml` running the core jobs.
- Conventional commits, a generated changelog, annotated tags with a one-paragraph message.

## What simulation proves and what it does not

The project validates architecture, algorithms, integration, failure handling and testing method; it does not validate sensor realism, timing on real-time hardware or perception on real imagery. Put this sentence in the README and say it in interviews.

**Fully demonstrated:** ROS 2 internals, lifecycle, composition, executors, QoS, TF2, ros2\_control for base and arm, Nav2 configuration and plugins, MoveIt 2 planning, behavior trees, perception pipeline design, inference integration, 3D geometry, filtering, factor graphs, tracking, fusion, mission orchestration, fleet interfaces, fault detection and degraded modes, automated testing at every level, CI/CD, containers, profiling and tracing, architecture.

**Partially demonstrated** (state these in `docs/12-limitations.md`)

| Area | Simulation gives | Missing | Substitute |
| --- | --- | --- | --- |
| Real-time behaviour | latency and jitter methodology | real-time kernel, priority tuning; Gazebo perturbs timing | measure jitter and p99; name PREEMPT\_RT and CPU isolation as next steps |
| Sensor noise | ideal or lightly noised sensors | intensity effects, blur, rolling shutter, depth holes, multipath | inject noise, blur, exposure shifts, dropped returns; call them approximations |
| Perception accuracy | mAP on synthetic images | real-world performance | state it bluntly; describe the sim-to-real evaluation you would run |
| Localization | ATE against perfect ground truth | real wheel slip, IMU drift, tag misdetections | inject slip, bias and dropout; name real datasets as next step |
| Grasping | planning and control on a rigid attach | contact physics, slip, compliance | DetachableJoint stated openly; force control as future work |
| Hardware drivers | interface understanding | drivers, CAN, SPI, device trees | point to your real-robot work experience separately |
| Edge deployment | ONNX and TensorRT workflow, FP16 | Jetson thermals, power modes, DLA | workflow transfers; platform tuning does not |
| Calibration | intrinsics and extrinsics model | checkerboard and hand-eye calibration | say you understand it and have not done it physically |
| Safety hardware | software safety logic | safety-rated scanners, e-stop circuits, PL assessment | use ISO 3691-4 vocabulary; the supervisor is application software, not a certified function |

## Demo video and presenting the project

A 6 to 7 minute video opens with the numbers, shows a fleet order turning into a safe fetch mission, and ends on the test suite rather than the robot.

| Time | Act | What is on screen |
| --- | --- | --- |
| 0:00 to 0:30 | The claim | One-sentence description and five numbers: p95 reaction latency, minimum clearance, scenario count, mAP gain, pick success rate |
| 0:30 to 1:15 | Order from the fleet | `mosquitto_pub` sending a VDA 5050 order with a pick action; RViz; Groot2 tree ticking |
| 1:15 to 2:00 | Localization | EKF and factor graph trajectories against ground truth; covariance ellipse shrinking at a tag |
| 2:00 to 2:45 | Perception and world model | Annotated camera view, tracks with covariance, costmap layer reacting; pallet and KLT box classes COCO lacks |
| 2:45 to 3:30 | The safety event | Worker crosses: detection, confirmed track, costmap inflation, SLOW then STOP, velocity plot; frozen frame with latency and clearance |
| 3:30 to 4:30 | The pick | Base stops, arm interlock released, MoveIt plan, Cartesian final approach, item delivered |
| 4:30 to 5:30 | Failure and recovery | Camera fault to LiDAR-only mode and clean recovery; perception node killed, safe stop, restart, resume; arm joint freeze, safe abort |
| 5:30 to 6:15 | The engineering | Green CI, 10-scenario table, latency budget, mAP and backend charts, localization table, p95 trend across tags |
| 6:15 to 6:45 | Honesty and reproduction | Limitations slide and the two commands that reproduce the demo |

**Presenting it**

- Pin the repo with a one-line description using the words employers search for: ROS 2, C++, perception, sensor fusion, Nav2, MoveIt 2, simulation, CI.
- Put the video and headline numbers at the top of the README; reviewers spend under 90 seconds.
- On the CV: one project entry with three to five measured bullets, not a technology list.
- On LinkedIn: a post per release with its GIF and one number; at v1.0, the 60-second safety event plus the engineering act, with a short note on the learned vs deterministic split.
- In applications, name the most relevant part: SiL suite for test and simulation roles, fusion and localization for ARX and Magazino, perception and TensorRT for the KUKA AI program and perception roles, manipulation for Agile Robots, Franka and NEURA, VDA 5050 for intralogistics, fault handling for defense and automotive.
- Prepare a 10-minute walkthrough: architecture, one file you are proud of (Kalman filter, EKF, safety supervisor or Cartesian controller), one test file, the latency table. Practise it aloud.
- Post once in the ROS Discourse projects category.
- Start applying from v0.4 onward; don't wait for v1.0.

## Interview preparation

Answer every question by pointing to something you built or measured; keep the answers in `docs/13-interview-notes.md` as you go.

- **Modern C++:** RAII and what breaks without it; `unique_ptr` vs `shared_ptr`; move semantics; rule of zero, three, five; const correctness; `optional` vs sentinels; data races and TSan; when atomics suffice; priority inversion; debugging memory errors; what never belongs in a real-time path; why the domain libraries have no ROS dependency.
- **ROS 2:** what the RMW abstracts; reliable vs best effort with your choice; transient local; silent QoS incompatibility; deadline and liveliness in your health monitor; lifecycle nodes; composition and your measured latency gain; executors, callback groups and starvation; TF2 time and extrapolation errors; ros2\_control interfaces and the update loop; Fast DDS vs Cyclone vs rmw\_zenoh; Humble vs Jazzy vs Lyrical.
- **Computer vision and deep learning:** single-stage detection; IoU; NMS; precision vs recall for people; mAP@0.5 vs mAP@0.5:0.95; what confidence scores are not; training vs inference; overfitting in curves; transfer learning; synthetic data limits; domain randomization with your ablation; split by episode; FP32 vs FP16; why ONNX; what TensorRT does; why the first inference is slow; native TensorRT vs ONNX Runtime.
- **3D perception and tracking:** pixel plus depth to 3D; wrong intrinsics; robust depth sampling; transforms at the image timestamp; Kalman state and covariance; choosing Q and R; Mahalanobis gating; when constant velocity breaks; track-level vs early fusion; what camera and LiDAR each add; Open3D vs PCL.
- **Robot localization:** EKF linearization and its failure modes; landmark updates; filter vs factor graph; iSAM2 incremental smoothing; IMU preintegration; ATE vs RPE; how slip and tag dropout showed in your results; when to trust a pose.
- **Manipulation:** forward kinematics; the Jacobian; singularities and damped least squares; the MoveIt 2 pipeline and planning scene; trajectory vs Cartesian control; real-time rules in a controller update; why the arm interlock is deterministic; what DetachableJoint hides.
- **Navigation and behaviour:** Nav2 end to end; global vs local planning; costmap updates and your layer; MPPI vs DWB; behavior tree for missions and FSM for safety; how the supervisor gates base and arm; VDA 5050 and fleet interoperability.
- **Reliability and testing:** perception failure behaviour; three staleness detectors; degraded state vs emergency stop; hysteresis; out-of-band injection; testing without a robot and its limits; flakiness; what HIL would add; extending toward ISO 3691-4.
- **Performance:** measuring callback latency; tracing vs logging; a throughput vs latency trade-off you hit; QoS effects; your jitter numbers; why p95 and p99; the performance gate.
- **Judgement questions:** what simulation can and cannot validate; why Gazebo over Isaac Sim (minimum GPU spec, LTS pairing, headless CI, time); where you deliberately avoided machine learning and why (practise two minutes); what you would do first with real hardware (calibration, sensor characterization, timing on target, perception on real images); the weakest part (perception never tested on real images, constant velocity model, rigid grasp attach); scaling to 20 robots (traffic management, VDA 5050 master control, deconfliction, shared maps, DDS discovery limits and Zenoh).

## CV: skills to claim and example bullets

Claim only what you built and measured; German interviewers probe every line.

**Claim confidently after v1.0:** modern C++17 for robotics (ownership, move semantics, concurrency); CMake; GoogleTest, GoogleMock, pytest, launch\_testing; sanitizers, clang-tidy, clang-format; ROS 2 lifecycle, composition, executors, callback groups; QoS and DDS behaviour (measured); custom interfaces; TF2 in C++; ros2\_control for mobile base and arm, including a custom controller plugin; Nav2 and custom costmap plugins; MoveIt 2 motion planning and pick pipelines; robot kinematics and Jacobian-based control; BehaviorTree.CPP; Gazebo Harmonic world authoring; OpenCV; PyTorch fine-tuning; detection evaluation; synthetic data with automatic labels and domain randomization; ONNX, ONNX Runtime, TensorRT FP16 in Python and C++; camera geometry; Open3D; EKF and Kalman filters written from scratch; GTSAM factor graph localization; multi-object tracking; track-level sensor fusion; trajectory evaluation; SiL scenario testing; fault injection and degraded modes; health monitoring; ros2\_tracing and latency budgeting; Docker; GitHub Actions and GitLab CI; MQTT and VDA 5050; technical documentation.

**Introductory exposure, say so:** PCL (one focused task); CUDA concepts without kernels; MPPI tuning; robot\_localization at configuration level; rmw\_zenoh and Bazel (aware, not used); Isaac Sim and Isaac Lab (architecture understood, no hands-on use); ISO 3691-4 and functional safety vocabulary; RL and robot foundation models (informed opinion). Phrase it as "working knowledge, applied in a personal project" vs "familiar with concepts, no production use".

**Needs physical hardware before claiming:** real-time Linux and PREEMPT\_RT; driver development; CAN, CANopen, SPI, I2C, UART in production; Jetson deployment; physical and hand-eye calibration; real sensor noise characterization; HIL testing; real grasping and force control; safety hardware integration; real-world perception accuracy; fleet field operations. Your real-robot work stays as work experience, separate from the project.

**Example bullets** (fill brackets with measured numbers only)

> **Autonomous Warehouse Mobile Manipulator Stack**, simulation based, ROS 2 Jazzy, modern C++17, Gazebo Harmonic (github.com/...)

1. Designed a ROS 2 perception-to-navigation pipeline in which a fine-tuned CNN detector, RGB-D back projection and a self-implemented Kalman tracker feed a custom Nav2 costmap layer, reaching a measured p95 reaction latency of **\[X\] ms** from image capture to commanded deceleration.
2. Built a fault-tolerant architecture separating learned perception from deterministic safety logic, with QoS deadline and liveliness health monitoring, out-of-band fault injection for **\[12\]** failure modes and defined degraded states, detecting camera loss within **\[X\] ms** and reaching a safe stop within **\[X\] ms**.
3. Developed an automated SiL suite of **10** end-to-end scenarios (crossing pedestrian, blocked route, sensor loss, inference timeout, localization degradation, node crash, LiDAR dropout, pick and deliver, arm joint failure) running headless in CI alongside **\[N\]** unit and integration tests and a p95 latency regression gate.
4. Implemented robot localization with a self-written EKF and a GTSAM iSAM2 factor graph fusing wheel odometry, IMU preintegration and AprilTag landmarks, reducing ATE from **\[X\]** to **\[Y\]** cm against robot\_localization and AMCL baselines under slip and landmark dropout.
5. Integrated a UR5e arm through ros2\_control and MoveIt 2 with a custom Jacobian-based Cartesian velocity controller, achieving **\[18+/20\]** successful picks of detected items, triggered by VDA 5050 orders.
6. Generated a **\[3,000\]**-image domain-randomized synthetic dataset with automatic labels, fine-tuned YOLO on intralogistics classes (mAP@0.5 **\[X\]** to **\[Y\]**), and deployed it with TensorRT FP16 in a native C++ node, cutting p95 inference from **\[X\]** to **\[Y\]** ms.
7. Engineered production-style delivery: ROS-free unit-tested C++ libraries behind lifecycle-managed adapters, containerized reproducible builds, and a four-workflow CI pipeline with static analysis, sanitizers, coverage and nightly scenario and performance jobs.

Lead with bullet 2 for safety-focused employers (ARX, Bosch, ZF, Magazino), bullet 4 for localization roles, bullet 5 for manipulation companies, and bullet 1 or 6 for perception roles.

## After the six weeks

From December on, convert the project into interviews, then deepen whichever specialization the interviews point to.

**Immediately (December and January)**

1. Apply, targeting Munich, Stuttgart and Berlin first; you already started at v0.4.
2. Keep German going toward certified B1 by mid 2027, then B2.
3. Publish the blog and ROS Discourse posts.
4. Evaluate your fine-tuned model on a public real-image indoor or warehouse dataset, and run your localization stack on a public real robot dataset. Document the drop honestly.

**Next specialization, chosen by where interviews come from**

| Interviews at | Learn next |
| --- | --- |
| Intralogistics and AMR (Magazino, idealworks, NODE, Jungheinrich, KION) | full VDA 5050, multi-robot traffic management, Open-RMF fleet coordination, fleet simulation, ISO 3691-4 in depth, WMS integration |
| Defense and field autonomy (ARX, Helsing, Quantum Systems, Rheinmetall) | GNSS-denied localization, LiDAR-inertial odometry (FAST-LIO, LIO-SAM), traversability, SROS2, Rust, Bazel, mission planning at scale |
| Automotive and AD (Bosch, Continental, ZF, CARIAD) | ISO 26262 and SOTIF, AUTOSAR Adaptive, radar signal processing, large-scale tracking, OpenSCENARIO and ASAM validation |
| Perception roles | point cloud 3D detection (PointPillars, CenterPoint), BEV perception, segmentation, multi-camera calibration, deeper CUDA and TensorRT plugins |
| Manipulation and humanoids (NEURA, Agile Robots, Franka) | dynamics, force and impedance control, MoveIt Task Constructor, grasp planning with learned grasps, ISO 10218 and ISO/TS 15066 |
| Platform and infrastructure | Bazel, Kubernetes for fleets, telemetry pipelines, SROS2, rmw\_zenoh in practice, fleet observability |

**Emerging tier, on your terms**

- With a GPU of 16 GB VRAM or more: a focused four-week Isaac Sim and Isaac Lab project porting the warehouse world, generating Replicator data and comparing against your Gazebo-trained model.
- PREEMPT\_RT timing measurements for your controllers.
- Only then RL, LeRobot or VLA models, always as components inside a deterministic safety envelope.

## Sources

Job postings and market data behind the skill choices, from the August roadmap and the October job market research.

**Postings**

- [ARX Robotics: Senior Engineer, Robotics Perception](https://job-boards.eu.greenhouse.io/arxroboticsgmbh/jobs/4793357101)
- [ARX Robotics: Senior Engineer, Robotics Software](https://job-boards.eu.greenhouse.io/arxroboticsgmbh/jobs/4768877101)
- [ARX Robotics: Robotics Software Engineer, Sensor Fusion and Localization](https://www.arbeitnow.com/jobs/companies/arx/robotics-software-engineer-sensor-fusion-localization-munchen-38801)
- [MOTOR Ai: Robotics Software Integration Engineer](https://join.com/companies/motor-ai/16489452-robotics-software-integration-engineer-m-f-d)
- [Magazino: Robotics Software Engineer, C++](https://www.magazino.eu/robotics-software-engineer-c-f-m-x/?lang=en)
- [Magazino: Robot Deployment Engineer](https://www.arbeitnow.com/jobs/companies/magazino-gmbh/robot-deployment-engineer-munich-236223)
- [Sereact: Core Robotics Engineer](https://jobs.ashbyhq.com/sereact/c3003bd7-e099-4d71-8ef2-5b77ef11faeb)
- [NEURA Robotics: Robotics Software Engineer](https://jobs.neura-robotics.com/offer/robotics-software-engineer-human/b5dc91f5-00b0-4be7-842c-2257b3145a01)
- [NEURA Robotics: AI Simulation Engineer](https://jobs.neura-robotics.com/offer/ai-simulation-engineer-human/2416190f-d241-40dd-9f3f-b9fed1d71577)
- [KUKA: Global AI Robotics Engineering Program](https://www.kuka.com/en-de/company/careers/vacancies/global-ai-robotics-engineering-program-4571)
- [Agile Robots: Software Engineer, Industrial Robot Control](https://www.dreamworkhq.com/job/2868263a-5c36-4c6a-b5bf-1c93fedf39f3)

**Market data and technical references**

- [CareersInRobotics: Robotics Skills Map](https://careersinrobotics.com/skills/map)
- [CareersInRobotics: Robot Simulation Software](https://careersinrobotics.com/guides/robot-simulation-software)
- [DEVjobs.de: ROS jobs in Germany](https://en.devjobs.de/jobs/robot-operating-system)
- [ROS 2 distributions](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Releases.rst)
- [Gazebo releases](https://gazebosim.org/docs/latest/releases/)
- [gz\_ros2\_control, Jazzy](https://control.ros.org/jazzy/doc/gz_ros2_control/doc/index.html)
- [Isaac Sim requirements](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/installation/requirements.html)
- [VDA 5050 explained (IFR)](https://ifr.org/post/vda-5050-explained)
- [Ultralytics licensing](https://www.ultralytics.com/license)
