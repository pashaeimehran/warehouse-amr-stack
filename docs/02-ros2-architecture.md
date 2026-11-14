# 02: ROS 2 architecture of the simulated platform

Scope: milestone v0.2-platform. How the ROS 2 side is put together (packages,
launch layers, TF tree, interfaces, lifecycle and composition), and two
measurements that justify the middleware and executor choices.

Everything below runs in the CI image (`docker/Dockerfile.ci`: ROS 2 Jazzy,
Gazebo Sim 8.15 Harmonic, headless). Measurements were taken on 2026-11-14 on
the development laptop (i7-10750H, WSL 2, inside Docker); they describe that
setup, not a robot computer.

## Packages and layering

| Package | Kind | Role |
| --- | --- | --- |
| `robot_core` | plain CMake, no ROS | domain types, mission FSM and executor (docs/01); also exported for colcon |
| `robot_interfaces` | rosidl | project messages; constants that mirror `robot_core` enums are checked by `static_assert` |
| `warehouse_description` | xacro | robot model, sensors, ros2_control, controller parameters |
| `warehouse_sim` | SDF, generated | world, AprilTag models, landmark map, station poses |
| `warehouse_bringup` | launch, config | layered launch, bridge, SLAM and navigation parameters |
| `perception_3d`, `tracking` | lifecycle components | v0.2 skeletons (lifecycle and composition only; logic in v0.5) |
| `qos_lab`, `executor_lab` | experiments | the two measurements below |

The ROS-free libraries never include ROS headers (checked by
`tools/ci/check_core.sh ros-independence`); adapters convert messages to core
types. The quality gate of the ROS workspace is `tools/ci/check_ros.sh`
(`build`, `test`).

## Launch layers

```
system.launch.py
 ├─ sim.launch.py         gz sim (headless: -s --headless-rendering), ros_gz_bridge
 ├─ robot.launch.py       robot_state_publisher, spawn, controller spawners
 ├─ navigation.launch.py  (navigation:=true) map_server, AMCL, Nav2
 └─ perception.launch.py  perception_container: perception_3d + tracking,
                          nav2_lifecycle_manager (autostart); apriltag_ros
navigation.launch.py      map_server, AMCL, Nav2 servers (system: navigation:=true)
mapping.launch.py         system (no perception, static world) + slam_toolbox
```

Each layer has its own arguments and can be started alone. Parameters live in
YAML files under `warehouse_bringup/config` and `warehouse_description/config`,
never inline in the launch files. The simulation layer sets
`GZ_SIM_RESOURCE_PATH` itself, so the same launch works from a sourced shell,
from `colcon test` and from `launch_testing` (an environment hook alone was not
applied under `colcon test`, and the world's `model://` includes failed).

## Robot model and actuation

- Differential drive, 0.70 x 0.50 m chassis, wheel radius 0.08 m, wheel
  separation 0.56 m, two frictionless casters.
- The wheels are driven only through ros2_control: `gz_ros2_control` provides
  velocity command interfaces and position and velocity state interfaces
  (the simulated wheel encoders); `diff_drive_controller` turns `/cmd_vel`
  into wheel velocities and publishes `/odom` and the `odom -> base_footprint`
  transform; `joint_state_broadcaster` publishes `/joint_states`. There is no
  direct Gazebo cmd_vel plugin (a test asserts it).
- Jazzy's `diff_drive_controller` accepts only `geometry_msgs/TwistStamped`,
  so every velocity command in the stack is stamped. A stale command older
  than 0.5 s (`cmd_vel_timeout`) stops the wheels.
- Ground truth: Gazebo's OdometryPublisher publishes the true base pose on
  `/ground_truth/odom` (frame `map`). It exists for evaluation only (ATE, 3D
  error) and never feeds control.

## TF tree (REP 105, REP 103)

```
map ──(AMCL or slam_toolbox, v0.4: own EKF)── odom ──(diff_drive_controller)── base_footprint
base_footprint ── base_link (z = 0.08 m, wheel axis)
base_link ── left_wheel_link, right_wheel_link     (robot_state_publisher, from /joint_states)
base_link ── laser_link                            (static)
base_link ── camera_link ── camera_optical_frame   (static; optical: z forward, x right, y down)
base_link ── imu_link                              (static)
base_link ── front_caster_link, rear_caster_link   (static)
```

The Gazebo world frame is the map frame, with its origin at the dock, so
ground truth, the landmark map (`warehouse_sim/config/tag_map.yaml`) and the
station poses share one frame. Images carry `camera_optical_frame` as frame
id and scans `laser_link`; the launch test checks both, and that every sensor
and wheel frame is reachable from `odom`.

## Topics

| Topic | Type | Source | Nominal rate |
| --- | --- | --- | --- |
| `/scan` | `sensor_msgs/LaserScan` | gpu_lidar via bridge | 10 Hz |
| `/camera/image_raw`, `/camera/depth/image_raw` | `sensor_msgs/Image` | rgbd_camera via bridge | 15 Hz |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | bridge | 15 Hz |
| `/imu` | `sensor_msgs/Imu` | IMU via bridge | 200 Hz |
| `/odom` | `nav_msgs/Odometry` | diff_drive_controller | 50 Hz |
| `/joint_states` | `sensor_msgs/JointState` | joint_state_broadcaster | 100 Hz |
| `/cmd_vel` | `geometry_msgs/TwistStamped` | Nav2 (later via safety_supervisor) | on demand |
| `/ground_truth/odom` | `nav_msgs/Odometry` | OdometryPublisher via bridge | 50 Hz |
| `/clock` | `rosgraph_msgs/Clock` | bridge | every sim step |

Rates are in simulation time. With CPU rendering (no GPU in WSL) the
simulation runs slower than real time, so wall-clock rates are lower: in the
smoke run the IMU reached 122 Hz, the camera 7.9 Hz and the scan 8.1 Hz of
wall time.

## Lifecycle and composition

`perception_3d` and `tracking` are `rclcpp_lifecycle::LifecycleNode`
components, loaded into one `component_container_mt` with
`use_intra_process_comms`. In v0.2 they are skeletons: they declare and
validate parameters in `on_configure`, create their publishers and
subscriptions, activate the lifecycle publishers in `on_activate`, and release
everything in `on_cleanup`. A subscription callback checks the node's state
itself, because rclcpp does not lifecycle-manage subscriptions.

`nav2_lifecycle_manager` configures and activates them in order when
`autostart:=true`; Nav2's bond heartbeat is disabled (`bond_timeout: 0.0`),
since these components do not implement it. The launch test
`test_perception_composition.py` checks that both components are listed in
the one container, and drives configure, activate and deactivate by hand
(the equivalent of `ros2 lifecycle set`).

Why lifecycle for perception: the node can be configured (model loaded, memory
allocated) before it starts consuming data, and a supervisor can deactivate
and reactivate it on a fault without restarting the process (scenario 7).
Why composition: nodes in one process can exchange messages without
serialization when intra-process communication is on; perception, 3D
localization and tracking form one pipeline per camera frame.

## Measurement 1: QoS, RMW and payload size (`qos_lab`)

Setup: a publisher and a subscriber process (`qos_publisher`,
`qos_subscriber`) on the same host, 1000 messages per cell at 100 Hz, three
complete runs (`benchmarks/results/qos_lab_2026-11-14_r{1,2,3}.json`). The
latency is one way, from the publisher's clock before `publish()` to the
subscriber's clock in the callback (same host clock). Loss counts sequence
numbers that never arrived. Profiles: `reliable_volatile` (ROS default, depth
10), `best_effort_volatile` (`SensorDataQoS`, depth 5),
`reliable_transient_local` (depth 1, the latched-topic profile).

Median over the three runs, [min, max] in brackets:

| RMW | Profile | Payload | Loss % | p50 ms | p95 ms | p99 ms |
| --- | --- | --- | --- | --- | --- | --- |
| Fast DDS | reliable_volatile | 256 B | 0.0 | 0.25 [0.25, 0.26] | 0.37 [0.34, 0.38] | 0.49 [0.43, 0.51] |
| Fast DDS | reliable_volatile | 1 MiB | 0.0 [0.0, 0.1] | 11.36 [11.31, 11.41] | 12.02 [12.01, 12.07] | 12.32 [12.17, 12.34] |
| Fast DDS | best_effort_volatile | 256 B | 0.0 | 0.23 [0.23, 0.24] | 0.36 [0.34, 0.39] | 0.51 [0.46, 0.52] |
| Fast DDS | best_effort_volatile | 1 MiB | 75.6 [55.7, 78.6] | 0.66 [0.65, 0.66] | 1.05 [1.04, 1.10] | 1.21 [1.17, 1.25] |
| Fast DDS | reliable_transient_local | 256 B | 0.0 | 0.28 [0.27, 0.28] | 0.38 [0.36, 0.40] | 0.52 [0.50, 0.54] |
| Fast DDS | reliable_transient_local | 1 MiB | 79.7 [69.2, 84.4] | 0.65 [0.61, 0.65] | 1.08 [1.04, 1.34] | 1.54 [1.38, 1.75] |
| Cyclone DDS | reliable_volatile | 256 B | 0.0 | 0.22 [0.22, 0.22] | 0.33 [0.30, 0.39] | 0.43 [0.42, 0.52] |
| Cyclone DDS | reliable_volatile | 1 MiB | 0.0 | 1.16 [1.14, 1.20] | 1.63 [1.55, 1.79] | 2.08 [1.84, 2.31] |
| Cyclone DDS | best_effort_volatile | 256 B | 0.0 | 0.21 [0.21, 0.21] | 0.31 [0.31, 0.32] | 0.42 [0.42, 0.43] |
| Cyclone DDS | best_effort_volatile | 1 MiB | 0.0 | 1.12 [1.11, 1.17] | 1.63 [1.54, 1.92] | 1.95 [1.84, 2.55] |
| Cyclone DDS | reliable_transient_local | 256 B | 0.0 | 0.22 [0.21, 0.22] | 0.31 [0.30, 0.32] | 0.43 [0.42, 0.43] |
| Cyclone DDS | reliable_transient_local | 1 MiB | 0.0 | 1.22 [1.21, 1.23] | 1.75 [1.66, 1.88] | 2.06 [2.01, 2.28] |
| either | pub best_effort / sub reliable | 256 B | 100.0 | n/a | n/a | n/a |

What the numbers say:

- **Small messages do not separate the configurations.** At 256 B every
  combination delivers everything with a p99 around 0.5 ms.
- **Large messages do.** At 1 MiB (the size of a 640 x 480 RGB frame, 0.9 MiB)
  Fast DDS with its default configuration delivered only 16 to 44 % of the
  messages under best effort, and under reliable with depth 1. Reliable
  delivery with depth 10 lost almost nothing, but at a p50 of 11.4 ms, ten
  times Cyclone's 1.2 ms. Cyclone DDS delivered every 1 MiB message under all
  three profiles. The latency columns of the lossy rows describe only the
  messages that arrived, so they are not comparable with the lossless rows.
- **"Reliable" is bounded by the history depth.** With `KEEP_LAST(1)` a
  reliable writer may overwrite a sample that has not been acknowledged yet,
  so reliable plus depth 1 is not "every message arrives". This explains the
  transient-local row under load; it is the right profile for latched state
  such as `/map`, not for streams.
- **Silent incompatibility.** A best-effort publisher and a reliable
  subscriber never match: 0 of 100 messages on both RMWs. Neither side errors;
  the only trace is one `incompatible_qos` event on each side, which
  `qos_subscriber` counts. This is why the sensor consumers in this stack
  subscribe with `SensorDataQoS` (best effort), which is compatible with both
  reliable and best-effort publishers.

Cause of the Fast DDS large-message loss, stated as a hypothesis that this
experiment does not isolate: at 100 Hz x 1 MiB (about 100 MB/s) the default
transport (shared memory with a UDP fallback, and fragmentation of large
samples) overruns its buffers. Fast DDS can be tuned (larger SHM segments,
socket buffers, flow control); the default behaviour is what matters here,
because it is what a stack gets without an XML profile.

Decision: **Cyclone DDS** (`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`) for the
stack, as planned in the roadmap, now backed by this measurement: lossless at
camera-sized payloads with a p99 near 2 ms on this host. Applied on 2026-11-14:
both images set `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` and `check_ros.sh`
defaults to it; the full ROS suite (88 tests) passed on Cyclone afterwards.
The four earlier consecutive green runs of the launch tests were on Fast DDS. The camera stays
best effort with a shallow depth, so a slow consumer drops old frames instead
of building latency.

## Measurement 2: executors and callback starvation (`executor_lab`)

Setup: one node with a 50 Hz timer (the "control loop", trivial work) and a
slow timer that burns 200 ms of CPU every 500 ms (a stand-in for point cloud
processing). Four configurations, 10 s each, three runs
(`benchmarks/results/executor_lab_2026-11-14_r{1,2,3}.json`). A missed
activation is a 20 ms slot in which the fast callback did not start.

| Configuration | 50 Hz calls (of 500) | Missed | Period p50 ms | p95 ms | max ms |
| --- | --- | --- | --- | --- | --- |
| SingleThreaded, default group | 310 | 200 | 20.01 [20.00, 20.01] | 219.83 [219.78, 219.91] | 220.49 [220.45, 220.88] |
| SingleThreaded, separate groups | 310 | 200 | 20.01 [20.00, 20.01] | 219.96 [219.86, 219.96] | 220.37 [220.34, 220.42] |
| MultiThreaded (4), default group | 309 | 190 | 20.01 [20.00, 20.02] | 219.86 [219.84, 219.97] | 220.42 [220.34, 220.50] |
| MultiThreaded (4), separate groups | 500 | 0 | 20.00 [20.00, 20.00] | 20.36 [20.36, 20.45] | 21.05 [20.68, 21.19] |

Explanation of the starvation result:

- A **single-threaded executor** runs one callback at a time. While the slow
  callback burns its 200 ms, the 50 Hz timer cannot start; when it finally
  runs, rclcpp skips the missed periods instead of catching up. Every slow
  callback therefore costs about ten fast activations (200 per 10 s), and the
  p95 period is about 220 ms (200 ms of work plus one period). Callback groups
  change nothing here, because there is only one thread.
- A **multi-threaded executor alone does not help either.** Both timers sit in
  the node's default callback group, which is *mutually exclusive*: the
  executor never runs two callbacks of that group concurrently, so the fast
  timer still waits for the slow one. This is the result that surprises people
  most: adding threads is not enough.
- Only **multi-threaded plus separate callback groups** lets the fast timer
  run on another thread while the slow work continues: 500 of 500 activations,
  p95 period 20.4 ms, worst case 21.2 ms over all runs.

Consequences for the stack: control-relevant callbacks (safety supervisor,
controller inputs) never share a mutually exclusive group with heavy
processing; heavy nodes (point clouds, inference) run in their own group or
their own executor thread. Each callback group is held as a member: nodes
keep only weak references to callback groups, and in a first version of this
experiment groups created as constructor locals were destroyed at the end of
the constructor, and their timers silently never ran (0 calls, no error).

## Mapping, localization and navigation

**Map.** `mapping.launch.py` runs slam_toolbox (online async) on the static
world variant (`warehouse_static.sdf`, no walking person, so no traces in the
map) while `tools/mapping/drive_route.py` drives a fixed route through every
aisle. The route steers on ground truth; that is acceptable for this offline
tool only, and gives every rebuild the same coverage and loop closures. The
saved map is `warehouse_bringup/maps/warehouse.{yaml,pgm}` (503 x 324 cells,
0.05 m).

Map accuracy against the true geometry (`tools/analysis/map_accuracy.py`,
which takes the obstacle outlines at LiDAR height from the world generator;
`benchmarks/results/map_accuracy_2026-11-14.json`):

| Metric | Value |
| --- | --- |
| Occupied cells | 6489 |
| Distance of occupied cells to the true outline | median 0.006 m, p95 0.056 m, max 0.106 m |
| Occupied cells within 0.10 m of a true outline | 99.98 % |
| Visible true outlines covered by the map (within 0.10 m) | 99.74 % |

The map frame coincides with the world frame because mapping starts at the
dock; no alignment is estimated, so drift would appear as error.

**Navigation layer** (`navigation.launch.py`, `config/nav2.yaml`): map_server,
AMCL, planner (NavFn), MPPI controller, smoother, behaviors, velocity smoother,
BT navigator and waypoint follower, managed by one lifecycle manager. This is
the project's own launch rather than nav2_bringup's, which in Jazzy also starts
route_server and docking_server. Costmaps use the real rectangular footprint
(0.72 x 0.63 m) with 0.55 m inflation, sized for the 2.2 m aisles.

**AprilTags.** `apriltag_ros` runs in the perception layer on the camera image
(Gazebo images have no distortion, so `image_raw` serves as the rectified
image) with decimation off: at the dock the nearest tags are about 22 px wide.

Acceptance (`test_navigation.py`, headless, full system):

| Check | Result (2026-11-14) |
| --- | --- |
| Tags 2 and 4 detected from the dock | yes; TF position error against `tag_map.yaml` 0.034 m or 0.106 m depending on the run (tolerance 0.15 m) |
| Dock to Station A with Nav2 and AMCL | SUCCEEDED, 19.2 to 21.2 s of simulation time, ground-truth goal error 0.08 to 0.13 m |

The tag error is dominated by range: a 0.2 m tag at about 4 m spans about
22 px, so half a pixel of corner error is about 9 cm along the viewing ray.

### Failure modes found while bringing this up

Each of these cost a debugging round; the fixes are in the code and the
reasoning is here so they are not repeated.

- **slam_toolbox is a lifecycle node in Jazzy.** Started as a plain node it
  waits unconfigured, publishes no `/map`, and logs nothing; `map_saver` then
  fails with "Failed to spin map subscription". The mapping launch manages it
  with a lifecycle manager (`use_lifecycle_manager: true`).
- **A node-name entry in a parameter file beats the launch file's value.**
  `nav2.yaml` had `map_server: yaml_filename: ""`; the launch file's map path
  went into a wildcard entry, which loses to the node-specific one, so
  map_server started without a map and AMCL never published `map -> odom`.
  The key is now set only by the launch file.
- **MPPI's sampling bounds are not the drive's limits.** Setting `ax_max` to
  the drive's 0.8 m/s^2 together with a 20 Hz loop that this CPU could not
  hold made MPPI command 0.03 m/s; the progress checker then aborted every
  attempt. With the default bounds and a 10 Hz loop matching `model_dt`, the
  robot cruises at 0.47 m/s. Acceleration is limited downstream (velocity
  smoother, diff_drive_controller).
- **A hung launch test orphans Gazebo.** When CTest kills a launch test on
  timeout, launch's shutdown never runs; the orphaned `gz sim` keeps the test's
  output pipe open, and `colcon test` then blocks until its outer timeout. The
  tests now wait with explicit deadlines (AMCL transform, action server,
  `bt_navigator` active) and fail instead of hanging.
- **Launch tests must be isolated.** A Gazebo server that ignored SIGINT for
  5 s in one test was still reachable from the next: its controller manager
  answered the next test's spawners ("controller already loaded"). Each launch
  test now sets its own `ROS_DOMAIN_ID` and `GZ_PARTITION`.
- **apriltag_ros 3.4.0 segfaults during teardown** (exit -11 after SIGINT,
  after working correctly). Only that process is allowed -11 in the exit-code
  check; every other process must exit cleanly.

## Tests of this layer

| Test | Level | Checks |
| --- | --- | --- |
| `robot_interfaces/test_interfaces` | GTest | mirrored constants equal `robot_core` enums (compile time), message composition |
| `qos_lab/test_probe` | GTest | probe layout, sequence accounting (loss, duplicates, reordering), profile policies |
| `executor_lab/test_period_analysis` | GTest | missed-activation counting, jitter tolerance, input validation |
| `warehouse_description/test_urdf` | pytest | frame tree, optical frame, controller geometry equals URDF, interfaces, sensors, no cmd_vel plugin |
| `warehouse_sim/test_world` | pytest | generator reproducible, tags decode to their ids, goals clear, aisle width |
| `warehouse_bringup/test_perception_composition` | launch_testing | two components in one container, manual lifecycle transitions |
| `warehouse_bringup/test_platform` | launch_testing, headless Gazebo | all topics within 20 s, TF complete, `/cmd_vel` moves the wheels |
| `warehouse_bringup/test_navigation` | launch_testing, headless Gazebo, full system | tags detected at their map poses; dock to Station A with Nav2 and AMCL |

Measured startup in `test_platform` (2026-11-14, CPU rendering): first
message after 6.7 s (`/imu`) to 10.9 s (`/odom`), budget 20 s.
