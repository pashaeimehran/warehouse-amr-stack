# Implementation status

Living record of what exists, what was verified and how, and what is next.
Update it after every meaningful stage. Numbers here are measured, never
estimated; anything not run is listed under "Unverified".

Last updated: 2026-11-14.

## Current milestone

**v0.2-platform** (roadmap week 1, Thu to Sun): every done-when criterion
verified locally; waiting for the first GitHub run of the new `ros` job and the
tag. v0.3-perception started (`perception_core`). v0.1-core is complete and
tagged (`v0.1-core`, tag set by the author).

## Environment

| Item | Value |
| --- | --- |
| Host | Windows 11, WSL 2 (kernel 6.18.58.1), Ubuntu 24.04, i7-10750H (12 threads), 16 GB RAM (WSL VM: 7.8 GB) |
| Docker | Docker Desktop with WSL integration; engine memory limit about 8 GB |
| ROS 2 and Gazebo | only inside the images: ROS 2 Jazzy, Gazebo Sim 8.15 (Harmonic) |
| CI image `wmm-ci:local` | 5.88 GB: toolchain, `docker/install_ros_packages.sh ci`, actor mesh (`fetch_sim_assets.sh`), `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` |
| Dev image `wmm-dev:local` | 5.88 GB: toolchain, ROS packages, RViz 2, actor mesh, Cyclone DDS; built and checked 2026-11-14 (GPU path unusable, see below) |
| GPU | RTX 2060 not reachable from WSL: `/dev/dxg` exists but the NVIDIA user-space libraries (`libcuda.so`, `nvidia-smi`) are missing under `/usr/lib/wsl/lib`. Gazebo therefore renders on Mesa llvmpipe (CPU) |

## Completed and verified

### v0.1-core (done)

- `robot_core`: see `docs/01-core-design.md`. GitHub Actions green on `main`
  (push and nightly), verified with `gh run view` on 2026-11-14: 237 tests
  debug and release, 239 ASan, 238 UBSan, 238 TSan, TSan MPMC stress 1000 of
  1000, line coverage 98.2 % (642 of 654), branch coverage 89.7 %.
- The first push failed with exit code 126: `/mnt/d` runs git with
  `core.filemode=false`, so `check_core.sh` was committed as 100644. Fixed
  by the author (commit 640c3c9). The `shell-lint` step now checks the index
  mode of every tracked `*.sh` so this cannot recur silently.

### v0.2-platform (so far)

| Component | State | Verified by |
| --- | --- | --- |
| `robot_core` colcon export (`package.xml`, install and export rules, `robot_coreConfig.cmake`) | done | consumer project with `find_package(robot_core)`; used by `qos_lab`, `executor_lab`, `robot_interfaces` tests |
| `robot_core/sample_statistics.hpp` (min, max, mean, stddev, NumPy-compatible percentiles) | done | 7 GTests; core suite 244 of 244 (debug, host) |
| `robot_interfaces` (13 messages: `DetectedObject3D(Array)`, `TrackedObject(Array)`, `HealthStatus`, `MissionCommand`, `MissionLeg`, `Waypoint`, `SafetyState`, `LocalizationStatus`, `ItemPose`, `PickCommand`, `ObjectClass`) | done | 3 GTests; `static_assert`s that mirrored constants equal the `robot_core` enums |
| `qos_lab` (probe publisher and subscriber, matrix driver) | done, measured | 10 GTests; 3 runs of the 3 x 2 x 2 matrix plus the incompatibility case (`benchmarks/results/qos_lab_2026-11-14_r{1,2,3}.*`) |
| `executor_lab` (starvation probe, 4 configurations) | done, measured | 4 GTests; 3 runs (`benchmarks/results/executor_lab_2026-11-14_r{1,2,3}.*`) |
| `perception_3d`, `tracking` lifecycle components | **skeletons**: transitions, parameters, publishers and subscriptions only; no geometry and no tracker yet (v0.5) | launch test: both in one container, manual configure, activate and deactivate |
| `warehouse_description` (diff-drive base, 2D LiDAR, RGB-D, IMU, ros2_control, ground-truth odometry) | done | 9 pytest checks (frame tree, REP 103 optical frame, controller geometry equals URDF, no direct cmd_vel plugin) |
| `warehouse_sim` (generated world: walls, 4 shelf rows, 96 KLT boxes, dock, stations A and B, drop-off, 8 AprilTags, walking actor; static variant for mapping; `tag_map.yaml`, `stations.yaml`) | done | 17 pytest checks: generator reproducible, every tag texture decodes to its id, goals clear of shelves, aisle width, actor loop clear of the test corridor |
| Walking actor mesh (Gazebo Fuel `Mingfei/actor` v1, CC BY 4.0) | baked into the images by `docker/fetch_sim_assets.sh`, SHA256-pinned | checksum verified in the image build; world loads it in every launch test |
| `warehouse_bringup`: `sim`, `robot`, `navigation`, `perception` (incl. apriltag_ros), `system`, `mapping` launch; bridge, slam_toolbox, Nav2 and AprilTag configs | done | launch tests `test_perception_composition`, `test_platform`, `test_navigation` |
| Map (`maps/warehouse.yaml`) from slam_toolbox along `tools/mapping/drive_route.py` | done | `tools/analysis/map_accuracy.py`: median 0.006 m, p95 0.056 m to the true outlines; 99.98 % of occupied cells within 0.10 m |
| Nav2 with AMCL, MPPI; `navigate_stations.py` (simple commander) | done | dock to Station A SUCCEEDED in launch test and diagnostic run (19.2 to 21.2 s sim time, goal error 0.08 to 0.13 m) |
| apriltag_ros detections as TF | done | tags 2 and 4 detected from the dock at 0.034 to 0.106 m from their map poses |
| CI job `ros` in `ci.yml` (`check_ros.sh build test`) | written, actionlint clean | **not yet run on GitHub** |
| Stack RMW: Cyclone DDS (`RMW_IMPLEMENTATION` in both images and `check_ros.sh`) | done | full ROS suite green on Cyclone |
| Launch-test isolation (own `ROS_DOMAIN_ID` and `GZ_PARTITION` per test) | done | 4 consecutive green runs of the suite on Fast DDS, then 1 on Cyclone |
| `perception_core` (v0.3, Python, no ROS): letterbox, IoU, NMS, `DetectorBackend`, `MockBackend`, YOLO decoder | done except `TorchBackend` | 27 pytest; ROS-independence gate now also checks Python imports (canary verified) |

Platform launch test (headless `gz sim -s --headless-rendering`, CPU
rendering, 2026-11-14): first message 6.7 to 10.9 s after start for all nine
topics (budget 20 s), TF from `odom` to all sensor and wheel frames, and
`/cmd_vel` (TwistStamped, 0.3 m/s for 3 s sim time) moved the robot 0.900 m
straight with wheel odometry within 5 cm of ground truth.

## Tests actually executed (latest)

| Suite | Where | Result |
| --- | --- | --- |
| `tools/ci/check_core.sh all` (JOBS=4) | host | ros-independence, format (53 files), shell-lint (5 scripts), tidy: clean; debug 244, release 244, ASan 246, UBSan 245, TSan 245 (canaries detected); coverage 98.2 % lines (669 of 681), 90.1 % branches |
| `tools/ci/check_ros.sh build test` | `wmm-ci` container, Cyclone DDS | 10 packages build with `-Werror`; 88 tests, 0 failures, 2 min 42 s |

GitHub Actions has not run the new `shell-lint` index check or the `ros` job:
nothing was pushed in this session.

## Known failures and limitations

- Real-time factor with CPU rendering is below 1 (sensor rates in wall time
  about 0.5 to 0.8 of nominal). Tests use simulation time; wall-clock rates
  are not representative of a GPU machine.
- Fast DDS loses about 79 % of 1 MiB messages under `best_effort_volatile` and
  `reliable_transient_local` (depth 1) at 100 Hz on this host; Cyclone DDS
  loses none. Measured in one container on one host only.
- colcon builds on the `/mnt/d` bind mount print "clock skew" warnings
  (drvfs timestamps); harmless.

## Unverified functionality

- RViz visualisation (needs the dev image with a display); navigation GIF.
- The dev image's GPU path (blocked, see Environment).
- `view_frames` output (TF completeness is asserted by the launch test instead).

## Blockers and prerequisites

| Blocker | Affects | Owner |
| --- | --- | --- |
| NVIDIA WSL user-space libraries missing | GPU dev container, TensorRT (v0.8), GPU rendering | author (Windows driver install or repair) |
| TinyGPU use not yet authorised for this project | training at scale (v0.8) | author |

## Next tasks

See `docs/implementation-plan.md`. Author actions: push, so the `ros` CI job
runs on GitHub for the first time; RViz check and navigation GIF on a machine
with a display; tag `v0.2-platform` once the CI run is green. Next
implementation step (v0.3): the `perception` lifecycle node around
`perception_core` (Detection2DArray with source stamps, drop-old-frames,
`PerceptionDiagnostics`, node tests with `MockBackend`), then the YOLO11n
baseline on saved Gazebo frames (needs PyTorch and Ultralytics in a tools-only
environment; CPU inference, since the GPU is not reachable from WSL).
