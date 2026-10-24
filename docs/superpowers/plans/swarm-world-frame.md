# Swarm world frame implementation plan

> Use executing-plans inline. Approved chat design authorizes implementation. No commits.

**Goal:** UAV1=(0,0,0,0), UAV2=(0,5,0,0), shared world maps/state/planning with separate FCU local frames.
**Architecture:** One adapter per UAV locks yaw/translation transforms from stationary LIO/FCU feedback, converts clouds/odom to world and commands to FCU local. EGO body-twist conversion is opt-in.
**Tech Stack:** ROS Noetic, catkin, C++14, Eigen, existing ROS messages, Python integration tests.
**Spec:** Approved chat design: level stationary initialization, same-height takeoff, known IMU-body mounting, controller_flag=1. No GNSS or online registration.

## Constraints and review focus
- Preserve existing edits and legacy defaults; no public message or LIO estimation changes.
- Check source/wall freshness, finite data, stationary attitude; no automatic realignment after READY.
- Preserve cloud fields/stamps and standard Odometry body twist/covariance.
- Own/peer readiness gates arming/motion; invalid data stops world setpoints for PX4 offboard-loss handling.
- Cover nonzero origin/yaw, 90deg, wrap, stale/backward stamps, tilt, jumps, masked commands and duplicate TF/publishers.

## Tasks
1. [x] Transform math: standalone C++ test RED, implement header, verify GREEN.
2. [x] ROS adapter: synthetic integration RED, initialization/invalidation/cloud/odom/setpoints, GREEN.
3. [x] Interfaces: controller readiness, EGO body velocity, isolated frames/inputs, UAV2 Y=5, disable old drift correction.
4. [x] Documentation and tests: launch audit, ROS tests, complete compile.sh summary, independent review and fixes.

## Execution ledger
- Baseline only ego_fastlio2_swarm_uav2.sh modified (X->Y, value 3). Preserve Y; authorized value 5.
- Reused checkout on codex/swarm-world-frame. No worktree requested; no commits.
- Ruling: existing design approval authorizes execution without another approval.
- Windows UNC patch/write unavailable; use WSL Python writes restricted to project files.

## Continuation ledger
- Resumed after the previous Turn stopped on a Codex usage limit; no code was lost.
- Launch audit failed on legacy load: `<group ns="">` is rejected by roslaunch. `mapping_mid360.launch` now falls back to ns=/ so legacy params/nodes stay at the root namespace; legacy remaps and frame names unchanged.
- launch_config_test compared remaps as tuples while roslaunch stores ordered lists: fixed by normalizing `remap_args`; added legacy/ world assertions for frames/body and frames/high_freq_body.
- `test_controller_requires_own_and_peer_readiness` was racy: it counted the single pre-arm idle heartbeat published at the ready->stale edge. It now asserts that motion setpoints stop and documents the allowed idle heartbeat.
- `run_frame_tests.sh` sourced ROS setup under `set -u`; ROS profile hooks read unbound variables. Strict mode now starts after sourcing.
- Review: IMU->body extrinsics composition moved into `swarm_frame_transform.h` helpers (`bodyPoint`, `bodyOrientation`) and is covered by the C++ test plus a new ROS test `test_body_imu_offset_shifts_lio_anchor`. Pose covariance rotation reviewed and kept (only pose block is rotated; body twist and its covariance untouched).
- Verification: `bash Modules/swarm_control/test/run_frame_tests.sh` (C++ test + 2 launch tests + 7 ROS tests) green twice, log `artifacts/swarm_world_frame/continue_tests.log`; `ROS_PARALLEL_JOBS="-j2 -l2" ./compile.sh` exit 0, all six modules OK, log `artifacts/swarm_world_frame/full_build_continue.log`.
- Not verified: real dual-aircraft flight, PX4 fusion parameters, clock sync, actual LIO relative drift. `git status` still shows the same edit set; nothing committed.

## Single-UAV SITL world-mode entry (Simulation/px4_forest)
- Added `--world-frame` to `run_sitl.py` and `world_frame=False` to `prepare_modules`, so the generated `modules.launch` binds LIO/control/planner to world mode (sensor topics passed as args, `uav_init_*=0`), and the generated `mavros_config.yaml` renames local_position frames to `uav1/fcu_local`/`uav1/base_link`.
- Ground station now waits for `/uav1/frame_ready` before sending Takeoff, otherwise the controller's readiness gate silently drops the single Takeoff message. Publisher audit switches to `/swarm_frame_adapter_uav_1` for `/uav1/mavros/setpoint_raw/local`, `/uav1/world/odom`, `/uav1/world/cloud`, `/uav1/prometheus/drone_state`.
- New `scripts/world_frame_evidence.py` + `tests/test_world_frame_evidence.py` produce `world_frame_evidence.json`: fixed-transform rigidity, initial anchor, stamp-paired command round trip (0x4000 heartbeat counted separately), and cloud alignment relative to the untransformed LIO cloud. `tests/test_module_launch.py` covers both frame modes. Simulation suite: 80 tests green.
- Two adapter defects found by the sim and fixed: (1) single-threaded spinner let a slow cloud callback starve the readiness watchdog -> `AsyncSpinner(3)` plus a state mutex, cloud transform/publish outside the lock; (2) `ros::WallTime` receive window equalled `source_timeout`, so CPU load latched INVALID -> separate `receive_timeout` (default `max(1.0, 2*source_timeout)`, yaml 1.5). INVALID now prints per-source age/recv/skew.
- Results: world mode `artifacts/px4_forest/world_frame_smoke3` ok (READY 5.15 s, armed 12.07 s, 2/2 goals, 4.8 cm final error, transform deviation 2e-16, anchor 5.3 mm/0.0075 deg, command error 2e-16, world-vs-LIO cloud median delta 1.6 mm). Legacy regression `artifacts/px4_forest/legacy_smoke` ok (2/2 goals, 12.6 cm), `frame_mode=legacy`, no adapter. Evidence copies in `artifacts/swarm_world_frame/`.
- Build: `ROS_PARALLEL_JOBS="-j2 -l2" ./compile.sh` exit 0, six modules OK (`full_build_sitl.log`). Still not verified: two-aircraft SITL, real flight, PX4 fusion tuning, LIO relative drift.

## Single-UAV hardware script in world mode
- `Experiment/mavros/launch/px4_mavros.launch` gained `enable_world_frame` (default false) overriding `local_position/frame_id`, `local_position/tf/frame_id`, `local_position/tf/child_frame_id` and `imu/frame_id` when enabled, matching the swarm launch.
- `ego_fastlio2_one_livox.sh` now defaults to world mode (driver via `swarm_mid360_driver.launch uav_id:=1`, LIO/control/EGO with `enable_world_frame:=true`, `swarm_num:=1`, `uav_init_*=0`); `WORLD_FRAME=0` restores the previous commands verbatim. Verified by expanding both modes with a stubbed `gnome-terminal`.
- `Modules/swarm_control/test/launch_config_test.py` gained single-UAV world wiring (frames, MAVROS params, adapter init, remaps, driver node) and legacy `px4_mavros.launch` frame assertions; adapter test roscore wait raised to 40 s after one transient startup timeout.
- Regression: world-mode SITL `artifacts/px4_forest/world_single_final` ok (READY 5.56 s, armed 12.09 s, 2/2 goals, 7.5 cm, transform deviation 2e-16, anchor 3.7 mm, command error 6e-17). Evidence pairing now ignores duplicate stamps, which removed a mm-level artifact in the earlier run. Tests: Modules 4 launch + 7 ROS green, Simulation 80 green.

## Recovery after the first commit
- Commit acd057d contained `CMakeLists.txt`'s test target but not `Modules/swarm_control/test/` (ignored by `.gitignore`); after the working tree cleanup those files were gone and `catkin_make` failed at configure time.
- Fix: the test target is now guarded with `EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test/frame_transform_test.cpp`, so a stripped checkout still configures; the four test files were restored (C++ math test, launch audit, adapter ROS test, runner) and re-run green (12 tests).
- `.gitignore` no longer ignores `Modules/swarm_control/test` or `docs/`; `Simulation/.gitignore`'s `tests/` rule still hides the Simulation tests and needs `git add -f` for the world-frame evidence test.
