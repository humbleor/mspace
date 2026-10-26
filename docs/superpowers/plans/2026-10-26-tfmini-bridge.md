# TFmini and Shared-Master Bridge Implementation Plan

> **For agentic workers:** Use executing-plans inline; verify each task before continuing. Steps use checkbox tracking.

**Goal:** Normalize takeoff_height, expose TFmini to ROS, and make the two-aircraft bridge testable without ROS topic feedback.
**Architecture:** Preserve LIO XYZ and disabled tree correction. Keep existing simulation/direct-ROS topology as the default planner mode, and add explicit bridge mode for real scripts. Use independent per-aircraft TX/RX names and bounded, framed ROS serialization on TCP/UDP.
**Tech Stack:** Ubuntu 20.04, ROS Noetic, C++14, Python unittest/rospy, MAVROS 1.20.1.
**Spec:** docs/2uav_real_experiment_plan.md plus the user's approved five-item fixes; scope here is takeoff, TFmini and bridge, not tree correction.

## Global Constraints
- Preserve pre-existing edits, including current lidar suffixes, GCS address and ground-station changes.
- Work in the current user-requested checkout; do not commit, reset, switch branches or operate hardware.
- PX4 EKF parameters are unchanged; TFmini ROS reception is not altitude fusion or clearance protection.
- Direct-ROS simulation must retain its existing broadcast topology.
- Both real computers must deploy the same new bridge protocol; reject old/malformed packets safely.

## Review Focus
- Same ROS Master must not create feedback or bypass bridge-mode transport.
- MAVROS sensor IDs are wire IDs, not uORB device_id; frames must be unique per UAV.
- TCP fragmentation/coalescing/truncation must not publish corrupt trajectories or allocate unbounded vectors.
- Startup without peer, peer restart and process shutdown must remain bounded.
- Invalid/self/duplicate UDP data and disconnect state must not be mistaken for valid peer measurements.

### Task 1: Normalize takeoff configuration
Files: swarm_controller.h/.cpp, ego_swarm_control.launch, Simulation/px4_forest/scripts/module_launch.py, focused launch/runtime tests.
Interface: canonical private takeoff_height float, launch argument default 1.2; all variable uses lowercase.
- [x] Write failing launch/isolated-controller tests for lower-case configured value.
- [x] Normalize key, variable, references and generated SITL configuration.
- [x] Verify configured takeoff changes the synthetic Z target; run existing frame tests.

### Task 2: Receive TFmini through MAVROS
Files: Experiment/mavros/launch TFmini config and single/swarm entries; focused launch and real MAVROS synthetic-FCU tests.
Interface: /uavN/mavros/distance_sensor/tfmini_down (sensor_msgs/Range), frame uavN/tfmini_link; configurable wire ID and FOV, no automatic TF from unknown installation.
- [x] Write failing tests for unique mapping and actual DISTANCE_SENSOR->Range reception.
- [x] Enable only the intended inbound range mapping; expose sensor ID on launch/script.
- [x] Verify separate synthetic FCU streams for both UAVs, IDs and metre units.

### Task 3: Repair bridge and document ground test
Files: bridge_node.cpp, bridge.launch/CMakeLists, real_ego_run.launch/advanced_param_px4.xml, real UAV scripts, transport/config tests, docs/tfmini_bridge_ground_test.md.
Interfaces: /uavN/bridge/tx/{multi_traj,bspline}; /uavN/bridge/rx/{prev_multi_traj,bspline}; explicit use_bridge=false planner default, true in real two-UAV scripts. TCP chain sends N->N+1, UDP peer exchange. Configurable ports enable isolated loopback tests.
- [x] Write failing topology and real-process socket tests.
- [x] Add bounded versioned packets, TCP stream framing, separate TX/RX, sender filtering, UDP deduplication, periodic latched TCP state and bounded reconnect/shutdown.
- [x] Run TCP split/combined/truncated frames, UDP invalid/self/duplicate tests and two-node same-Master no-feedback/restart checks.
- [x] Update scripts and practical ground-test instructions; update experiment-plan status.

### Task 4: Build and review
- [x] Run focused and existing frame/launch tests; compile.sh all modules, inspect full summary.
- [x] Run fresh read-only review, fix important findings with regression tests.
- [x] Report software evidence and the outstanding real hardware ground test; do not claim flight readiness.
