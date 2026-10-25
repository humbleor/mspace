# PX4 forest fleet implementation plan

> **For agentic workers:** Use executing-plans task by task. User approved the technical route and requested implementation, with dual-UAV acceptance and configuration-driven N-UAV expansion.

**Goal:** N independent PX4/LIO/EGO/control chains in one forest; accept two UAVs first.
**Architecture:** Preserve single-UAV runner. Add fleet runner with per-vehicle config/state and shared ROS/Gazebo masters, BVH, clock and process ownership. Parameterize model/module generators; retain navigation algorithms.
**Tech Stack:** Ubuntu 20.04, ROS Noetic, Python 3, Gazebo Classic 11, pinned PX4 v1.13.2.
**Spec:** Approved technical route in this chat.

## Global constraints
- No GPS navigation or runtime truth feedback to navigation. Configured initial placements define world.
- One map-to-Gazebo transform; common takeoff-height origin, init_z=0.
- Consecutive project IDs starting at 1 for EGO predecessor protocol; independent PX4 instances/ports.
- Preserve point timing, IMU units and navigation algorithms.
- Reuse authorized current checkout; no commits, branch changes or unrelated edits.
- Dynamic UAV lidar returns deferred; initial scope is forest sensing and trajectory communication.

## Review focus
Three-aircraft resource isolation; nonzero placement/yaw with shared forest; all-vehicle completion and stale-source failure; model-owned tree/UAV contacts; scoped cleanup including partial startup.

### Task 1: Configuration and generators
Files: fleet.py, runtime_model.py, module_launch.py, scan_message.py, world_frame_evidence.py, tests/test_fleet.py.
- [x] RED: invalid/duplicate IDs, three-vehicle resource isolation, frame serialization, nonzero anchors and shared sensor origin.
- [x] GREEN: validated vehicle specs, parameterized model/module generators, configurable frames/anchors.
- [x] Verify old helper defaults and two/three-aircraft generated structure.

### Task 2: Physical interfaces and runtime
Files: forest_plant.cpp, run_swarm.py, fleet_evidence.py, run.py, fleet_two.json, fleet_three.json.
- [x] RED: continuous pair separation, graph ownership and all-vehicle gates.
- [x] GREEN: parameterized plugin with owner-filtered tree/UAV contacts, independent sensor and navigation contexts, shared lifecycle.
- [x] Build plugin/affected Modules; real two-UAV hover/forest validation, three-UAV startup if resources allow.

### Task 3: Entry, docs and verification
Files: start.sh, Simulation/README.md, px4_forest/README.md, docs/fleet.md.
- [x] Add fleet command and examples.
- [x] Available simulation suites, shell syntax, diff checks, original single-UAV regression.
- [x] Fresh independent review; record actual results/limits.

## Progress / rulings
- Workspace clean on main, no attached worktree; reuse current checkout under user authorization. No branch changes/commits.
- Inline execution. This document is also the progress ledger.
- Native patch tool cannot write UNC workspace; use scoped WSL Python file writes instead.

- Task 1 complete: 87 PX4 tests passed (7 new), including three real SDF generations using pinned PX4. Model file and world now contain identical plugin configuration.

- Task 2: real dual hover passed, minimum separation 2.989 m; scoped cleanup complete.
- Fixed pinned Gazebo per-model speed multiplication by neutral gzserver environment and first-Tick global rate assignment (native plugin rebuilt).
- Fixed actual ROS trigger remap ordering and case-sensitive simulation Takeoff_height parameter. Disabled legacy global visualization TF in generated world-mode launch.
- Independent review found unbounded service calls before cleanup; bounded calls and a global final-audit budget added. Re-review found no remaining actionable issues.
- Coincident synthetic cloud regression proved reference thinning biased absolute alignment distances. Keep full reference KD tree, unchanged thresholds.
- Real dual forest run fleet_forest_two_v3_20261025 passed: both original goals, physics contacts, EGO clearance, world alignment, graph ownership/truth isolation, pair separation and cleanup. Goal errors 0.0626/0.0852 m.
- New tests moved to tracked checks/ because historical Simulation/**/tests are ignored.

- Task 2 complete: three-aircraft forest at .25 speed passed with initial yaw=.4 on UAV3, 301 scans each, goal errors .0679/.0932/.0730 m. All pair minima >=1.565 m, cleanup complete. .5 speed exceeded sensor backlog and failed closed; no tolerance relaxed.
- Task 3 checks: complete simulation suites 33+16+80=129 passing, shell syntax and document links checked. Original single-UAV short physical regression running.

- Task 3 complete: original single-UAV short passed with physical goal error .0794 m, original EKF/report checks and cleanup complete. All changes remain uncommitted. Final artifact audit and related module build verification recorded in this session.
