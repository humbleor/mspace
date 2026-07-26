# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概述

**Mspace** 是一个基于 ROS Noetic (Ubuntu 20.04) 的无人机自主飞行项目，集成 **Fast-LIO2**（激光-惯性里程计）和 **EGO-Planner Swarm**（去中心化多机路径规划）。支持单机和蜂群（多 UAV）配置，适配 Livox Mid-360 和 Ouster 激光雷达。

## 构建

**前置条件：构建前必须先 source `livox_ros_driver2` 工作空间**（这是最常见的构建失败原因）：
```bash
source ~/workspace/ws_livox/devel/setup.bash
```

全量构建（始终在项目根目录）：
```bash
./compile.sh
```

`compile.sh` 按顺序构建：mavros → msgs → fast_lio2 → ego_planner_swarm → swarm_control → realsense_ros。`msgs` 必须先于依赖自定义消息的模块构建。所有构建产物输出到**项目根目录**的 `build/<module>/` 和共享的 `devel/`，而不是各模块内部。

单独重建某个模块（与 `compile.sh` 内部命令一致）：
```bash
catkin_make --source Modules/ego_planner_swarm --build build/ego_planner_swarm
```

运行任何节点前先 source 工作空间：
```bash
source devel/setup.bash
```

**本项目没有 lint、测试套件和 CI** — 通过 `./compile.sh` 构建并检查编译错误来验证改动。

## 目录名 ≠ ROS 包名

`roslaunch` / `rosrun` 使用**包名**而非目录名：

| 目录 | ROS 包名 |
|------|---------|
| `Modules/fast_lio2/` | `fast_lio` |
| `Modules/ego_planner_swarm/plan_manage/` | `ego_planner` |
| `Modules/swarm_control/` | `prometheus_swarm_control` |
| `Experiment/mavros/` | `mavros_bringup` |
| `Simulation/mspace_drone/` | `mspace_drone` |

## 代码架构

### Modules/ego_planner_swarm — 核心规划与仿真
最大的模块，fork 自 ZJU-FAST-Lab/ego-planner-swarm。各 catkin 包**直接位于模块根目录下**（原 `src/planner/`、`src/uav_simulator/` 嵌套结构已扁平化）：

- **plan_manage** — 高层规划调度与 launch 文件（包名 `ego_planner`）
  - `launch/` — 原始 ego-planner launch（`simple_run.launch`、`swarm.launch`、`simulator.xml`）
  - `launch_new/` — **当前使用的 launch**：LiDAR/VIO 仿真变体（`1uav_mid360_sim.launch`、`1uav_os128_sim.launch`、`2uav/4uav/10uav_mid360_sim.launch`、`*_vio_sim.launch`）与真机入口 `real_ego_run.launch`
- **plan_env** — 环境感知：`grid_map`（体素占据栅格）、`obj_predictor`（动态障碍物预测）、`raycast`
- **bspline_opt** — B 样条轨迹优化（碰撞/平滑/可行性梯度下降优化）
- **path_searching** — `DynAStar` 运动学 A* 路径搜索
- **traj_utils** — 轨迹工具函数
- **drone_detect** — 基于深度/相机数据的无人机互检，CATKIN_IGNORE忽略编译
- **drone_detect_lidar** — 森林场景树干特征多机相对定位（见下节）
- **rosmsg_tcp_bridge** — 基于 TCP 的 ROS 消息桥接，用于蜂群通信
- **uav_simulator/** — 仿真基础设施：`fake_drone`（默认，轻量运动学模型）、`local_sensing`（仿真深度/点云传感器，CPU/GPU 模式）、`lidar_map_generator` / `map_generator` / `mockamap`（地图生成）、`so3_control` + `so3_quadrotor_simulator`（完整 SO(3) 动力学，可选，CPU 占用高）

### drone_detect_lidar — 树干匹配相对定位
三个节点组成的流水线（详见其 README）：
1. `tree_detector_node` — 点云 → 体素降采样 → 高度裁剪 [0.3, 3.0]m → PatchWork++ 地面剔除 → 欧氏聚类 → PCA 多条件筛选 → 树干位置（质心 XY + 最低点 Z），广播 `TreeDetection` 消息
2. `tree_loc_node` — 三角形哈希匹配（HashReg 思想）+ 投票验证 + 2D SVD 闭式解求 (dx, dy, yaw)，发布 `tree_pose_error` / `TreeRelativePose`
3. `tree_graph_node` — anchor 机上的 GTSAM 因子图优化，发布 `TreeRelativePoseOptimized`。**GTSAM 是可选依赖**：未安装时此节点自动跳过编译（CMake `find_package(GTSAM QUIET)`）

漂移评估流水线（注入漂移 → 仿真 → 精度评估，结果输出到 `Scripts/drift_eval_<时间戳>/`）：
```bash
cd Modules/ego_planner_swarm/drone_detect_lidar/Scripts
./run_tree_drift_eval.sh [时长] [uav1_dx] [uav1_dy] [uav1_yaw°] [uav2_dx] [uav2_dy] [uav2_yaw°]
# 示例：仅 UAV2 漂移，UAV1 为 anchor
./run_tree_drift_eval.sh 120 0 0 0 0.5 0.3 5.0
```

### Modules/fast_lio2 — 激光-惯性里程计（包名 `fast_lio`）
单一 catkin 包：`src/drone_laserMapping.cpp`（主建图节点）、`src/preprocess.cpp`（点云预处理）、`src/IMU_Processing.hpp`（IMU 积分）。`include/ikd-Tree` 是 git submodule。每种雷达对应一份 `config/*.yaml`（`mid360.yaml`、`ouster128.yaml` 等）和 `launch/mapping_*.launch`。

### Modules/swarm_control — 真机蜂群执行（包名 `prometheus_swarm_control`）
物理硬件上的蜂群任务节点：`swarm_terminal_control`（终端任务控制）、`swarm_controller`（高层协调）、`swarm_formation_control`（编队）、`swarm_estimator`（状态估计）、`swarm_ground_station`（地面站）、`ego_traj_to_cmd`（EGO 轨迹 → MAVROS 指令）。

### 其他模块
- **Modules/common/msgs** — 跨模块共享的自定义 msg/action（`DroneState`、`SwarmCommand`、`ControlCommand`、`Formation` 等）
- **Modules/realsense_ros** — RealSense D435/D435i 驱动 fork（真机录制用）
- **Experiment/mavros** — 真机部署的 MAVROS launch/配置（包名 `mavros_bringup`，真机脚本依赖它）
- **Simulation/mspace_drone** — PX4 SITL + Gazebo 仿真集成（`sitl_ego_planner.launch`、`sitl_ego_planner_mid360.launch`）。**不在 `compile.sh` 构建列表中**；需先自行搭建 PX4 仿真环境并运行 `Simulation/shell/gazebo_setup.bash`，依赖 NLopt

## 运行

### 仿真（先 `source devel/setup.bash`）
```bash
roslaunch ego_planner 1uav_mid360_sim.launch    # 单机，Livox Mid-360
roslaunch ego_planner 1uav_os128_sim.launch     # 单机，Ouster OS128
roslaunch ego_planner 4uav_mid360_sim.launch    # 4 机蜂群（另有 2uav/10uav/vio 变体）
roslaunch drone_detect_lidar 2uav_lidar_detect_sim.launch   # 双机树干互定位（3uav 版含因子图优化）
```
LiDAR 仿真从 `~/bagfiles/resource/` 加载 PCD 地图（仓库外资源，路径由 launch 文件的 `map_name` 参数指定）。

### 真机（gnome-terminal 多标签脚本，会同时 source `ws_livox` 和本工作空间）
```bash
./ego_fastlio2_one_livox.sh       # 单机 Livox
./ego_fastlio2_ouster.sh          # 单机 Ouster
./ego_fastlio2_swarm_mavros.sh    # 蜂群主机1：MAVROS
./ego_fastlio2_swarm_uav1.sh      # 蜂群主机1：UAV1 规划
./ego_fastlio2_swarm_uav2.sh      # 蜂群主机2：UAV2
./topiclistOS0.sh                 # rosbag 录制点云/相机话题
```
真实蜂群是多主机部署，各机通过 MAVROS/ROS 网络通信。

## 分析脚本（Scripts/）
- `visualize_waypoints.py` — 航点可视化
- `analyze_flight_stability.py` / `analyzer_ego_stability.py` — 飞行/EGO 规划器稳定性分析
- `analyze_tree_pose_error.py` — `tree_pose_error` 话题数值统计

## 关键依赖
- ROS Noetic、Ubuntu 20.04、PCL >= 1.10、Eigen >= 3.3.4
- `livox_ros_driver2`（构建前必须 source）、`libarmadillo-dev`（ego_planner_swarm 必需）
- GTSAM（可选，仅 `tree_graph_node` 需要）
- `librealsense`（RealSense 相机用）；PX4 + Gazebo（仅 `Simulation/` SITL 用）

## 注意事项
- 仿真默认使用 `fake_drone` 运动学模型。如需完整 SO(3) 动力学，编辑 `plan_manage/launch/simulator.xml`
- `local_sensing` 支持可选 CUDA 加速 — 在其 CMakeLists.txt 中设置 `ENABLE_CUDA true` 启用 GPU 深度图渲染
- CPU 频率调节影响规划器性能 — 将 governor 设为 `performance` 以获得一致的计时表现

# CLAUDE.md

Behavioral guidelines to reduce common LLM coding mistakes. Merge with project-specific instructions as needed.

**Tradeoff:** These guidelines bias toward caution over speed. For trivial tasks, use judgment.

## 1. Think Before Coding

**Don't assume. Don't hide confusion. Surface tradeoffs.**

Before implementing:
- State your assumptions explicitly. If uncertain, ask.
- If multiple interpretations exist, present them - don't pick silently.
- If a simpler approach exists, say so. Push back when warranted.
- If something is unclear, stop. Name what's confusing. Ask.

## 2. Simplicity First

**Minimum code that solves the problem. Nothing speculative.**

- No features beyond what was asked.
- No abstractions for single-use code.
- No "flexibility" or "configurability" that wasn't requested.
- No error handling for impossible scenarios.
- If you write 200 lines and it could be 50, rewrite it.

Ask yourself: "Would a senior engineer say this is overcomplicated?" If yes, simplify.

## 3. Surgical Changes

**Touch only what you must. Clean up only your own mess.**

When editing existing code:
- Don't "improve" adjacent code, comments, or formatting.
- Don't refactor things that aren't broken.
- Match existing style, even if you'd do it differently.
- If you notice unrelated dead code, mention it - don't delete it.

When your changes create orphans:
- Remove imports/variables/functions that YOUR changes made unused.
- Don't remove pre-existing dead code unless asked.

The test: Every changed line should trace directly to the user's request.

## 4. Goal-Driven Execution

**Define success criteria. Loop until verified.**

Transform tasks into verifiable goals:
- "Add validation" → "Write tests for invalid inputs, then make them pass"
- "Fix the bug" → "Write a test that reproduces it, then make it pass"
- "Refactor X" → "Ensure tests pass before and after"

For multi-step tasks, state a brief plan:
```
1. [Step] → verify: [check]
2. [Step] → verify: [check]
3. [Step] → verify: [check]
```

Strong success criteria let you loop independently. Weak criteria ("make it work") require constant clarification.

---

**These guidelines are working if:** fewer unnecessary changes in diffs, fewer rewrites due to overcomplication, and clarifying questions come before implementation rather than after mistakes.
