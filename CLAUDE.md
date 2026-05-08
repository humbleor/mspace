# CLAUDE.md

本文件为 Claude Code (claude.ai/code) 提供在此仓库中工作的指引。

## 项目概述

**Mspace** 是一个基于 ROS Noetic 的无人机自主飞行项目，集成了 **Fast-LIO2**（激光-惯性里程计）和 **EGO-Planner Swarm**（去中心化多机器人路径规划）。支持单机和蜂群（多 UAV）配置，适配 Livox Mid-360 和 Ouster 激光雷达。

## 构建

始终使用根目录下的 `./compile.sh` 构建整个项目：

```bash
./compile.sh
```

构建产物输出到**项目根目录**下的 `build/` 和 `devel/`，而不是各个模块内部：
- `build/<module_name>/` — 编译中间文件
- `devel/` — 共享开发空间，包含 `setup.bash`

单独重建某个模块（`compile.sh` 内部也是同样的命令）：
```bash
catkin_make --source Modules/ego_planner_swarm --build build/ego_planner_swarm
```

启动前务必 source 项目根目录的 `devel/setup.bash`：
```bash
source devel/setup.bash
```

## 代码架构

仓库代码组织在 `Modules/` 下，每个子目录是一个 catkin 包或工作空间：

### Modules/ego_planner_swarm — 核心规划与仿真
最大的模块，fork 自 ZJU-FAST-Lab/ego-planner-swarm。包含：

- **src/planner/plan_manage** — 高层规划调度，launch 文件（`simple_run.launch`、`swarm.launch`、`single_run_in_sim.launch` 等），仿真器接线
- **src/planner/plan_env** — 环境感知：`grid_map`（体素占据栅格）、`obj_predictor`（动态障碍物预测）、`raycast`（光线投射）
- **src/planner/bspline_opt** — 轨迹优化：`UniformBSpline`、`BsplineOptimizer`（碰撞/平滑/可行性梯度下降优化）、`GradientDescentOptimizer`
- **src/planner/path_searching** — `DynAStar`（运动学 A* 路径搜索）
- **src/planner/traj_utils** — 轨迹工具函数
- **src/planner/drone_detect** — 基于深度/相机数据的无人机检测
- **src/planner/drone_detect_lidar** — 基于树干特征匹配的多机相对定位（森林场景）。高度裁剪 + PCA 线性度筛选树干 + 距离匹配 + 2D SVD 闭式解求 (dx, dy, yaw)，输出 `tree_pose_error` 话题
- **src/planner/rosmsg_tcp_bridge** — 基于 TCP 的 ROS 消息桥接，用于蜂群通信
- **src/uav_simulator/** — 仿真基础设施：
  - `local_sensing` — 仿真深度相机 / 点云传感器（CPU/GPU 模式）
  - `mockamap` — 程序化地图生成
  - `fake_drone` — 轻量级运动学无人机模型（默认，低 CPU 占用）
  - `so3_control` + `so3_quadrotor_simulator` — 完整 SO(3) 动力学仿真（可选，CPU 占用高）
  - `map_generator` / `lidar_map_generator` — 测试地图生成器
  - `Utils` — 通用仿真工具

### Modules/fast_lio2 — 激光-惯性里程计
包含 `FAST_LIO` — 紧耦合激光-惯性里程计系统：
- 核心文件：`drone_laserMapping.cpp`（主建图节点）、`preprocess.cpp`（点云预处理）、`IMU_Processing.hpp`（IMU 积分）
- 配置文件在 `config/`，launch 文件在 `launch/`

### Modules/swarm_control — 真实蜂群执行
用于物理硬件上部署蜂群任务的节点：
- `swarm_terminal_control.cpp` — 终端/CLI 任务控制接口
- `swarm_controller.cpp` — 高层蜂群协调
- `swarm_formation_control.cpp` — 编队保持
- `swarm_estimator.cpp` — 蜂群成员状态估计
- `swarm_ground_station.cpp` — 地面站 UI/遥测
- `ego_traj_to_cmd.cpp` — 将 EGO-Planner 轨迹转换为 MAVROS 指令

### Modules/common/msgs — 共享消息定义
跨模块使用的自定义 ROS msg/action 定义。

### Modules/realsense_ros — Intel RealSense 驱动
Fork 的 RealSense ROS 驱动，支持 D435/D435i 相机。

### Experiment/mavros — MAVROS 集成
自定义 MAVROS launch/配置，用于真实环境部署。

## 启动

### 仿真
```bash
# 单机
roslaunch ego_planner simple_run.launch

# 蜂群（4 架 UAV）
roslaunch ego_planner swarm.launch
```

### 真机（单机，Livox Mid-360）
```bash
./ego_fastlio2_one_livox.sh
```

### 真机（单机，Ouster）
```bash
./ego_fastlio2_ouster.sh
```

### 真机（蜂群）
主机 1：
```bash
./ego_fastlio2_swarm_mavros.sh
./ego_fastlio2_swarm_uav1.sh
```
主机 2：
```bash
./ego_fastlio2_swarm_uav2.sh
```

## 脚本
- `Scripts/visualize_waypoints.py` — 航点可视化
- `Scripts/analyze_flight_stability.py` — 飞行稳定性分析
- `Scripts/analyzer_ego_stability.py` — EGO 规划器稳定性分析

## 关键依赖
- ROS Noetic, Ubuntu 20.04
- PCL >= 1.10, Eigen >= 3.3.4
- `libarmadillo-dev`（ego_planner_swarm 必需）
- `livox_ros_driver2`（构建前必须先 source）
- `librealsense`（RealSense 相机用）

## 注意事项
- 仿真默认使用 `fake_drone`（运动学模型，低 CPU 占用）。如需启用完整 SO(3) 动力学仿真，编辑 `src/planner/plan_manage/launch/simulator.xml`。
- `local_sensing` 支持可选的 CUDA 加速 — 在其 CMakeLists.txt 中设置 `ENABLE_CUDA true` 可启用 GPU 深度图渲染。
- CPU 频率调节会影响规划器性能 — 将 governor 设为 `performance` 以获得一致的计时表现。

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
