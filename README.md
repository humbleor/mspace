# Mspace

[![zread](https://img.shields.io/badge/Ask_Zread-_.svg?style=flat&color=00b0aa&labelColor=000000&logo=data%3Aimage%2Fsvg%2Bxml%3Bbase64%2CPHN2ZyB3aWR0aD0iMTYiIGhlaWdodD0iMTYiIHZpZXdCb3g9IjAgMCAxNiAxNiIgZmlsbD0ibm9uZSIgeG1sbnM9Imh0dHA6Ly93d3cudzMub3JnLzIwMDAvc3ZnIj4KPHBhdGggZD0iTTQuOTYxNTYgMS42MDAxSDIuMjQxNTZDMS44ODgxIDEuNjAwMSAxLjYwMTU2IDEuODg2NjQgMS42MDE1NiAyLjI0MDFWNC45NjAxQzEuNjAxNTYgNS4zMTM1NiAxLjg4ODEgNS42MDAxIDIuMjQxNTYgNS42MDAxSDQuOTYxNTZDNS4zMTUwMiA1LjYwMDEgNS42MDE1NiA1LjMxMzU2IDUuNjAxNTYgNC45NjAxVjIuMjQwMUM1LjYwMTU2IDEuODg2NjQgNS4zMTUwMiAxLjYwMDEgNC45NjE1NiAxLjYwMDFaIiBmaWxsPSIjZmZmIi8%2BCjxwYXRoIGQ9Ik00Ljk2MTU2IDEwLjM5OTlIMi4yNDE1NkMxLjg4ODEgMTAuMzk5OSAxLjYwMTU2IDEwLjY4NjQgMS42MDE1NiAxMS4wMzk5VjEzLjc1OTlDMS42MDE1NiAxNC4xMTM0IDEuODg4MSAxNC4zOTk5IDIuMjQxNTYgMTQuMzk5OUg0Ljk2MTU2QzUuMzE1MDIgMTQuMzk5OSA1LjYwMTU2IDE0LjExMzQgNS42MDE1NiAxMy43NTk5VjExLjAzOTlDNS42MDE1NiAxMC42ODY0IDUuMzE1MDIgMTAuMzk5OSA0Ljk2MTU2IDEwLjM5OTlaIiBmaWxsPSIjZmZmIi8%2BCjxwYXRoIGQ9Ik0xMy43NTg0IDEuNjAwMUgxMS4wMzg0QzEwLjY4NSAxLjYwMDEgMTAuMzk4NCAxLjg4NjY0IDEwLjM5ODQgMi4yNDAxVjQuOTYwMUMxMC4zOTg0IDUuMzEzNTYgMTAuNjg1IDUuNjAwMSAxMS4wMzg0IDUuNjAwMUgxMy43NTg0QzE0LjExMTkgNS42MDAxIDE0LjM5ODQgNS4zMTM1NiAxNC4zOTg0IDQuOTYwMVYyLjI0MDFDMTQuMzk4NCAxLjg4NjY0IDE0LjExMTkgMS42MDAxIDEzLjc1ODQgMS42MDAxWiIgZmlsbD0iI2ZmZiIvPgo8cGF0aCBkPSJNNCAxMkwxMiA0TDQgMTJaIiBmaWxsPSIjZmZmIi8%2BCjxwYXRoIGQ9Ik00IDEyTDEyIDQiIHN0cm9rZT0iI2ZmZiIgc3Ryb2tlLXdpZHRoPSIxLjUiIHN0cm9rZS1saW5lY2FwPSJyb3VuZCIvPgo8L3N2Zz4K&logoColor=ffffff)](https://zread.ai/humbleor/mspace)

**Mspace 是面向森林林冠下 GNSS 拒止环境的无人机自主导航项目，支持单机与多机协同。** 项目基于 ROS Noetic，将 Fast-LIO2 激光雷达惯性定位与建图、EGO-Planner 路径规划与轨迹优化，以及 MAVROS/PX4 控制接口集成到同一 catkin 工作区。

项目提供 Livox Mid-360、Ouster 雷达相关配置，包含森林点云场景仿真、雷达机间检测、多机通信桥接和实机启动脚本。Intel RealSense 相机用于数据记录相关场景。具体功能与运行条件以代码和配置为准。

## 1. 系统组成

主要处理链路：

```text
激光雷达 + IMU → Fast-LIO2 定位/建图 → EGO-Planner 规划/轨迹优化 → 控制模块 → MAVROS/PX4
                                     ↑
                           机间检测与多机消息通信
```

导航以 GNSS 不可用为应用前提，依靠激光雷达与惯性信息提供定位。多机运行还需配置各机编号、初始位置、坐标系及通信网络；各机独立 LIO 坐标不能直接视为统一全局坐标。

| 目录                                              | 功能 / 主要 ROS 包                           |
| ------------------------------------------------- | -------------------------------------------- |
| `Modules/common/msgs/`                          | 共享消息，`prometheus_msgs`                |
| `Modules/fast_lio2/`                            | 激光雷达惯性里程计，`fast_lio`             |
| `Modules/ego_planner_swarm/`                    | 规划、环境表示、检测、通信与仿真，包含多个包 |
| `Modules/ego_planner_swarm/plan_manage/`        | 规划管理及启动配置，`ego_planner`          |
| `Modules/ego_planner_swarm/drone_detect_lidar/` | 雷达机间检测，`drone_detect_lidar`         |
| `Modules/ego_planner_swarm/rosmsg_tcp_bridge/`  | 多机消息桥接，`rosmsg_tcp_bridge`          |
| `Modules/swarm_control/`                        | 控制与地面站，`prometheus_swarm_control`   |
| `Modules/realsense_ros/`                        | RealSense 驱动，包含`realsense2_camera`    |
| `Experiment/mavros/`                            | MAVROS 启动配置，`mavros_bringup`          |

`roslaunch` 和 `rosrun` 使用 ROS 包名。例如规划模块目录为 `ego_planner_swarm`，启动时使用包名 `ego_planner`。

## 2. 环境与依赖

- Ubuntu 20.04、ROS Noetic、catkin。
- PCL >= 1.10、Eigen >= 3.3.4。
- [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2)：需在外部工作区预先构建，现有脚本使用 `~/workspace/ws_livox/`。
- [librealsense](https://github.com/IntelRealSense/librealsense)：RealSense 驱动依赖。
- MAVROS 及对应 ROS 包依赖；实机需 PX4 飞控与正确的连接配置。
- Ouster 实机需额外安装并构建 Ouster ROS 驱动；当前启动脚本包含固定的外部驱动路径，需要按本机环境调整。
- 实机集成脚本使用 `gnome-terminal`；多机 MAVROS 脚本还使用 SSH 和 `sshpass`。

以下命令在 Ubuntu/WSL 的 Bash 中执行。实机部署需具备传感器、飞控、网络和图形终端等条件。

## 3. 获取与构建

现有实机脚本默认项目位于 `~/workspace/mspace`：

```bash
mkdir -p ~/workspace
cd ~/workspace
git clone https://github.com/humbleor/mspace.git
cd mspace

source /opt/ros/noetic/setup.bash
source ~/workspace/ws_livox/devel/setup.bash
./compile.sh
source devel/setup.bash
```

构建前必须加载 `livox_ros_driver2` 工作区环境。若使用其他安装路径，同步修改加载命令和实机脚本。

`compile.sh` 按以下顺序构建：

```text
mavros → msgs → fast_lio2 → ego_planner_swarm → swarm_control → realsense_ros
```

各模块使用根目录 `build/` 下的独立子目录，共用根目录 `devel/`。脚本遇到模块失败后仍继续构建，最终打印汇总，并以失败模块数作为退出码；请检查所有模块的结果。

单独重建规划模块（先加载 ROS、Livox 和已有工作区环境）：

```bash
catkin_make --source Modules/ego_planner_swarm --build build/ego_planner_swarm
```

## 4. 森林场景仿真

在新终端中加载环境：

```bash
source /opt/ros/noetic/setup.bash
source ~/workspace/ws_livox/devel/setup.bash
source ~/workspace/mspace/devel/setup.bash
```

### 地图准备

部分入口依赖仓库外部的森林 PCD 地图。以 `Modules/ego_planner_swarm/plan_manage/launch_new/1uav_mid360_sim.launch` 为例，其 `map_name` 当前指向：

```text
$(env HOME)/bagfiles/resource/plot2/Largeforest-2_38_32.pcd
```

运行前准备地图，或修改相应 launch 文件中的 `map_name`，并核对地图偏移、地图范围、初始位置及航点范围。不同入口的配置可能不同。

### 启动入口

按需要选择一个入口：

```bash
roslaunch ego_planner 1uav_mid360_sim.launch              # 单机 Livox Mid-360
roslaunch ego_planner 1uav_os128_sim.launch               # 单机 Ouster OS2-128
roslaunch ego_planner 2uav_mid360_sim.launch              # 双机 Mid-360
roslaunch ego_planner 4uav_mid360_sim.launch              # 四机 Mid-360
roslaunch ego_planner 10uav_mid360_sim.launch             # 十机 Mid-360
roslaunch drone_detect_lidar 2uav_lidar_detect_sim.launch # 双机雷达检测仿真
```

Mid-360/Ouster 规划入口位于 `plan_manage/launch_new/`，原有配置位于 `plan_manage/launch/`。仿真模型由入口及 include 文件决定：`uav_simulator/fake_drone/` 对应轻量运动学包 `poscmd_2_odom`，另有 `so3_quadrotor_simulator` 动力学模型。

## 5. 实机部署

### 单机

先核对工作区路径、雷达驱动、雷达与 IMU 参数、飞控连接，以及规划和控制配置，再选择启动脚本：

```bash
./ego_fastlio2_one_livox.sh # Livox Mid-360
./ego_fastlio2_ouster.sh   # Ouster
```

脚本通过多个终端启动 MAVROS、雷达驱动、Fast-LIO2、控制、规划、地面站和相机。实机规划入口为 `ego_planner` 包的 `launch_new/real_ego_run.launch`。

### 双机协同

当前实机多机脚本配置为两台无人机，采用多主机 ROS 网络，并启动机间雷达检测与 `rosmsg_tcp_bridge`。四机、十机仿真入口不代表已有相同规模的实机部署脚本。

启动前同步检查各脚本中的：

- `ROS_MASTER_URI`、`ROS_IP`、主机地址、广播地址和 SSH 配置。
- 飞控连接、`uav_id`/`drone_id`、`swarm_num` 和初始位置。
- 各机坐标系、初始偏移、话题命名与桥接配置。
- 工作区路径与外部驱动环境。

**主机 1：** 在一个终端启动 ROS Master 和两机 MAVROS：

```bash
./ego_fastlio2_swarm_mavros.sh
```

该脚本在本机启动 UAV1 MAVROS，通过 SSH 在主机 2 启动 UAV2 MAVROS，并持续等待后台进程；本身不启动规划器。另开终端启动 UAV1 导航链路：

```bash
./ego_fastlio2_swarm_uav1.sh
```

**主机 2：** 启动 UAV2 导航链路：

```bash
./ego_fastlio2_swarm_uav2.sh
```

UAV1 脚本启动地面站，UAV2 不重复启动。MAVROS 多机脚本退出时会执行进程清理，部署前应阅读脚本并按实际设备修改配置。

## 6. 数据记录

`topiclistOS0.sh` 使用 `rosbag record` 记录 Ouster IMU/点云、匹配的 Livox 雷达/IMU 话题，以及指定的压缩彩色图像话题。先加载 ROS 环境，并确保记录目录存在：

```bash
mkdir -p ~/data
./topiclistOS0.sh
```

运行前通过 `rostopic list` 核对实际话题，按需要调整脚本中的记录列表。bag 文件写入 `~/data`。

## 7. 开发与排查

- **找不到 Livox 依赖：** 确认外部驱动已构建，并先加载 `ws_livox/devel/setup.bash`。
- **找不到 ROS 包：** 确认对应模块构建成功，加载项目 `devel/setup.bash`，并使用正确的包名。
- **仿真地图无法加载：** 检查对应入口的 PCD 路径、文件及地图配置。
- **多机通信异常：** 检查 ROS Master、各机可达地址、消息桥接配置、编号与话题。
- **共享消息变更：** 先重建 `Modules/common/msgs/`，再重建相关模块，核对发布端、订阅端和桥接。

当前默认构建启用 `Modules/ego_planner_swarm/`；旧 `Modules/ego_planner` 构建项被注释。`drone_detect` 存在 `CATKIN_IGNORE`，当前参与构建的是 `drone_detect_lidar`。

项目主要采用模块编译及 ROS 仿真/实机链路验证，没有统一的项目级自动化测试流程。编译通过仅说明构建成功，导航效果需结合森林场景中的定位、建图、避障、轨迹执行和多机运行进一步验证。

## 8. 森林雷达 / LIO 离线基准

使用实测 Mid-360 bag 建立原始数据审计、隔离回放、点云场景传感器生成和已知轨迹验证。
运行入口与限制见 [使用说明](Simulation/forest_lio/README.md)，实际验证结果见
[第一阶段实验报告](artifacts/forest_lio/historical_reports/forest_lio_phase1.md)。该离线基准用于建立参考数据，不启动飞控控制节点；在线闭环入口见下文。

第二阶段已接入在线 PCD → Mid-360/IMU → Fast-LIO2 → EGO → 简化运动反馈闭环；运行步骤与限制见 [工具说明](Simulation/forest_lio/README.md)，实跑结果见 [第二阶段报告](artifacts/forest_lio/historical_reports/forest_lio_phase2.md)。

森林传感器现支持 BVH 候选面查询与逐点位姿；验证及性能边界见 [渲染改进报告](Simulation/forest_lio/docs/forest_lio_renderer.md)。

PX4 软件飞控在环入口已组织在 [Simulation/px4_forest](Simulation/px4_forest/README.md)，复用森林 PCD/LIO 模块并连接 Gazebo Iris 六自由度动力学。原生悬停与森林无 GPS 阶段分别验收；实际运行成功与否以每次 result.json 为准。

一键启动森林仿真：`bash Simulation/start.sh`（完整 15 点任务），短程检查：`bash Simulation/start.sh short`。

仿真代码已统一到 [Simulation](Simulation/README.md)：`python3 Simulation/run.py` 提供数据准备、轻量闭环和 `px4` 物理闭环入口，完整流程与启动命令见该目录说明。
