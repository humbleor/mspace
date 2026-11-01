# Mspace

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

| 目录                                            | 功能 / 主要 ROS 包                           |
| ----------------------------------------------- | -------------------------------------------- |
| `Modules/common/msgs/`                          | 共享消息，`prometheus_msgs`                  |
| `Modules/fast_lio2/`                            | 激光雷达惯性里程计，`fast_lio`               |
| `Modules/ego_planner_swarm/`                    | 规划、环境表示、检测、通信与仿真，包含多个包 |
| `Modules/ego_planner_swarm/plan_manage/`        | 规划管理及启动配置，`ego_planner`            |
| `Modules/ego_planner_swarm/drone_detect_lidar/` | 雷达机间检测，`drone_detect_lidar`           |
| `Modules/ego_planner_swarm/rosmsg_tcp_bridge/`  | 多机消息桥接，`rosmsg_tcp_bridge`            |
| `Modules/swarm_control/`                        | 控制与地面站，`prometheus_swarm_control`     |
| `Modules/realsense_ros/`                        | RealSense 驱动，包含`realsense2_camera`      |
| `Experiment/mavros/`                            | MAVROS 启动配置，`mavros_bringup`            |

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

## 4. EGO-Swarm 规划仿真

单机 / 多机森林场景规划与雷达机间检测仿真的环境加载、地图准备和启动命令，见 [EGO-Swarm 规划仿真说明](Modules/ego_planner_swarm/README_simulation.md)。

包含 Fast-LIO2 的森林传感器闭环与 PX4/Gazebo 物理闭环见第 8 节。

## 5. 实机部署

### 单机

先核对工作区路径、雷达驱动、雷达与 IMU 参数、飞控连接，以及规划和控制配置，再选择启动脚本：

```bash
./ego_fastlio2_one_livox.sh # Livox Mid-360，默认 world 模式（公共坐标）
./ego_fastlio2_ouster.sh   # Ouster
```

脚本通过多个终端启动 MAVROS、雷达驱动、Fast-LIO2、控制、规划、地面站和相机。实机规划入口为 `ego_planner` 包的 `launch_new/real_ego_run.launch`。

Livox 单机脚本默认启用公共坐标 world 模式：MAVROS 本地 frame 为 `uav1/fcu_local`，Fast-LIO2 输出 `uav1/lio_init`，`swarm_frame_adapter` 把两者换算到同一个 `world`，规划器改用 `world/odom` 与 `world/cloud`。起飞前需静止约 3 秒，`/uav1/frame_ready` 为 true 后控制器才放行任务。回退旧链路（两套独立 `world`，仅供对比）：`WORLD_FRAME=0 ./ego_fastlio2_one_livox.sh`。说明见 [WORLD_FRAME.md](Modules/swarm_control/WORLD_FRAME.md)。

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

`topiclistOS0.sh` 用 `rosbag record` 只录需要的传感器话题：Ouster/Livox 雷达点云与 IMU、压缩彩色图。不开 `-a`，避免把 `/rosout`、MAVROS 诊断、大体积派生点云一起录进来。bag 写入 `~/data`，`--duration=60` 是每 60 秒切一片，Ctrl+C 结束。

```bash
source /opt/ros/noetic/setup.bash && source ~/workspace/mspace/devel/setup.bash
CHECK_ONLY=1 ./topiclistOS0.sh   # 先看会录哪些话题，不写文件
./topiclistOS0.sh
```

正则按整名匹配，同时兼容旧命名与 world 模式：`/livox/lidar_*`、`/uav1/livox/lidar_*`、`/ouster/points`、`/os_cloud_node_1/points`，以及 `/camera`、`/d400` 的压缩图。雷达话题换了名字或前缀时，改正则为上；相机用 `camera:=uav1_camera` 启动的话，把 `uavN_camera` 加进去。

回放注意：world 模式录出的 bag 里 frame 是 `uavX/lio_init` / `uavX/fcu_local`，要用 `enable_world_frame:=true` 的 launch 回放。

## 7. 森林传感器与导航闭环仿真

仿真工具统一位于 [Simulation](Simulation/README.md)，通过 `python3 Simulation/run.py` 提供以下流程：

- **离线数据准备与 LIO 基准：** 使用实测 Mid-360 bag 进行原始数据审计、隔离回放、森林 PCD 地图与场景生成，以及已知模拟轨迹下的 LIO 验证。具体命令与限制见 [forest_lio 使用说明](Simulation/forest_lio/README.md)。
- **轻量在线闭环：** PCD → Mid-360/IMU → Fast-LIO2 → EGO → 简化运动反馈，用于检查传感器、定位与规划的在线联通。运行步骤与模型限制见 [forest_lio 使用说明](Simulation/forest_lio/README.md)。
- **PX4 软件飞控在环（SITL）：** 复用森林 PCD/LIO 模块，连接 MAVROS、PX4 与 Gazebo Iris 六自由度动力学。环境准备、构建和运行步骤见 [PX4 森林仿真说明](Simulation/px4_forest/README.md)，验收方法与适用边界见 [验证说明](Simulation/px4_forest/docs/validation.md)。

完成依赖安装、项目与 PX4 构建，并准备好森林地图、扫描模板和场景文件后，在项目根目录执行：

```bash
bash Simulation/start.sh short # 短程检查
bash Simulation/start.sh       # 完整 15 点任务
```

首次使用或更换数据时，先按 [统一仿真说明](Simulation/README.md) 完成数据准备；默认启动所需的地图和场景文件不随仓库提供。

森林传感器的 BVH 候选面查询、逐点位姿及性能边界见 [渲染说明](Simulation/forest_lio/docs/forest_lio_renderer.md)，地图与观测覆盖限制见 [地图与几何说明](Simulation/forest_lio/docs/forest_lio_map_geometry.md)。

`artifacts/forest_lio/` 和 `artifacts/px4_forest/` 用于保存本地生成的数据、日志与实验报告，已由 Git 忽略，克隆仓库后不会包含这些文件。运行后从本次输出目录查看 `result.json`、`report.md` 等结果；各阶段是否通过以对应验收结果为准，启动成功不等于导航验收通过。
