# AGENTS.md

## 项目定位（长期约定）

Mspace 是面向**森林林冠下 GNSS 拒止环境的无人机自主导航项目，支持单机与多机协同**。后续分析、设计、代码修改和文档编写均以这一背景为准。

项目基于 Ubuntu 20.04、ROS Noetic 和 catkin，主要链路为激光雷达/IMU → Fast-LIO2 定位与建图 → EGO-Planner 规划与轨迹优化 → 控制模块及 MAVROS/PX4。支持 Livox Mid-360 和 Ouster 雷达；包含 RealSense 相机驱动，README 将相机用途描述为记录。

- 导航设计以 GNSS 不可用为前提，不默认依赖 GPS 定位或全局地理坐标。
- 森林场景需关注树干、枝叶、遮挡、点云稀疏区域及定位漂移对建图、避障和轨迹执行的影响。
- 修改公共接口时兼顾单机与多机，检查编号、命名空间、话题、坐标系、初始位置与通信配置。
- 多机能力的具体实现程度以代码和验证结果为准，不把项目目标当作已经完成的功能。

## 构建与环境

命令在 Ubuntu/WSL 的 Bash 中执行，工作目录为项目根目录。构建前加载 ROS 与外部 Livox 驱动工作区：

```bash
source /opt/ros/noetic/setup.bash
source ~/workspace/ws_livox/devel/setup.bash
./compile.sh
```

未加载 `livox_ros_driver2` 环境容易导致构建失败。README 还列出 PCL >= 1.10、Eigen >= 3.3.4 和 librealsense 等依赖。

实际构建顺序：mavros → msgs → fast_lio2 → ego_planner_swarm → swarm_control → realsense_ros。共享消息需先于依赖它的模块构建。脚本在模块失败后继续构建，最后汇总并以失败模块数作为退出码，必须检查完整结果。

单模块重建示例（先加载上述环境及已有工作区环境）：

```bash
source devel/setup.bash
catkin_make --source Modules/ego_planner_swarm --build build/ego_planner_swarm
```

各模块使用根目录 `build/` 下独立构建子目录，共用根目录 `devel/`。运行前执行 `source devel/setup.bash`。

## 模块与包名

`roslaunch` / `rosrun` 使用 `package.xml` 中的包名，不能直接使用模块目录名。

| 路径 | ROS 包名或说明 | 用途 |
| --- | --- | --- |
| `Modules/common/msgs/` | `prometheus_msgs` | 共享消息 |
| `Modules/fast_lio2/` | `fast_lio` | 激光雷达惯性里程计与建图 |
| `Modules/ego_planner_swarm/` | 包含多个包 | 规划、机间检测、通信与仿真 |
| `Modules/ego_planner_swarm/plan_manage/` | `ego_planner` | 规划管理与启动配置 |
| `Modules/ego_planner_swarm/drone_detect_lidar/` | `drone_detect_lidar` | 雷达机间检测 |
| `Modules/ego_planner_swarm/rosmsg_tcp_bridge/` | `rosmsg_tcp_bridge` | 多机消息桥接 |
| `Modules/swarm_control/` | `prometheus_swarm_control` | 控制与地面站 |
| `Modules/realsense_ros/` | 包含 `realsense2_camera` 等包 | 相机驱动 |
| `Experiment/mavros/` | `mavros_bringup` | PX4 MAVROS 启动配置 |
| `Simulation/mspace_drone/` | `mspace_drone` | 额外仿真工具 |

规划子包直接位于 `Modules/ego_planner_swarm/` 下，不存在原说明中的 `src/planner/` 层级：

- `plan_manage`：高层规划管理、launch 与参数配置。
- `bspline_opt`：B 样条轨迹优化。
- `path_searching`：A* 路径搜索。
- `plan_env`：栅格地图与射线投射等环境表示。
- `traj_utils`：轨迹工具。
- `drone_detect_lidar`：当前参与构建的雷达机间检测。
- `drone_detect`：存在 `CATKIN_IGNORE`，不参与当前构建。
- `rosmsg_tcp_bridge`：多机通信桥接。
- `uav_simulator`：地图生成、传感器模拟、无人机模型与可视化。

## 仿真运行

加载环境后使用以下入口：

```bash
roslaunch ego_planner 1uav_mid360_sim.launch              # 单机 Mid-360
roslaunch ego_planner 1uav_os128_sim.launch               # 单机 Ouster OS2-128
roslaunch ego_planner 2uav_mid360_sim.launch              # 双机
roslaunch ego_planner 4uav_mid360_sim.launch              # 四机
roslaunch ego_planner 10uav_mid360_sim.launch             # 十机
roslaunch drone_detect_lidar 2uav_lidar_detect_sim.launch # 双机雷达检测
```

规划启动配置分为 `plan_manage/launch/` 和 `plan_manage/launch_new/`；上述 Mid-360/Ouster 入口及实机 `real_ego_run.launch` 位于 `launch_new/`。

部分仿真依赖仓库外的森林 PCD 地图。例如 `1uav_mid360_sim.launch` 的 `map_name` 指向 `$(env HOME)/bagfiles/resource/plot2/Largeforest-2_38_32.pcd`。运行前检查对应入口的地图路径与文件。

`uav_simulator/fake_drone/` 对应包 `poscmd_2_odom`，用于轻量运动学仿真；完整 SO(3) 模型位于 `so3_quadrotor_simulator`，实际选用模型以入口及 include 文件为准。

## 实机与多机运行

现有脚本含固定工作区路径和网络配置，部署前核对：

```bash
./ego_fastlio2_one_livox.sh     # 单机 Livox 完整链路
./ego_fastlio2_ouster.sh        # 单机 Ouster，另需外部 Ouster 驱动环境
./ego_fastlio2_swarm_mavros.sh  # 主机1：ROS Master、本机 MAVROS，并通过 SSH 启动 UAV2 MAVROS
./ego_fastlio2_swarm_uav1.sh    # 主机1：UAV1 感知、LIO、检测、规划、控制、桥接和地面站等
./ego_fastlio2_swarm_uav2.sh    # 主机2：UAV2 对应链路，不启动地面站
```

单机与多机 UAV 脚本通过 `gnome-terminal` 打开多个终端；多机 MAVROS 脚本使用后台进程与 SSH，本身不启动规划器。

- 脚本加载 `/opt/ros/noetic/`、`~/workspace/ws_livox/` 和 `~/workspace/mspace/` 环境，路径变化时同步调整。
- Ouster 脚本另有固定的外部驱动工作区路径，需要确认安装位置。
- 当前实机多机脚本配置为两机，扩展机数需同步调整启动和通信配置，不能直接等同于多机仿真入口。
- 核对 `ROS_MASTER_URI`、`ROS_IP`、主机与广播地址、飞控连接、`uav_id`/`drone_id`、`swarm_num` 和初始位置。现有编号示例从 1 开始，具体约束以对应模块为准。
- 修改多机定位、检测或规划时检查各机坐标系与初始偏移，避免将独立 LIO 坐标直接当作统一全局坐标。
- 多机 MAVROS 脚本依赖 SSH/sshpass，退出时会清理相关进程；阅读后再运行，文档和日志不要复制其凭据。

## 共享消息与验证

共享消息位于 `Modules/common/msgs/msg/`，包括 `DroneState.msg`、`SwarmCommand.msg`、`ControlCommand.msg`、`Formation.msg`、`BoundingBox.msg`。修改后先重建 msgs，再重建相关模块，核对发布端、订阅端及桥接。

- 当前主要采用 catkin 编译及 ROS 仿真/实机链路验证，没有统一的项目级自动化测试流程；编译通过不代表飞行性能已验证。
- 代码修改至少构建相关模块；涉及公共消息、跨模块接口或构建配置时执行 `./compile.sh` 并检查汇总。
- launch/参数修改检查包名、include 路径、参数、话题与坐标系，具备条件时验证对应单机或多机仿真。
- 仅修改文档时核对描述和命令，无需完整编译或启动飞行节点。
- 当前启用的规划模块为 `Modules/ego_planner_swarm/`；compile.sh 中 `Modules/ego_planner` 构建项被注释。
- `Simulation/mspace_drone/` 不在 compile.sh 中，需按需单独构建并检查 PX4 SITL 等依赖。
- 当前 `.gitignore` 的 `AGENTS.md` 忽略项被注释，文件未被该规则忽略，跟踪状态以 `git status` 为准。
- 保留已有用户修改，不擅自提交、恢复或改动无关文件。
