# EGO-Swarm 森林场景规划仿真

本说明对应 Mspace 中的 EGO-Swarm 单机 / 多机规划与雷达机间检测仿真入口。单机 Mid-360 入口通过模拟点云和 `poscmd_2_odom` 提供感知输入与运动反馈，用于检查规划、避障和轨迹执行；完整 Fast-LIO2 与 PX4/Gazebo 链路见 [统一仿真说明](../../Simulation/README.md)。

模块上游介绍见 [README](README.md)，项目依赖与构建步骤见 [项目 README](../../README.md)。以下命令在项目构建完成后，于 Ubuntu/WSL Bash 中执行。

在新终端中加载环境：

```bash
source /opt/ros/noetic/setup.bash
source ~/workspace/ws_livox/devel/setup.bash
source ~/workspace/mspace/devel/setup.bash
```

## 地图准备

部分入口依赖仓库外部的森林 PCD 地图。以 `plan_manage/launch_new/1uav_mid360_sim.launch` 为例，其 `map_name` 当前指向：

```text
$(env HOME)/bagfiles/resource/plot2/Largeforest-2_38_32.pcd
```

运行前准备地图，或修改相应 launch 文件中的 `map_name`，并核对地图偏移、地图范围、初始位置及航点范围。不同入口的配置可能不同。

## 启动入口

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
