# swarm_control — 无人机群控制模块

**ROS 包名:** `prometheus_swarm_control`

## 概述

`swarm_control` 是连接上层规划器（EGO-Planner Swarm）与底层飞控（PX4 via MAVROS）的**中层控制模块**。负责：
- **状态估计**：融合激光/视觉里程计与飞控 IMU，发布融合位姿供规划器使用
- **指令转换**：将 EGO 规划的 B 样条轨迹转换为飞控可执行的 SwarmCommand
- **飞行控制**：状态机驱动的全流程飞行管理（解锁→起飞→任务→降落→上锁）
- **蜂群编队**：集中式/分布式多机编队控制

## 架构

```
                       ┌──────────────────────────┐
                       │    EGO-Planner Swarm      │
                       │  (plan_manage/traj_server)│
                       │                          │
                       │  quadrotor_msgs/         │
                       │  PositionCommand         │
                       └──────────┬───────────────┘
                                  │ /uavX/planning/ego/traj_cmd
                                  ▼
┌─────────────────────────────────────────────────────────────────────┐
│                        swarm_control 模块                            │
│                                                                      │
│  ┌─────────────────┐    ┌──────────────────┐    ┌────────────────┐  │
│  │ swarm_estimator  │    │ ego_traj_to_cmd  │    │ swarm_controller│  │
│  │                  │    │                  │    │                │  │
│  │ 定位源→DroneState│    │ PositionCommand  │    │ SwarmCommand   │  │
│  │ →MAVROS vision   │    │ →SwarmCommand    │    │ →PositionTarget│  │
│  │ →odom/rviz       │    │ (Mode=Move,      │    │ →飞控          │  │
│  │                  │    │  Move_mode=      │    │                │  │
│  │                  │    │  TRAJECTORY)     │    │ 控制模式:      │  │
│  │                  │    │                  │    │ ·姿态环(自研)  │  │
│  │                  │    │                  │    │ ·位置环(PX4)   │  │
│  └────────┬────────┘    └────────┬─────────┘    └───────┬────────┘  │
│           │                      │                      │           │
│  ┌────────┴──────────────────────┴──────────────────────┴────────┐  │
│  │  辅助节点                                                       │  │
│  │  · swarm_ground_station    — 地面站状态监控                     │  │
│  │  · swarm_terminal_control  — 终端手动指令 (调试用)              │  │
│  │  · swarm_formation_control — 蜂群编队终端 (调试用)              │  │
│  └────────────────────────────────────────────────────────────────┘  │
└──────────────────────────────────────────────────────────────────────┘
                                  │ /uavX/mavros/setpoint_raw/local
                                  ▼
                          ┌──────────────┐
                          │    MAVROS     │
                          │  setpoint_raw │
                          └──────┬───────┘
                                 │ MAVLink SET_POSITION_TARGET_LOCAL_NED
                                 ▼
                          ┌──────────────┐
                          │   PX4 飞控    │
                          │ FlightTask    │
                          │ Offboard      │
                          └──────────────┘
```

## 节点说明

### 1. `swarm_estimator` — 状态估计器

融合定位源与飞控数据，发布统一的无人机状态。

| 方向 | 话题 | 消息类型 | 说明 |
|------|------|----------|------|
| **订阅** | `/uavX/mavros/state` | `mavros_msgs/State` | 飞控状态 (连接/解锁/模式) |
| **订阅** | `/uavX/mavros/local_position/pose` | `PoseStamped` | 飞控本地位置 (ENU) |
| **订阅** | `/uavX/mavros/local_position/velocity_local` | `TwistStamped` | 飞控本地速度 (ENU) |
| **订阅** | `/uavX/mavros/imu/data` | `sensor_msgs/Imu` | 飞控 IMU 姿态 |
| **订阅** | 定位源 (根据 `input_source`) | | |
| | · `input_source=0` | `PoseStamped` | Mocap (`/vrpn_client_node/uavX/pose`) |
| | · `input_source=1` | `Odometry` | Faster-LIO (`/Odometry`) |
| | · `input_source=2` | `Odometry` | Gazebo 真值 |
| | · `input_source=4` | `Odometry` | **Fast-LIO2** (`/drone_Odom_high_freq`) |
| **发布** | `/uavX/mavros/vision_pose/pose` | `PoseStamped` | 视觉定位 (50Hz) → 飞控 EKF |
| **发布** | `/uavX/prometheus/drone_state` | `DroneState` | 融合状态 → 上层节点 |
| **发布** | `/uavX/prometheus/drone_odom` | `Odometry` | Rviz 可视化 |

**参数:**
| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `uav_id` | int | 0 | 无人机编号 |
| `input_source` | int | 0 | 定位源: 0=mocap, 1=faster-lio, 2=gazebo, 4=fast-lio2 |
| `offset_x/y/z` | float | 0 | 定位设备安装偏移 [m] |

### 2. `ego_traj_to_cmd` — EGO 轨迹 → 控制指令桥接

**这是 EGO planner 与 swarm_control 的接口节点**，负责将 EGO 规划的连续轨迹转换为 `SwarmCommand`。

| 方向 | 话题 | 消息类型 | 说明 |
|------|------|----------|------|
| **订阅** | `/uavX/planning/ego/traj_cmd` | `quadrotor_msgs/PositionCommand` | EGO 采样轨迹 |
| **发布** | `/uavX/prometheus/swarm_command` | `SwarmCommand` | 控制指令 → swarm_controller |

**工作流程:**
1. 接收 EGO 的 `PositionCommand`（含 position、velocity、acceleration、yaw）
2. 根据 `control_flag` 参数设置 `Move_mode` 并填充 `position_ref`、`velocity_ref`、`acceleration_ref`、`yaw_ref`
3. 发布 `SwarmCommand` 到 swarm_controller

**参数:**
| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `swarm_num` | int | 1 | 集群数量 |
| `uav_id` | int | 0 | 无人机编号 |
| `control_flag` | int | 0 | 0=TRAJECTORY(全量轨迹), 1=XYZ_POS(纯位置), 2=XY_VEL_Z_POS |

> **注意:** 当 EGO 发布的速度为 0（到达目标点）时，该帧不会转为控制指令（`get_ego_traj=false`），防止飞控接收到零速指令导致悬停抖动。

### 3. `swarm_controller` — 核心飞行控制器

模块的核心节点，包含两个定时器回调：

**`mainloop_cb` (10 Hz)** — 状态机与任务调度:
- 读取当前 `SwarmCommand` 指令
- 执行对应的飞行任务逻辑
- 计算 `pos_des`、`vel_des`、`acc_des`、`yaw_des`

**`control_cb` (50 Hz)** — 指令输出:
- 根据 `controller_flag` 选择控制模式
- 发布 `mavros_msgs/PositionTarget` 或 `AttitudeTarget` 到飞控

| 方向 | 话题 | 消息类型 | 说明 |
|------|------|----------|------|
| **订阅** | `/uavX/prometheus/swarm_command` | `SwarmCommand` | 控制指令 |
| **订阅** | `/uavX/prometheus/drone_state` | `DroneState` | 本机状态 |
| **订阅** | `/uavX/prometheus/drone_state` (邻居) | `DroneState` | 邻居状态 (蜂群) |
| **订阅** | `/uavX/tree_pose_error` (邻居) | `Vector3` | 树干定位漂移补偿 |
| **发布** | `/uavX/mavros/setpoint_raw/local` | `PositionTarget` | 位置/速度指令 → MAVROS |
| **发布** | `/uavX/mavros/setpoint_raw/attitude` | `AttitudeTarget` | 姿态指令 → MAVROS |
| **服务** | `/uavX/mavros/cmd/arming` | `CommandBool` | 解锁/上锁 |
| **服务** | `/uavX/mavros/set_mode` | `SetMode` | 切换飞行模式 |

#### 支持的控制模式

**SwarmCommand.Mode (主模式):**

| Mode | 说明 |
|------|------|
| `Idle` (0) | 怠速。发送空指令维持 Offboard，可通过 yaw_ref=999 解锁切模式 |
| `Takeoff` (1) | 从当前位置起飞到 `Takeoff_height` 高度（参数可配） |
| `Hold` (2) | 悬停，锁定当前位置 |
| `Land` (3) | 降落。到达 `Disarm_height` 后自动切 Manual + 上锁 |
| `Disarm` (4) | 紧急上锁 |
| `Move` (8) | 单机运动（EGO 轨迹走此模式） |
| `Position_Control` (5) | 集中式编队位置控制 |
| `Velocity_Control` (6) | 分布式一致性编队 |
| `Accel_Control` (7) | 加速度控制编队 |

**Move_mode (Move 的子模式):**

| Move_mode | 值 | 控制输出 | EGO 场景 |
|-----------|-----|---------|---------|
| `XYZ_POS` | 0 | `send_pos_setpoint` — 位置 + yaw | 航点飞行 |
| `XY_VEL_Z_POS` | 2 | `send_pos_vel_acc_setpoint` — xy速度 + z位置 | 速度控制 |
| `TRAJECTORY` | 5 | `send_pos_vel_xyz_setpoint` — 位置 + 速度前馈 + yaw | **EGO 轨迹跟踪** |

#### 双控制策略

**controller_flag=0 — 自研姿态环:**
级联 PID 控制 `pos_controller()` → 力分解为姿态角 → `send_attitude_setpoint()`。
- 位置误差 → 速度期望 (Kp)
- 速度误差 + 积分项 → 力期望 (Kv + Kvi)
- 力期望 + 质量 + 前馈加速度 → 期望力 F_des
- F_des 分解为 roll/pitch/yaw + 油门
- 包含推力限幅 (0.5~2 倍重力)、姿态角限幅、积分饱和保护

**controller_flag=1 — PX4 位置环 (推荐):**
直接将期望值发布给飞控，由 PX4 内部控制回路执行跟踪。轨迹模式下使用**位置+速度前馈**以提升跟踪精度。

#### 蜂群编队

- **Position_Control**: 集中式 — 地面站发布虚拟领队位置 → 各机叠加 `formation_separation` 偏移量
- **Velocity_Control**: 分布式 — 双向环拓扑一致性算法 (`k_p`, `k_aij`, `k_gamma`) + APF 人工势场避碰
- 支持阵型: 一字型 (`One_column`)、三角型 (`Triangle`)、方型 (`Square`)、圆型 (`Circular`)
- 预定义 4 机和 8 机的阵型偏移量 (`formation_utils.h`)

#### 安全机制

- **地理围栏** (`geo_fence_x/y/z`): 超出围栏自动降落
- **Disarm 优先级**: 一旦收到 Disarm，屏蔽其他所有指令
- **非 Offboard 时积分清零**: 防止切模式时积分饱和
- **无人机位置约束**: 高度 < 0 时强制设为 0.01m

#### 漂移补偿 (树定位)

通过 `tree_drift_cb` 接收 `drone_detect_lidar` 模块的树干相对定位结果 `(dx, dy, yaw)`：
- EMA 滤波平滑
- 异常值拒绝（阈值 `outlier_threshold`）
- 过期检测（`max_age` 超时重置）
- 2D 刚体变换修正邻居位置，提升蜂群相对定位精度

### 4. `swarm_ground_station` — 地面站

监控所有无人机状态的控制台节点，订阅全部 UAV 的 `DroneState` 和 `SwarmCommand`，2Hz 打印状态摘要。

### 5. `swarm_terminal_control` — 终端手动控制

调试用手动控制节点，提供交互式菜单：
1. Move (XYZ_POS) — 手动输入目标位姿
2. EGO Goal — 发布目标点触发 EGO 规划 (`/uavX/prometheus/ego/goal`)
3. Hold / Land / Disarm

### 6. `swarm_formation_control` — 编队终端

多机编队调试用交互式终端，可切换阵型、控制虚拟领队位置，支持 Position_Control / Velocity_Control / Accel_Control。

## 与 EGO Planner 的联动

### EGO → swarm_control 数据流

```
EGO bspline_opt 优化器
  ↓ B样条轨迹
plan_manage/traj_server (100Hz 采样)
  ↓ quadrotor_msgs/PositionCommand
  │  · position  (x, y, z)     [m]
  │  · velocity  (vx, vy, vz)  [m/s]
  │  · acceleration (ax, ay, az) [m/s²]
  │  · yaw                       [rad]
  │  · yaw_dot                   [rad/s]
/uavX/planning/ego/traj_cmd
  ↓
ego_traj_to_cmd (本模块)
  ↓ prometheus_msgs/SwarmCommand
  │  Mode = Move
  │  Move_mode = TRAJECTORY
  │  position_ref  = ego.position
  │  velocity_ref  = ego.velocity
  │  acceleration_ref = ego.acceleration
  │  yaw_ref = ego.yaw
/uavX/prometheus/swarm_command
  ↓
swarm_controller (本模块)
  ↓ 根据 controller_flag:
  │  0 → pos_controller() → send_attitude_setpoint()
  │  1 → send_pos_vel_xyz_setpoint(pos, vel, yaw)
/uavX/mavros/setpoint_raw/local 或 /uavX/mavros/setpoint_raw/attitude
  ↓
MAVROS → PX4 FlightTaskOffboard → 混控器 → 电机
```

### 关键设计决策

1. **traj_server 发布** `PositionCommand` 到 `/position_cmd`，launch 文件中通过 remap 映射为 `/uavX/planning/ego/traj_cmd`，ego_traj_to_cmd 通过此话题接收轨迹。

2. **零速检测**: ego_traj_to_cmd 检查 `velocity.x == 0` 来判断是否到达目标——到达时不发控制指令，避免飞控收到零速指令后位置漂移。

3. **增益切换**: swarm_controller 在 TRAJECTORY 模式下自动切换到高增益参数组 (`Kp_track`, `Kv_track`)，悬停/起降使用较低增益 (`Kp_hover`, `Kv_hover`) 以保证稳定性。

4. **前馈架构**: TRAJECTORY 模式将 EGO 规划的**完整轨迹（位置+速度）**发送给 PX4，速度作为前馈项。PX4 内部以 `PositionControl.cpp` 的 "position + velocity feedforward" 模式执行，比纯位置跟踪响应更快。

5. **swarm_controller 不负责避障**：避障由 EGO planner 的 bspline_opt 在轨迹优化阶段完成。swarm_controller 仅做编队级的 APF 避碰（`Velocity_Control` 模式），不涉及单机的环境避障。

## 启动

### 仿真（EGO + 单机）

`ego_swarm_control.launch` 启动单机的完整控制栈：

```bash
source devel/setup.bash
roslaunch prometheus_swarm_control ego_swarm_control.launch uav_id:=1 swarm_num:=1
```

此 launch 文件同时启动 `swarm_estimator`、`swarm_controller`、`ego_traj_to_cmd` 三个核心节点。

### 真机蜂群

`ego_station.launch` + 各机独立运行的控制脚本：

```bash
# 地面站终端
roslaunch prometheus_swarm_control ego_station.launch swarm_num:=2

# 各机分别启动 (见项目根目录脚本)
./ego_fastlio2_swarm_uav1.sh
./ego_fastlio2_swarm_uav2.sh
```

## 参数配置

`launch/ego_control_lidar_config.yaml` 包含完整的 PID 增益、地理围栏、起飞高度等参数。

### 关键参数速查

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `controller_flag` | 1 | 0=自研姿态环, 1=PX4位置环(推荐) |
| `controller_hz` | 50 | 控制频率 [Hz] |
| `Takeoff_height` | 1.2 | 起飞高度 [m] |
| `Disarm_height` | 0.2 | 自动上锁高度 [m] |
| `Land_speed` | 0.2 | 降落速度 [m/s] |
| `quad_mass` | 1.5 | 四旋翼质量 [kg] |
| `hov_percent` | 0.47 | 悬停油门比例 (0~1) |
| `hover_gain/Kp_xy` | 2.0 | 悬停位置P增益 |
| `hover_gain/Kv_xy` | 2.0 | 悬停速度P增益 |
| `track_gain/Kp_xy` | 3.0 | 轨迹跟踪位置P增益 (更高) |
| `track_gain/Kv_xy` | 3.0 | 轨迹跟踪速度P增益 (更高) |
| `geo_fence/x_min` | -500 | 地理围栏 x 下限 [m] |
| `drift_correction/enable` | false | 是否启用树干定位漂移补偿 |
| `drift_correction/ema_alpha` | 0.5 | 漂移 EMA 滤波系数 |
| `k_p` / `k_aij` / `k_gamma` | 1.2 / 0.2 / 1.2 | 编队一致性控制参数 |

## 目录结构

```
Modules/swarm_control/
├── CMakeLists.txt
├── package.xml
├── README.md
├── include/
│   ├── formation_utils.h          # 编队阵型偏移量定义
│   ├── math_utils.h               # 数学工具函数
│   ├── swarm_controller.h         # swarm_controller 类声明 + 全部控制函数
│   └── swarm_estimator.h          # estimator 变量声明
├── launch/
│   ├── ego_station.launch         # 地面站 + 终端控制 + rviz
│   ├── ego_swarm_control.launch   # 单机全栈: estimator + controller + ego_traj_to_cmd
│   └── ego_control_lidar_config.yaml  # 控制参数配置
├── meshes/                        # 3D 模型资源
└── src/
    ├── ego_traj_to_cmd.cpp        # [★] EGO→SwarmCommand 桥接
    ├── swarm_controller.cpp       # [★] 核心飞行控制器
    ├── swarm_estimator.cpp        # [★] 状态估计融合
    ├── swarm_ground_station.cpp   # 地面站监控
    ├── swarm_terminal_control.cpp # 终端手动控制
    └── swarm_formation_control.cpp# 编队终端控制
```
