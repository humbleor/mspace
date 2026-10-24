# 双机公共坐标首版

本模式每机启动一个 swarm_frame_adapter。UAV1 的初始参考点固定为 world 原点，初始机头为 +X，左侧 +Y，上方 +Z；UAV2 默认在 (0,5,0)，两机初始航向均为 0。world 不跟随 UAV1 移动，也不依赖 GNSS。

## 全流程坐标系（当前代码实际行为）

### 坐标系定义

记号 | 名称 | 内容 | 谁发布
---|---|---|---
S | 雷达系 | 雷达原始点云 | Livox 驱动
I | 雷达 IMU 系 | Fast-LIO2 状态对应的机体系（child 叫 lio_body） | Fast-LIO2
L | 本机 LIO 局部系 | lio_init，原点/朝向由该机 LIO 启动决定 | Fast-LIO2
F | 飞控本体系 | fcu_local，MAVROS local_position 的 ENU 表达 | MAVROS
B | 机体 | base_link，MAVROS 本地 TF 的 child | MAVROS
W | 公共系 | world，原点=UAV1 初始参考点，X=初始机头，Y=左，Z=上 | swarm_frame_adapter

W 是右手系、Z 向上，语义与 MAVROS 的 ENU 一致（只是没有地理含义）。每架飞机都有自己的 L、F，两机之间只有 W 是共同的。

### 数据链路

```text
感知/定位（每机独立，坐标仍是本机的）
  Livox 点云 S ──雷达-IMU 外参──► I ──Fast-LIO2──► L(lio_init)
                                                │
      ┌── /uavX/drone_cloud_registered        │ 本机点云，frame=lio_init
      └── /uavX/drone_Odom_high_freq          │ 本机位姿，lio_init -> lio_body
                                                ▼
  swarm_estimator：vision = p_L + vision_offset，姿态直接用 LIO 姿态（无旋转补偿）
                                                │
                    /uavX/mavros/vision_pose/pose (frame=lio_init)
                                                ▼
  MAVROS：ENU→NED / FLU→FRD ──► PX4 EKF ──► PX4 本地估计
                                                │
                    MAVROS 转回 ROS：/uavX/mavros/local_position/odom
                                                │ frame=fcu_local, child=base_link, twist 在机体系
                                                ▼
坐标适配（唯一的双向边界）
  L ──T_WL──► W ：/uavX/world/cloud（frame=world）
  F ──T_WF──► W ：/uavX/world/odom、/uavX/prometheus/drone_state、/uavX/frame_ready
  W ──T_WF⁻¹─► F ：/uavX/mavros/setpoint_raw/local
                                                │
规划与控制（全部在 W 中计算）
  /uavX/world/odom + /uavX/world/cloud ──► EGO ──► /uavX/planning/bspline
        ──► traj_server ──► /uavX/planning/ego/traj_cmd（world 位置/速度/加速度/yaw）
        ──► ego_traj_to_cmd ──► /uavX/prometheus/swarm_command（world Move/TRAJECTORY）
        ──► swarm_controller ──► /uavX/world/setpoint_raw/local（frame_id=world）
                                                │
                                                ▼
  adapter 逆变换 ──► /uavX/mavros/setpoint_raw/local（frame_id=uavX/fcu_local）
        ──► MAVROS ENU→NED ──► PX4 位置环
```

### 每一段接口的坐标语义

数据 | frame / child | 坐标含义 | 发布者 → 消费者
---|---|---|---
`/uavX/drone_cloud_registered` | lio_init | 本机 L 系点云 | Fast-LIO2 → adapter
`/uavX/drone_Odom_high_freq` | lio_init → lio_body | 本机 L 系位姿（高频） | Fast-LIO2 → estimator / adapter
`/uavX/mavros/vision_pose/pose` | lio_init | 外部定位位置 + LIO 姿态 | estimator → MAVROS
`/uavX/mavros/local_position/odom` | fcu_local → base_link | 飞控本地系位置/姿态，线速度在机体系 | MAVROS → adapter（旧模式也给 EGO）
`/uavX/local/drone_state` | fcu_local | 连接、模式、armed 等状态 | estimator → adapter
`/uavX/world/cloud` | world | 公共系点云（已含 T_WL） | adapter → EGO 建图
`/uavX/world/odom` | world → base_link | pose 在 world，twist 保留在机体系（标准 Odometry） | adapter → EGO/显示
`/uavX/prometheus/drone_state` | world | 位置/线速度/姿态在 world，角速度在机体系 | adapter → 控制器/地面站
`/uavX/frame_ready` | — | 本机变换已锁定且输入新鲜 | adapter → 控制器（含邻机）
`/uavX/planning/bspline`、`/uavX/planning/ego/traj_cmd` | world | 轨迹与轨迹点/速度/加速度/yaw | EGO → traj_server / ego_traj_to_cmd
`/uavX/prometheus/swarm_command` | world | Move(TRAJECTORY) 的期望位姿 | ego_traj_to_cmd / 地面站 → 控制器
`/uavX/world/setpoint_raw/local` | world | frame_id=world，coordinate_frame=FRAME_LOCAL_NED（ENU 语义） | 控制器 → adapter
`/uavX/mavros/setpoint_raw/local` | fcu_local | 逆变换后的飞控本地系位置/速度/加速度/yaw | adapter → MAVROS
`/uavX/planning/swarm_trajs` | world | 机间共享轨迹（经 TCP 桥接） | EGO ↔ EGO

### 变换公式

初始化时用静止采样得到两套固定变换（`Δψ` 是该系相对公共 X 轴的航向差）：

```text
p_W = R_z(Δψ_K)·p_K + t_K        K = L 或 F
q_W = R_z(Δψ_L)·q_L              （点云只用位置；姿态用于 world/odom、DroneState）
v_W = R_z(Δψ_F)·v_F
t_K = p_init − R_z(Δψ_K)·p_K(t0)
Δψ_K = wrap(ψ_init − ψ_K(t0))
```

控制指令反向：

```text
p_F = R_z(Δψ_F)ᵀ·(p_W − t_F)
v_F = R_z(Δψ_F)ᵀ·v_W
a_F = R_z(Δψ_F)ᵀ·a_W
ψ_F = wrap(ψ_W − Δψ_F)
```

上式的 `R_z`、`t`、`ψ` 都由 `swarm_frame_transform.h` 实现并被`frame_transform_test.cpp`覆盖；imu→机体的安装外参 T_I_B 另由 `bodyPoint`/`bodyOrientation`组合，不包含在上述公式里。

### 边界与约定

- **只有两个变换边界**：adapter 内（W↔L 点云、W↔F 状态与指令）和控制器出口标签（frame_id=world）。规划器、控制器、桥接、地面站都不做额外坐标变换。
- **不转换的数据**：`drone_cloud_registered_body`、`drone_Laser_map`、`drone_Odometry`、`drone_path` 仍是本机 L 系；world 链路不使用它们。`drone_detect_lidar`（树干相对定位）在本模式下不启动。
- **速度语义**：MAVROS 1.20 的 `local_position/odom` 线速度在机体系，adapter 用姿态把它旋到 world 后才发布；`world/odom` 自身保持标准 Odometry（twist 在 child），EGO 用 `fsm/odom_twist_in_body=true` 再转到 world。
- **yaw 语义**：EGO 与控制器里的 yaw 都是相对 world +X；adapter 逆变换时减去 `Δψ_F`。姿态四元数则用 `R_z(Δψ)·q` 左乘。
- **高度基准**：起飞/悬停基于 world 当前位置，降落写死 z=0 与 Disarm_height，因此要求共同起飞地面高度；不同地面高度需另行改造。
- **TF 森林**：adapter 发布 `world→uavX/lio_init`、`world→uavX/fcu_local`；Fast-LIO2 发布 `lio_init→lio_body`；MAVROS 发布 `fcu_local→base_link`。每条边只有一个发布者。
- **参考点要一致**：estimator 的 `vision_offset_*` 把 LIO 位置平移后送给 PX4，adapter 的 `body_in_imu` 把 LIO 锚点换算到同一参考点。两者必须描述同一个参考点/安装关系；否则 T_WF 只能把飞控位置锚定到 `init_*`，点云与 odom 会差一个常量（`vision_pose` 的 frame 名只作元数据，MAVROS 不会按 frame_id 做变换）。
- **失效处理**：输入超时/倒退/跳变后 adapter 锁定 INVALID 并停止转发运动指令，不再自动重标定；飞控按自身 OFFBOARD 失联策略处置。

## 使用前提

- 两机水平、静止、同向且共用起飞高度基准；首版 init_z 必须为 0。
- 当前仅支持 controller_flag=1（PX4 位置环）。本机 Fast-LIO2 到 PX4 的外部位置/航向融合必须先单独验证正常，包括当前固件的融合参数、外部定位安装偏移与时间同步。本功能不改写飞控参数。
- launch/frame_adapter.yaml 的 body_in_imu 定义 T_I_B：机体参考点在雷达 IMU 系中的位置，以及把机体向量旋转到该 IMU 系的 RPY（弧度）。默认零值是安装轴对齐、参考点重合的假设，须根据实际安装测量确认；Fast-LIO2 的雷达-IMU 外参不能代替这个外参。该项会影响 LIO→world 的锚点（测试 test_body_imu_offset_shifts_lio_anchor 覆盖平移部分，旋转符号仍须以实机点云重合为准）。
- 雷达网络配置仍由每台机外部 Livox 工作区的 MID360_config.json 提供。LIDAR_ID 只选择 ROS 话题后缀，不会修改硬件 IP。两台机的时钟需要同步；轨迹消息还依赖一致的时间基准。
- 本模式不做在线配准或漂移修正。树干修正入口暂不启动，控制器 drift_correction/enable=false。

## 启动顺序

先按 AGENTS.md 加载 ROS、Livox 和本工作区环境并构建。

1. 在 UAV1 运行更新后的 ego_fastlio2_swarm_mavros.sh；它为本机和远端 MAVROS 开启 enable_world_frame。不要在日志或文档中复制脚本凭据。
2. 两机按地面标记摆好，保持静止。分别运行 ego_fastlio2_swarm_uav1.sh 与 ego_fastlio2_swarm_uav2.sh。UAV2 默认 export UAV2_INIT_Y=5.0，可按实测值覆盖。各机雷达后缀可通过 LIDAR_ID 覆盖。
3. 两机的 /uavX/frame_ready 均连续为 true 后，再使用已有地面站起飞/任务操作。控制器会检查两机就绪心跳；初始化前收到的任务不会排队在就绪后自动执行。
4. RViz Fixed Frame 设为 world；查看 /uavX/world/cloud 和 /uavX/world/odom。需要另开 RViz 时使用独立节点名。

底层入口默认 enable_world_frame=false，保留旧单机/仿真接线；三个多机脚本显式开启新模式。仅手动启动部分组件时，Fast-LIO2、MAVROS、control、EGO 四个入口必须一致开启该开关。旧模式下 fast_lio 的 group ns 取 /，参数与节点仍位于根命名空间，与改动前一致。

### 单机实机（Livox）

`ego_fastlio2_one_livox.sh` 默认已是 world 模式，不再需要手动拼参数：

```bash
./ego_fastlio2_one_livox.sh                 # world 模式（默认）
WORLD_FRAME=0 ./ego_fastlio2_one_livox.sh   # 旧链路，仅供对比/排障
```

- MAVROS：`px4_mavros.launch` 新增 `enable_world_frame`（默认 false），开启后 `local_position` 与 TF 的 frame 改为 `uav1/fcu_local`、child `uav1/base_link`，`imu/frame_id` 为 `uav1/base_link`。
- 驱动：改用 `fast_lio swarm_mid360_driver.launch uav_id:=1`，雷达/IMU 话题为 `/uav1/livox/lidar_<LIDAR_ID>`、`/uav1/livox/imu_<LIDAR_ID>`（`LIDAR_ID` 默认 `192_168_2_174`，只改话题后缀，不改 MID360_config.json）。
- 控制与规划：`swarm_num:=1`、`uav_init_*=0`、`enable_world_frame:=true`；任务指令在 `/uav1/frame_ready` 为 true 前会被控制器丢弃（不排队），因此起飞前保持静止。
- 单机同样需要静止约 3 秒完成变换锁定；这段等待由适配节点自己完成，脚本的终端顺序无需改变。

单机 world 路径已在 PX4/Gazebo SITL 中验证（`swarm_num=1`、UAV1=`(0,0,0,0)`）：READY 5.6 s、解锁 12.1 s、目标 2/2 到达、固定变换偏差 2e-16、初始锚点 3.7 mm、指令往返误差 6e-17。验证方法和证据字段见上文 SITL 一节。

## 接口与唯一发布者

各话题的坐标语义见上面表格，这里只列唯一发布者约束：

- Fast-LIO2：`lio_init→lio_body` TF、`drone_cloud_registered`、`drone_Odom_high_freq`；不再发布 world→base_link。
- MAVROS：`fcu_local→base_link` TF（local_position/tf），并吞下 `setpoint_raw/local` 做 ENU→NED。
- estimator：只发 `mavros/vision_pose/pose` 与 `/uavX/local/drone_state`；适配模式下旧 odom/path 可视化关闭。
- adapter：`/uavX/prometheus/drone_state`、`/uavX/world/odom`、`/uavX/world/cloud`、`/uavX/frame_ready`、`/uavX/world/*→fcu_local` 的 setpoint，以及 `world→lio_init`/`world→fcu_local` 两条固定 TF。
- controller：只写 `/uavX/world/setpoint_raw/local`；ego_traj_to_cmd、地面站、桥接均不碰飞控话题。

## 初始化和失效

从真实、有有效时间戳且时间接近的 LIO/FCU 输入计算固定 yaw+XYZ 变换。默认静止窗口 3 秒、线速度不超过 0.05 m/s、角速度不超过 0.1 rad/s、初始倾斜不超过 5 度，位置/航向窗口容差分别 3 cm/1 度。采样位置和圆周平均航向用于扣除非零局部初值。机体-IMU 外参仅用于正确建立 LIO->world，不能替代本机外部定位安装标定。

READY 后，断连、超时、倒退时间戳、无效姿态或明显局部位姿跳变会锁定 INVALID；恢复输入不会自动重新标定。停止任务指令转发后，由飞控已有的 OFFBOARD 失联处置接管，需在实机前核对该策略。落地并保持未解锁，再重启适配节点/对应 control launch 重新初始化。基于 Odometry 不能保证检测所有很小的飞控参考系重置；首版仅检测明显跳变。

启动前不发布虚构 world 位置。控制器检查全部配置 UAV 的就绪状态及 0.5 秒接收时效；紧急 Disarm 不受坐标门控阻止。当前只支持 FRAME_LOCAL_NED 的 ROS ENU 输入；部分 XY 分量启用的指令拒绝，因为 yaw 旋转会混合 XY。现有 idle、位置、速度、XY速度+Z位置和位置速度组合均有明确处理。

## 验证

```bash
bash Modules/swarm_control/test/run_frame_tests.sh
source /opt/ros/noetic/setup.bash
source ~/workspace/ws_livox/devel/setup.bash
source devel/setup.bash
ROS_PARALLEL_JOBS="-j2 -l2" ./compile.sh
```

ROS 测试创建独立本机 Master 和模拟反馈/服务，不启动飞控或雷达驱动；退出时仅清理测试创建的进程组。launch_config_test.py 只做离线加载审计：同时加载两机的 LIO、MAVROS、control、EGO 入口，检查命名空间、frame 名、适配节点参数、唯一节点名，以及旧模式下的默认值（world/body/base_link、estimator 偏移与可视化、无适配节点）。构建与测试日志位于 artifacts/swarm_world_frame。静止显示 (0,5,0) 只验证初始化约束，不是独立精度证明；实测仍需检查同一树干点云重合、已知距离移动、原地旋转与实际轨迹执行。软件测试通过不能代表双机飞行或长期相对定位精度已验证。

### 单机 SITL（真 Fast-LIO2 + MAVROS + PX4）

`Simulation/px4_forest` 的 PX4/Gazebo 闭环跑的就是本模式的真实链路。加 `--world-frame` 后，仿真侧把三个入口绑定到 world 模式，MAVROS 的本地 frame 同步改名为 `uav1/fcu_local`，并在起飞前等待 `/uav1/frame_ready`：

```bash
source /opt/ros/noetic/setup.bash && source ~/workspace/ws_livox/devel/setup.bash && source devel/setup.bash
python3 Simulation/run.py px4 run --stage forest --scene artifacts/forest_lio/route_125_155.json \
  --output artifacts/px4_forest/world_single --duration 90 --speed .5 --headless --goals 1 --world-frame
```

运行目录新增 `world_frame_evidence.json`，即使适配节点自己打日志也不能伪造的四项证据：固定变换刚性（用 world/odom 与 fcu odom 的同一批次反算 Δψ 与 t，偏差应为浮点精度级）、初始锚点（READY 后第一帧世界位姿在 `init_*` 附近）、指令往返（控制器 world setpoint 与适配节点 fcu setpoint 按 header.stamp 配对后必须满足逆变换；`0x4000` 预解锁心跳按设计原样放行，单独计数）、点云对齐（world 点云与未变换 LIO 点云对同一物理地图的对齐度不得变差）。任一项失败会把整次运行判为 failed。

首次单机运行结果（UAV1=`(0,0,0,0)`，speed 0.5，`--no-tree-collision`，日志 `artifacts/swarm_world_frame/sitl_world_smoke3.log`）：READY 于 5.15 s，解锁于 12.07 s，两个目标点均到达，终点误差 4.8 cm，固定变换偏差 2e-16、初始锚点 5.3 mm/0.0075°、指令往返误差 2e-16、world 与 LIO 点云中位差 1.6 mm。这只能说明**单机链路与坐标系语义正确**，不能替代双机对齐与实机精度。

适配节点运行时的两个相关约束：`receive_timeout`（默认 1.5 s）是"发布者彻底静默"的看门狗，与 `source_timeout`（默认 0.5 s 时间戳陈旧）分开；回调由 `AsyncSpinner(3)` 并发处理，云变换在锁外执行，避免一次慢回调把就绪状态误判为失效。SITL 在 0.5 s 单窗口下曾因此误锁定 INVALID，日志会打印每个源的 age/recv/skew 以便定位。
