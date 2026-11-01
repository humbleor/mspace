# 共享森林中的多机 PX4 SITL

使用一个 Gazebo 世界、一个私有 ROS Master 和一个仿真时钟。每机独立运行 PX4、MAVROS、Mid-360/IMU、Fast-LIO2、公共坐标适配器、EGO 和控制模块。静态森林 PCD/BVH、树木碰撞几何与实验生命周期共享。

## 构建与启动

命令在项目根目录的 Ubuntu/WSL Bash 中执行。沿用既有 ROS/Livox/项目构建环境；修改物理插件后运行：

```bash
bash Simulation/start.sh build
```

先双机物理悬停，再执行森林网格任务：

```bash
bash Simulation/start.sh swarm \
  --fleet Simulation/px4_forest/config/fleet_two.json \
  --stage hover --duration 45 --speed .5 \
  --output "artifacts/px4_forest/two_hover_$(date +%Y%m%d_%H%M%S)"

bash Simulation/start.sh swarm \
  --fleet Simulation/px4_forest/config/fleet_two.json \
  --scene artifacts/forest_lio/route_125_155.json \
  --duration 600 --speed .25 \
  --output "artifacts/px4_forest/two_forest_$(date +%Y%m%d_%H%M%S)"
```

默认同时启动 RViz 和 Gazebo：RViz 显示共享森林、各机世界坐标里程计、点云及规划轨迹，Gazebo 显示物理世界。可用 `--gui gazebo` / `--gui rviz` 只开一个窗口，`--gui none` 无窗口运行；`--keep-open` 在任务稳定完成后暂停并保留窗口。多机入口自己生成 `report.md`，无需调用单机报告工具。

悬停模式保留 GPS，仅诊断双 PX4、MAVROS 与动力学，不启动 LIO/EGO，不代表 GNSS 拒止导航验收。森林模式移除 GPS，并禁止 Gazebo 向 PX4 注入真值外部位姿。

## 启动进度、缓存与 RViz 配置

终端会逐阶段打印 `[startup +Xs]`：模型/导航配置、PCD 读取、法向量/BVH、扫描模板、几何缓存/提取、ROS Master、PX4/Gazebo 后端、窗口、MAVROS 就绪、树木模型和导航启动。每阶段的墙钟耗时及失败状态写入本次输出目录的 `startup_timing.json`，结果报告也保留这些记录。`--speed` 只控制仿真倍率，不加速预处理。

树木几何默认缓存在 `artifacts/px4_forest/geometry_cache/`。缓存键包含变换后点云内容、提取参数、几何格式与提取算法源码哈希；不匹配或损坏会重新生成，写入使用原子替换。已有运行只有带匹配的 `geometry_cache.json` 来源证明（算法及几何内容哈希）时才自动复用；旧版没有来源证明的文件不自动导入。`--forest-geometry PATH` 仍可显式指定同地图几何，但不会把显式覆盖提升为默认缓存。当前运行的 `geometry_cache.json` 记录来源，`forest_geometry.json` 保留实际使用的几何。

RViz/Gazebo 窗口在后端进程启动后立即打开，早于 MAVROS 就绪检查、树木模型加载和导航启动，因此刚打开时可能暂时没有树木或飞机点云。地图/BVH/扫描模板等预处理仍在后端启动前完成。首次无可信缓存仍需提取几何，后续匹配运行跳过提取。

多机可视化配置位于 `Simulation/px4_forest/config/fleet.rviz`，启动时会按当前 fleet 机数更新并打印绝对路径，不是固定的双机配置。本次运行同时在 `<本次输出目录>/fleet.rviz` 保留相同内容的快照供 RViz 加载，避免另一场仿真更新公共配置影响当前运行。RViz 进程已连接本次私有 ROS Master，另开 RViz 时也需使用该目录 `manifest.json` 中的 ROS Master 端口。

航点显示使用每机 `/sim/view/uavN/goals`（MarkerArray，锁存完整原请求航点）。UAV1 为黄色、UAV2 为蓝色，颜色与各机点云/里程计保持一致；显示副本不修改规划器目标。里程计 `Keep=1`，仅保留最新箭头，规划轨迹仍独立显示。

### 坐标就绪撤销

`frame alignment lost` 表示适配器将 `/uavN/frame_ready` 置为 false；具体原因看本机 `navigation.log` 的 `Frame alignment INVALID`。运行 `two_forest_20261102_015154` 在仿真时间 43.004 秒记录：LIO age=0.184s、FCU age=0.004s，时间戳 skew=-0.18s 超出 `max_source_skew=0.15s`，触发锁存失效。两路 source age 尚未超过 0.5s；该记录证明时间戳不同步，尚不能单凭它归因为坐标跳变或某个线程故障。保护阈值未放宽。降低倍率到 `.25` 可用于检查负载影响，但不能保证解决定位或航线问题。

### 物理接触报错

`physical contact at ...: A <-> B` 表示 Gazebo 报告了实体接触，程序保留失败并退出；不是普通启动异常。查看 `uavN/contacts.json` 可区分树木与机间接触，`pair_separation.json` 单独记录机间间距。

`two_forest_20261102_013846` 的首次接触为仿真时间 39.472 秒，UAV1 `rotor_0` 与 `trunk_0001` 接触，机间间距检查通过。该次 UAV1 的名义 PCD 径向净空检查通过，但真实转子/拟合树干接触失败，两者模型不同；不能据此把该网格视为可安全飞行。此次启动改进没有修改规划器、碰撞模型或该失败判定，森林路线闭环验证仍未通过。

## 三机及更多飞机

三机使用同一个命令，更换配置：

```bash
bash Simulation/start.sh swarm \
  --fleet Simulation/px4_forest/config/fleet_three.json \
  --duration 150 --speed .25 \
  --output artifacts/px4_forest/my_three_forest
```

也可仅生成资源检查编号和接口：

```bash
bash Simulation/start.sh swarm \
  --fleet Simulation/px4_forest/config/fleet_three.json \
  --output artifacts/px4_forest/my_three_prepared --prepare-only
```

配置中的每条记录形如：

```json
{
  "uav_id": 2,
  "instance": 21,
  "initial": [1.5, 2.0, 0.0, 0.0],
  "goals": [[2.5, 2.0, 1.5]]
}
```

- `uav_id`：当前 EGO 前序轨迹协议要求从 1 连续编号；最多接受 50 条配置，与坐标适配器编号范围一致。这是输入限制，不是已验证的运行规模。
- `instance`：每机唯一，范围 0..253。固定 PX4 v1.13.2 使用 `MAV_SYS_ID=instance+1`，MAVROS target_system 随之配置。
- `initial`：公共 forest world 下的 x/y/z/yaw，单位米和弧度。当前适配器要求 z=0、水平静止初始化；起始相对位置和航向视为已知配置。
- `goals`：公共 world 中的原请求航点，各机可不同；非空且高度至少 0.5 米。森林任务实际起飞高度为 1.2 米。
- `grid`：与 `goals` 二选一，按本机范围和步长自动生成网格，使用 EGO 原生 `flight_type=3 / generateWps`。两者同时填写会报错。每机最多 300 个网格航点，与原生 EGO 数组容量一致。
- 出生间距至少 1.2 米，且应满足 `--minimum-separation`；更换场景时必须重新选择出生点和航线。示例位置只针对默认森林准备。
- 增加飞机只需追加记录，无需复制运行器或改写按 UAV 编号分支的代码。CPU、内存和实际传感器积压决定可运行规模。

### 每机独立网格

`fleet_two.json` 默认配置 UAV1 起点 `[0,0,0,0]`、UAV2 起点 `[0,5,0,0]`。两机分别使用自己的 `grid`，不是共同飞向同一份航点：

| 飞机 | X 范围/步长 | Y 范围/步长 | Z 范围/步长 | 航点数 |
| --- | --- | --- | --- | --- |
| UAV1 | 0～20 / 5 | 0～2 / 2 | 2～2 / 2 | 10 |
| UAV2 | 0～20 / 5 | 5～7 / 2 | 2～2 / 2 | 10 |

每机记录中使用以下配置替代 `goals`（此处为 UAV2）：

```json
"grid": {
  "waypointDistriFlag": 0,
  "grid_direction": 0,
  "x": {"min": 0, "max": 20, "step": 5},
  "y": {"min": 5, "max": 7, "step": 2},
  "z": {"min": 2, "max": 2, "step": 2}
}
```

范围均为共享 forest world 的绝对坐标，不再叠加 `initial`。两项方向参数可省略，默认均为 0；`waypointDistriFlag` 当前只支持 0（网格），`grid_direction=0` 为 X 折返、Y 推进，1 为 Y 折返、X 推进。每轴必须是有限数值的 `min/max/step`，要求 `min<=max`、`step>0`；生成高度至少 0.5 米。非整除边界的顺序沿用原生生成器，不强制补齐终点。

每机先在自己的起点起飞至 1.2 米，再触发自己的网格任务；生成航点记录在 `uavN/route_goals.json`（首项为起飞点），同一组网格航点用于原请求顺序到达验收。`--prepare-only` 可生成配置并检查每机航点及 launch，不会启动飞行进程。

`--duration` 是整个任务的仿真秒数上限；120 秒不保证完成较长网格，可从 600 秒开始。上述条带只是示例配置，尚未完成这组起点/航线的闭环验收；分开配置航点也不等于已证明机间避碰和森林通行安全。

## 坐标和传感器

森林 PCD 和碰撞几何使用同一个 forest world。Gazebo 物理坐标到 forest world 只有一个共享平移，由静止初始物理布置与配置的一致性检查确定。每机雷达从自己的物理位姿历史渲染，不能各自减去出生位置后去同一原点取样。

导航输入仅为每机模拟雷达/IMU及自身估计：
`/sim/uavN/mid360/lidar`、`/sim/uavN/mid360/imu`。
真值 `/sim/uavN/plant/odom` 仅用于传感器生成与独立评估。

每机 LIO 输出保留自己的 `uavN/lio_init`，MAVROS 使用 `uavN/fcu_local`；适配器将点云和状态转换到公共 `world`，并将 world 控制指令逆变换到 FCU。模型、TF、LIO 参数、日志和 PCD 输出均按飞机区分。

EGO 在同一个 ROS Master 内通过 `/broadcast_bspline` 与前序 `/uavN/planning/swarm_trajs` 通信，不需要另启跨主机 TCP 桥。任务触发 remap 到每机 `/uavN/traj_start_trigger`；所有飞机完成坐标初始化和起飞后才分别触发一次。

## 仿真速度与资源

PX4 v1.13 的 Gazebo MAVLink 插件在每次模型 Load 时缩放全局物理频率。多机若均读取 .5 倍速，第二次初始化可能因频率不是 250 Hz 整数倍而终止。

多机运行器给 gzserver 单独设置 `PX4_SIM_SPEED_FACTOR=1`，待模型全部加载后，由第一架物理插件在首次更新时统一设置世界物理频率。PX4 进程仍使用请求倍率进行自身超时缩放。保留锁步与 0.004 秒物理步长，不修改上游 PX4 源码。

渲染器共享静态 BVH，轮流为每机处理一帧，默认每机 10 Hz；扫描相位/帧计数/位姿缓存独立。先从 .25 倍速验证，按每机 `sensor_timing.csv` 中的积压和处理时间决定是否提高倍率；不能只看总平均值。

## 验收和退出

本次实验目录包含：

- `manifest.json`：飞机配置、编号、端口、PX4 提交和共享物理原点。
- `uavN/`：每机独立 PX4 启动目录、日志、传感器统计、LIO/FCU/world/物理轨迹及 `result.json`。
- `uavN/requested_waypoint_evidence.json`：world 估计与物理轨迹对原请求航点的顺序到达检查；模块重定位后的 accepted_goals 单独记录。
- `graph.json`：唯一发布者、每机传感器订阅、轨迹广播及真值隔离检查。
- `pair_separation.json`：每一对飞机的最小距离，检查物理采样之间的分段线性轨迹，避免漏掉采样间交叉。
- `result.json`、`report.md`：所有飞机和公共检查的整体结果。
- `cleanup.json`：严格按本次 ROS_HOME 归属清理进程。

每机须保持 OFFBOARD/解锁、完成任务并稳定停留；任意一机进程退出、周期源断流、坐标就绪丢失或树木/机间接触均使整个实验失败。树木接触观察器按机体归属过滤，包含机间物理接触。关闭树木碰撞只能用于诊断，不具备森林验收通过资格。

退出前的 Gazebo/参数服务调用设有超时，审计失败不能无限阻止进程清理。Ctrl+C 会记录中断并清理整组；从其他终端清理时沿用：

```bash
python3 Simulation/run.py px4 stop --output artifacts/px4_forest/my_two_forest --dry-run
python3 Simulation/run.py px4 stop --output artifacts/px4_forest/my_two_forest
```

要求 `cleanup.json` 为 `status=complete`、`remaining=[]`，不会按进程名清理其他实验。

## 验证范围

动态无人机几何尚未加入 PCD 射线求交：雷达只产生静态森林回波，机间避碰输入来自规划轨迹通信。若要验证雷达机间检测，后续需实现另一机几何、遮挡、扫描期间运动与本机排除。

已知初始布置不代表在线机间定位；短程、分离航线通过也不等于交叉航线、通信故障或大规模协同飞行全部通过。实际运行结果和限制应以对应目录的报告为准。

## 自动化检查

`python3 Simulation/run.py px4 test` 首先运行随代码提供的 `checks/`，若存在本地历史 `tests/`，再运行该目录。新检查覆盖三机模型/launch 隔离、编号校验、非零坐标锚点、扫描帧序列化、共享传感器原点、采样间机间距离和服务调用超时。


### 本地验证记录

- 双机物理悬停：通过，最小机间距 2.989 米；输出 `artifacts/px4_forest/fleet_hover_two_v2_20261025/`。
- 双机森林短程、0.5 倍速：通过。两机原请求航点均完成，物理终点误差 0.0626 / 0.0852 米；点云、固定坐标变换、控制指令逆变换、树木接触、EGO 净空、轨迹广播连接、真值隔离和清理检查通过。输出 `artifacts/px4_forest/fleet_forest_two_v3_20261025/`。
- 三机森林短程、0.25 倍速：通过。第三架初始 yaw=0.4 rad；三机物理终点误差 0.0679 / 0.0932 / 0.0730 米，三对最小距离分别 2.484 / 1.565 / 3.955 米。三机均有 301 帧模拟雷达数据，坐标、任务、物理碰撞、图审计和清理检查通过。输出 `artifacts/px4_forest/fleet_forest_three/`。
- 三机 0.5 倍速尝试触发扫描积压保护，按失败保存，进程清理 complete；该倍率不作为当前机器的三机通过配置。
- 自动化检查：33 项森林/LIO 历史测试、16 项随仓库提供的新检查和 80 项 PX4 历史测试，共 129 项通过。
- 短程分离航线验证不代表交叉航线动态避碰已经验收。

- 原单机 `short`、0.5 倍速回归：通过，物理终点误差 0.0794 米，清理 complete；输出 `artifacts/px4_forest/single_fleet_regression_20261025/`。
