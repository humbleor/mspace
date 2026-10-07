# PX4 森林闭环仿真

实跑结果、性能剖析和验证边界见 [验证报告](docs/validation.md)。

入口在 `Simulation/px4_forest/`，传感器渲染与森林场景数据继续复用 `Simulation/forest_lio/`。

链路：模拟 Mid-360/IMU → 原 Fast-LIO2 → 原 swarm_estimator → MAVROS 外部位姿 → PX4 EKF；PX4 里程计和 LIO 点云进入原 EGO，轨迹经原 traj_server、ego_traj_to_cmd 和 swarm_controller 发送给 MAVROS/PX4，Gazebo 更新真实模拟运动。复用 Modules 的实机 launch 模板及参数，接入细节见 [原模块接入约定](docs/module_alignment.md)。

当前任务显式启用原 EGO 节点新增的 `fsm/strict_waypoint_tracking=true`，使用实际里程计确认逐点到达，再规划下一点；参数默认 false，保留原连续任务模式。该修复改变原模块的任务调度，运行文件明确记录，未修改 B 样条避障优化器、控制器、轨迹前视或验收半径。更新后须重建规划模块。

这是 **PX4 SITL 软件飞控在环**。实体飞控硬件在环需要另接飞控并适配 HIL 通信，当前没有完成该部分。

## 一键运行

从任意目录执行项目内的 `Simulation/start.sh`，脚本自动切换到项目根目录并加载环境。默认完整网格：`bash Simulation/start.sh`；六点：`bash Simulation/start.sh small`；短程：`bash Simulation/start.sh short`。选项与离线准备流程见 [统一说明](../README.md)。

## 环境与构建

Ubuntu 20.04、ROS Noetic、Gazebo Classic 11。使用本机 PX4 v1.13.2 的兼容组合，不代表最新 PX4 推荐环境。构建脚本克隆已提交代码到项目 artifacts，原 PX4 工作区的未提交修改不会被复制或改变。

在项目根目录：

```bash
source /opt/ros/noetic/setup.bash
source ~/workspace/ws_livox/devel/setup.bash
source devel/setup.bash
python3 Simulation/run.py px4 build --source ~/workspace/PX4_Firmware
python3 Simulation/run.py px4 test
```

依赖包括 Gazebo 开发库、PX4 SITL 工具链、jinja2/empy/genmsg、ROS MAVROS、numpy/scipy/matplotlib/yaml。子模块缺少缓存时需网络下载。脚本默认并行 2 个编译任务，原生传感器插件以 1 个任务构建。

## 运行

输出目录必须为空。进程、ROS Master、Gazebo Master 与 MAVLink 端口独立；结束只清理本次启动的进程。PX4 instance 默认 20；同时运行多个测试需显式使用不同 instance。

先验证原生 PX4 悬停（该阶段保留 GPS，只验证飞控/动力学链路）：

```bash
python3 Simulation/run.py px4 run --stage hover \
  --output artifacts/px4_forest/my_hover --duration 40 --speed .5
```

森林阶段删除 GPS 插件，禁用 Gazebo 真值位姿 MAVLink 输出，设置 EKF2_AID_MASK=24（外部位置和航向）及 EKF2_HGT_MODE=3（外部高度）。导航输入来自 Fast-LIO2：

```bash
python3 Simulation/run.py px4 run --stage forest \
  --scene artifacts/forest_lio/route_125_155.json \
  --output artifacts/px4_forest/my_forest --speed .25 --duration 120 --keep-open
python3 Simulation/run.py px4 report artifacts/px4_forest/my_forest
```

### 按 generateWps 生成完整网格路线

使用 config/grid_route.json 配置 x/y/z 各自的 min、max、step。当前配置对应 x=0..32、y=0..16、z=2，step_X=8、step_Y=8、step_Z=2，waypointDistriFlag=0、grid_direction=0。生成 15 个目标，另加独立起飞点：

```bash
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=4
python3 Simulation/run.py px4 run --stage forest \
  --scene artifacts/forest_lio/route_125_155.json \
  --waypoints Simulation/px4_forest/config/grid_route.json \
  --output artifacts/px4_forest/my_grid_route \
  --duration 600 --speed .5 --keep-open
```

不传 --goals 就执行全部生成目标。duration 是仿真时间上限，不是墙钟秒数；目标距离较长时需为原模板 0.6 m/s 的规划速度与避障留出时间。倍速是时间推进速度，与无人机的规划飞行速度不同。0.1 倍速用于低负载复验，并非必须；更高倍速若传感器积压超过限度会如实失败。

这些坐标使用 RViz 的局部 world，z 是相对初始机体位置的高度，传入 2 就是 2 米，不再叠加 takeoff-height。scene 仍提供森林地图、覆盖证据及坐标变换；--waypoints 覆盖 scene 的参考轨迹目标。

网格任务使用原 EGO flight_type=3，原 C++ generateWps 负责生成和调度全部目标。waypoints.py 只用于配置校验、显示和结果对照，测试会编译原函数逐点对照。网格不接受 --goals；scene 路线使用原 PRESET_TARGET（flight_type=2），可用 --goals 做短程验证。

输出保存 waypoint_config.json、route_goals.json、module_alignment.json、module_params.json 和 graph.json。goals 保留原请求目标，accepted_goals 记录原 EGO 自身重定位标记；运行器不选择替代点。两者偏移会写入报告。当前 (24,0,2) 邻近地图障碍，整条路线能否通过以实跑结果为准。旧入口的通过记录不能作为本次接入的证据；当前严格模式六目标与 15 点网格双窗口实跑均通过，证据见 [验证报告](docs/validation.md)。

### 图形窗口

默认 `--gui both` 自动启动 Gazebo 和 RViz，使用本次 ROS / Gazebo Master 的独立端口。图形模式需要 DISPLAY 或 WAYLAND_DISPLAY；无显示环境使用 `--headless`（等同 `--gui none`）。也可单独选择 `--gui gazebo` 或 `--gui rviz`。

- Gazebo：官方 Iris 机体、电机响应和地面物理运动。当前 PCD 没有转成 Gazebo 树木碰撞几何。
- RViz：森林静态地图、LIO 实时点云、EGO 轨迹和目标；蓝色物理轨迹、橙色 LIO 轨迹、绿色 PX4 估计轨迹，以及机体姿态和飞控模式。默认配置为 `config/forest.rviz`，Fixed Frame 为 `world`。
- `--keep-open`：任务成功后暂停物理并保留窗口，关闭窗口或 Ctrl+C 后保存结果并清理本次进程；失败仍立即退出。查看保留窗口期间，仿真时钟停止。

可视化由独立只读观察进程发布。静态地图展示最多 20 万点，雷达仍对完整森林几何做逐点射线查询；显示抽稀不改变传感器输入。真值轨迹只供显示和评估。关闭单个窗口不停止飞行；窗口异常退出会记录到 `result.json.visualization.failed_windows`。

`--goals 1` 只测试起飞及第一个森林目标。`--speed` 是仿真时间与墙钟时间比例，降速不会重复点云或使用陈旧遮挡结果。每条射线按该点时间的实际位姿查询静态世界 BVH；超过传感器积压限度直接判失败。机体 IMU 输出为物理比力、单位 g，与现有 Mid-360 Fast-LIO2 配置一致。

慢速仿真会拉长墙钟等待时间：0.1 倍速时，初始化等待 8 秒仿真时间约需 80 秒墙钟时间，再经过切换模式与解锁。运行器在输出目录生成独立 MAVROS 配置，心跳墙钟超时为 max(10, 3 / speed) 秒；实际值保存在 mavros_config.yaml。每次生成的 PX4 rcS 使用明确的 bc 小数精度，避免 0.1 × 0.5 被截为零，保持原有超时随倍速缩放的含义；COM_OF_LOSS_T 在验收时实际读取，零值会判失败。没有改动外部 PX4 工作区或关闭 OFFBOARD 保护。

仿真不连接遥控器，因此仅在本次运行的 PX4 参数副本中设置 COM_RC_IN_MODE=4、COM_RCL_EXCEPT=4，允许 OFFBOARD 不因遥控器缺席触发保护。OFFBOARD 指令超时保护与解锁前状态检查仍保留；这组设置不要直接复制到实机。

森林原始地图坐标依据 scene 的平移和旋转转换到导航局部坐标。起飞后进入原采集高度；物理真值仅用于生成传感器与离线评估。每次运行保留参数、状态变化、话题订阅审计、点云帧统计、物理/LIO/PX4 轨迹、IMU、PX4 ULog 和各进程日志。

## 验证边界

- PCD 影响激光雷达测量，目前没有树干/枝叶实体接触碰撞模型。
- Iris 动力学和 IMU 尚未用你的机体、推力曲线、真实振动标定。
- 地图未观测区域不能作为已知自由空间，覆盖证据仍只用于评估。
- 当前入口为单机；不宣称多机闭环或实体飞控 HIL 已验证。
- `result.json` 状态为 ok 才代表该次验收通过；有日志不代表成功。

官方参考：[Gazebo SITL](https://docs.px4.io/v1.13/en/simulation/gazebo)、[外部位姿融合](https://docs.px4.io/v1.13/en/ros/external_position_estimation)、[MAVROS OFFBOARD](https://docs.px4.io/v1.13/en/ros/mavros_offboard_python)。
