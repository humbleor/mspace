# 原模块接入约定

PX4 森林入口运行项目 Modules 的节点和 launch 模板。当前模块新增可选 fsm/strict_waypoint_tracking，仿真显式启用；不是声称原连续任务行为未改变。Simulation 提供 PCD 雷达/物理 IMU、Gazebo 动力学、任务操作、可视化和评估。它不替代定位、轨迹优化、轨迹转换、状态估计或控制器。

链路：

Mid-360 模拟点云 + IMU → fastlio_mapping → /uav1/drone_cloud_registered → EGO 栅格地图。
Fast-LIO2 高频位姿 → swarm_estimator → /uav1/mavros/vision_pose/pose → PX4 EKF → /uav1/mavros/local_position/odom → EGO 里程计与地图位姿。
EGO B 样条 → traj_server → ego_traj_to_cmd → SwarmCommand → swarm_controller → /uav1/mavros/setpoint_raw/local → PX4 → Gazebo。

## 复用的模板

- Modules/fast_lio2/launch/mapping_mid360.launch 和 config/mid360.yaml：原滤波、外参和噪声参数；原 /uav1/drone_* 输出。
- Modules/ego_planner_swarm/plan_manage/launch_new/real_ego_run.launch：include 原 advanced_param_px4.xml，原地图分辨率 0.10、地图尺寸、速度 0.6、加速度 0.5、规划范围 3.5、traj_server 前视时间 1.0。
- Modules/swarm_control/launch/ego_swarm_control.launch 与 ego_control_lidar_config.yaml：原 swarm_estimator、ego_traj_to_cmd、swarm_controller；input_source=4，controller_flag=1，控制器起飞高度 1.2。
- Experiment/mavros/launch/px4_config.yaml 和 px4_pluginlists.yaml：保持项目 MAVROS 插件及配置，使用私有连接和 /uav1/mavros 命名空间。

不再用仿真专用 ego_local.yaml 配置 PX4 森林导航，也不再用 Python 发送原始飞控 setpoint 或发布 vision_pose。轻量 closed-loop 工具仍保留它的历史参数，不能代替本入口的原模块集成测试。hover 阶段是独立 PX4/动力学冒烟验证，仍直接发送参考点，不代表原导航模块测试。

## 接入覆盖项

以下是显式的仿真边界设置，记录到模块接入文件和实际 ROS 参数快照：

1. 原雷达/IMU 话题 remap 到 /sim/mid360/lidar、/sim/mid360/imu；输出目录和 RViz 启动位置属于记录/显示设置。
2. MAVROS 的 UDP 端口、target_system_id、连接超时按隔离 SITL 配置；森林 PX4 删除 GPS/真值导航输出，使用外部位姿融合。
3. 起飞后通过 /traj_start_trigger 启动任务：覆盖 fsm/realworld_experiment=true，使用原模块的触发门控。模板原值 false 会在完成起飞之前开始轨迹，与地面站起飞命令争抢。
4. 网格任务只覆盖 box_min/max、step、waypointDistriFlag、grid_direction 和 flight_type=3；generateWps 由原 C++ 节点执行。Python 同序生成仅用于配置验证、显示和结果对照，不向 EGO 逐点发送。
5. fsm/strict_waypoint_tracking=true：任务调度仍在原 C++ EGO 节点内，逐点规划并用实际 PX4 里程计确认到达。到达门槛为 min(原 no_replan_thresh, 0.3 m)，不放宽仿真 0.5 m 验收半径；终点仍需停留及物理真值独立验收。目标无可用执行点时停止任务，不跳过。当前点新出现障碍时仍用原膨胀栅格与 BFS 重定位，生成新的当前段并记录接受目标。模块默认该参数 false，保留旧连续模式；未改 Fast-LIO2、B 样条优化器、轨迹前视、控制器或地图安全距离。
6. scene 参考轨迹测试使用原 PRESET_TARGET（flight_type=2），把参考点加载到 fsm/waypoint*；--goals 只用于该模式。不能给原生网格任务使用 --goals 来偷偷截断任务。

森林场景的 --takeoff-height 仍用于 scene 参考高度与地图坐标转换；实际起飞动作服从原控制器的 1.2 米设置。网格 z 使用局部 world 的高度，不叠加参考高度。上述两种高度用途在 module_alignment.json 与 manifest.json 分别记录。

模拟地面站先发送 Takeoff，使原控制器持续发布有效起飞参考，再通过 MAVROS 服务请求 OFFBOARD 和解锁，起飞后发送一次任务触发。起飞和后续飞控 setpoint 全部由原模块生成。原 Idle(yaw_ref=999) 使用旧 0x4000 掩码；在当前 PX4 1.13 组合中该包启用零位置/速度/加速度，出现尚未爬升但 landed 已清除、起飞状态停在 ready 的情况，因此仿真地面站不使用这一旧解锁流程。没有改写原控制器或飞控起飞保护。

## 验收证据

运行目录保存 module_alignment.json、module_params.json、graph.json。图审计要求 setpoint 发布者为 swarm_controller_uav_1、vision_pose 为 swarm_estimator_uav_1、轨迹指令为 uav1_traj_server。物理真值不得被 LIO、EGO、状态估计器、控制器或 MAVROS 订阅。

结果保留 goals（原请求）和 accepted_goals（观察到的原 EGO 重定位标记）。Simulation 不选择邻近点；若模块自身重定位，报告应给出偏移。module_alignment.json 明确记录 algorithms_modified=true 和本次原模块任务调度变更，禁止把修复后运行称作完全未修改的基线。原始目标到达与模块接受目标的到达不能混称。

完整 32×16 网格：

    python3 Simulation/run.py px4 run --stage forest \
      --scene artifacts/forest_lio/route_125_155.json \
      --waypoints Simulation/px4_forest/config/grid_route.json \
      --output artifacts/px4_forest/my_native_grid \
      --duration 600 --speed .5 --keep-open

原模块短网格验证使用 config/grid_alignment_smoke.json（6 个目标），参考轨迹短程使用 --goals 1。


当前验证：可选严格模式六目标网格已通过，终点物理误差约 8.9 cm，话题发布者与真值隔离审计通过；原请求六点在 LIO 和物理轨迹中均按顺序进入 0.5 m 半径。15 目标完整网格也已通过，终点物理误差约 10.1 cm，原请求 15 点在 LIO 与物理轨迹中均按顺序进入 0.5 m 半径。两组为 0.5 倍速 Gazebo/RViz 双窗口实验。历史连续模式漏过中间点的失败证据仍保留，详细记录见 [验证报告](validation.md)。
