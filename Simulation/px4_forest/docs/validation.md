# 当前 PX4 森林网格验收

## 仿真地图内存修复（2026-10-13）

完整网格仿真由任务范围和规划余量生成 96×64×10 m 地图，分辨率 0.1 m、膨胀 0.25 m 保持不变；原实机模板保留。74 项测试通过。双窗口完整网格配置运行 40 s，EGO RSS 约 1.0 GiB、VmSwap=0，并正常进入 OFFBOARD 和任务；因限时未走完整路线，任务结果仍为 failed，不能当作 15 点通过证据。详见 [内存修复记录](map_memory.md)。


## PCD 物理碰撞接入与四组接触验证（2026-10-12）

当前森林入口默认加载圆柱代理并检查真实 Iris 接触，接触即 failed；名义距离仍使用 EGO 参数。最终模型含 29 树干代理、76 矮干片段和 11 粗枝候选，共 116 个形状，未识别几何仍是覆盖限制。独立正面、擦边、旁路、6 m/s 撞击均通过；单机无窗口 0.25 倍速森林短程通过，传感器 P95 约 41.7 ms、最大积压 0.224 s、接触观察器最大间隔 4 ms。显式初始接触夹具验证 failed/非零退出及报告保存；不能当作自然森林的碰撞次数。

69 项测试、Gazebo 插件构建与 ./compile.sh 六模块均通过。数据、运行命令、坐标核验和未建模边界见 [森林物理碰撞报告](forest_physical_collision.md)。下文为此前阶段证据，不代表当前完整网格或 1 倍速已验证。

## EGO 参数验收、历史定位与单根树干物理测试（2026-10-12）

按用户约定默认改为 EGO 实际 obstacles_inflation 名义距离（当前 0.25 m），不叠加机体或 surfel 半径。该径向检查与原 cloudCallback 的 XY 三格/Z 一格栅格膨胀不完全等价，报告明确边界；原保守 proxy 模式保留为可选，并保留之前全部报告。

历史六目标在当前标准下 clear（最小净间距 +0.32424 m）；15 目标仍有一个 129.32–130.96 s 距离不足区间（−0.20673 m）。定位/参考轨迹和证据边界见 [历史原因定位](clearance_diagnosis.md)，不能将下文旧 proxy 失败与当前标准混称。

单根树干插件 CMake 构建通过；真实 Gazebo 正面接触/旁路两组均通过，接触力改变机体运动，并非软件判定后强制停机。此为零重力刚体接触测试，不是 PX4 导航飞行实验。详见 [单根树干场景](single_trunk.md)。本次 60 项仿真测试通过（森林 28、PX4/碰撞/树干 32）。

新默认 EGO 模式的无窗口 0.25 倍速短程闭环通过，退出码 0，报告保存实际读取的 EGO 参数，导航发布者和真值隔离审计通过。

- [新默认短程报告](../../../artifacts/px4_forest/ego_clearance_short_20261012/report.md)
- [新默认短程距离证据](../../../artifacts/px4_forest/ego_clearance_short_20261012/collision_evidence.json)

## 新增碰撞验收（2026-10-12）

森林入口新增扫掠球与 PCD 障碍代理的独立验收，细节见 [碰撞验收说明](collision_acceptance.md)。下文 2026-10-07 的 ok 是历史导航验收结果，尚未包含这一新条件。

使用 0.4 m 机体包络、0.2 m 障碍代理和最大 0.05 s 真值间隔，复核历史轨迹：六目标最小净间距约 −0.0258 m，首次相交约 37.588 s；完整 15 目标最小净间距约 −0.5567 m，首次相交约 36.789 s。两者在该保守代理模型下均未通过碰撞验收，不修改历史 result.json，也不把代理相交称为实际撞树。点云约 100.6 万点，轨迹未抽稀，分别检查 22,643 和 114,250 段。

- [六目标碰撞复核](../../../artifacts/px4_forest/collision_acceptance_20261012/six_goal_final_evidence.json)
- [15 目标碰撞复核](../../../artifacts/px4_forest/collision_acceptance_20261012/fifteen_goal_evidence.json)

新增 10 项几何/证据测试，与现有套件共 53 项通过。首次新短程半速实验触发原有 0.4 s 传感器积压保护，整体 failed，仍保存碰撞证据和失败报告；不能作为完整任务通过记录。

随后四分之一速无窗口短程闭环通过，入口退出码 0，result.status=ok、collision.status=clear。完整检查 9,271 段，最小净间距约 0.0362 m；采用同一 0.4 / 0.2 m 模型，未修改定位、规划、控制或避障参数。此短程结果不升级为六点/15 点任务通过。

- [新增短程通过报告](../../../artifacts/px4_forest/collision_short_quarter_20261012/report.md)
- [新增短程碰撞证据](../../../artifacts/px4_forest/collision_short_quarter_20261012/collision_evidence.json)

负向集成采用相同短程任务和 0.25 倍速，仅将本次验收机体半径放大到 1 m（不是默认机体尺寸）。目标已到达，仍因碰撞项 failed 返回退出码 1，错误为 `Forest collision acceptance failed: collision`，并正常保存报告；证明航点通过不能绕过碰撞门槛。

- [负向集成报告](../../../artifacts/px4_forest/collision_negative_quarter_20261012/report.md)
- [负向集成结果](../../../artifacts/px4_forest/collision_negative_quarter_20261012/result.json)

当前入口使用原 Modules 节点和显式严格航点模式；仅这里列出的运行作为当前网格验收证据。环境为 Ubuntu 20.04、ROS Noetic、Gazebo Classic 11、PX4 v1.13.2。

## 严格航点任务修复与两组网格验收（2026-10-07）

独立连续模式基线再次失败：648 帧雷达，整帧处理 P95=48.64 ms、最大积压 0.044 s，无积压故障；最终物理终点误差 9.25 cm，但顺序计数 1/7。六个中间/终点请求的最近 LIO 距离约为 0.930、0.498、1.152、1.516、0.561、0.018 m。原因是原 readGivenWps 只把全部点放进连续全局参考，局部规划的当前终点是规划范围内的后续参考点，并不强制经过所有中间点；原进度判断还使用计划 B 样条位置而不是实际里程计。EGO 的碰撞优化不等价于逐点任务约束。

在原 Modules/ego_planner_swarm/plan_manage 的 C++ FSM 新增可选 fsm/strict_waypoint_tracking，默认 false 保留旧连续模式；Simulation 显式启用 true。严格模式逐点调用原 planNextWaypoint，实际 PX4 里程计进入 min(no_replan_thresh,0.3 m) 才执行下一点；无法找到可执行点时停止而不跳过。当前目标进入感知范围后持续用原膨胀栅格/BFS 检查新出现的占据，重定位后同步 wps 与当前段终点。未改 Fast-LIO2、B 样条优化器、traj_server 的 1 秒前视、控制器、地图分辨率/安全距离或验收半径。原 isKnownOccupied 本身读取膨胀占据，不是普通原始占据；此修复没有替换 BFS 占据条件。生成的 module_alignment.json 明确记录 algorithms_modified=true 及任务调度变更，不能把修复后运行称为未修改基线。

| 实验 | GUI / 倍速 | 验收状态 | 到达计数（含起飞） | 终点物理误差 | LIO 位置 RMSE | 雷达帧数 |
| --- | --- | --- | --- | --- | --- | --- |
| 连续模式独立基线 | 无窗口 / 0.5 | failed | 1/7 | 9.25 cm | 4.40 cm | 648 |
| 严格六目标 | Gazebo + RViz / 0.5 | ok | 7/7 | 8.94 cm | 4.78 cm | 884 |
| 严格 15 目标 | Gazebo + RViz / 0.5 | ok | 16/16 | 10.13 cm | 5.43 cm | 4548 |

完整网格保持原 x=0..32/8、y=0..16/8、z=2/2 请求，全部 15 点顺序执行。EGO 将 (24,0,2) 调整到 (23.65,0.05,2.05)，偏移约 0.357 m；小网格 (4,2,2) 调整到 (3.85,1.75,1.75)，偏移约 0.384 m。两次报告分别保留原请求和接受坐标，并从记录的 LIO、独立物理轨迹按时间顺序检查原请求点的 0.5 m 半径，结果均 complete=true（6/6 与 15/15）；物理轨迹接受点顺序检查也通过。不是只凭调整后终点宣称原请求任务完成，也不表示精确穿过障碍坐标。

两次均满足原中间点 0.5 m 观察半径、终点 LIO 小于 0.2 m 且持续 3 秒仿真时间、最终物理误差小于 0.3 m、已解锁 OFFBOARD、参数与发布者审计。graph.module_publishers_valid=true、导航真值订阅为空；ULog 报告确认外部位置融合且无 GPS 依赖，双窗口无异常退出。完整任务整帧处理墙钟 P95=43.46 ms、最大积压 0.112 s，小网格 P95=60.60 ms、最大积压 0.076 s；未触发 0.4 s 保护，未使用陈旧点云或错误遮挡缓存。GUI 验证依据实际进程/数据和运行日志，没有截图外观验收。

相关规划模块 catkin_make -j2 重建通过，完整 43 个测试（森林 28、PX4 15）通过，Python 语法与 diff 格式检查通过。新增报告检查的测试覆盖乱序访问不能证明完成、触发前/半径边界不计入到达、重定位点到达不能证明原目标到达。源码保留原连续模式；本次通过的是半速、单机、严格任务模式 SITL，未验证 1 倍速、多机、实机动力学标定或实体飞控 HIL。

- [独立基线失败报告](../../../artifacts/px4_forest/grid_baseline_independent/report.md)
- [六目标通过报告](../../../artifacts/px4_forest/grid_strict_visual/report.md)
- [六目标原请求顺序证据](../../../artifacts/px4_forest/grid_strict_visual/requested_waypoint_evidence.json)
- [15 目标通过报告](../../../artifacts/px4_forest/grid_15_strict_visual/report.md)
- [15 目标原请求顺序证据](../../../artifacts/px4_forest/grid_15_strict_visual/requested_waypoint_evidence.json)
- [15 目标原节点话题审计](../../../artifacts/px4_forest/grid_15_strict_visual/graph.json)
- [严格模式编译日志](../../../artifacts/px4_forest/strict_waypoint_build.log)

早期入口的悬停、慢速修复、轻量控制和接入调试记录已归档到 [历史结果](../../../artifacts/px4_forest/historical_reports/validation.md)，不作为当前入口验收替代。
