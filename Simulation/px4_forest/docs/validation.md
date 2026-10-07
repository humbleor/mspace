# 当前 PX4 森林网格验收

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
