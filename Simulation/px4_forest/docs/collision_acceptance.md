# 单机森林碰撞验收

森林入口在任务结束后暂停 Gazebo，检查完整高频物理轨迹，再决定是否通过和保留窗口。按用户约定，默认使用 EGO 的实际参数进行名义安全距离验收；失败或提前退出时也尽量生成证据。距离不足/证据不可用均不能取得 `result.json.status=ok`；hover 没有森林地图，碰撞项标为 not_applicable。

## 检测模型

### 默认 ego 模式：按规划参数验收

读取实际 ROS 参数 `/uav_1_ego_planner_node/grid_map/obstacles_inflation`；历史复核读取原 module_params.json，缺失或无效不使用硬编码替代。当前实验该参数为 0.25 m。检查机体中心轨迹与完整 PCD 点的最小距离，净间距 = 中心到点距离 − 该参数，**不再额外加 0.4 m 机体或 0.2 m surfel 支撑**。

这一结果属于规划安全距离验收，不是实际机体接触判定。EGO 点云回调的实现是栅格膨胀：XY 步数 ceil(inflation/resolution)，Z 固定展开一格；当前 resolution=0.1 m 时为 XY 三格、Z 一格。默认径向检查与该非球形栅格集合并不完全等价，更不能重现历史时刻的局部观测/占据状态；两者差异明确写入证据。完整静态地图只是独立对照，不输入规划器。

### 可选 proxy 模式：独立保守机体包络

- 机体采用以 base_link 为中心、半径 0.4 m 的保守球包络。当前 Iris 转子中心约 (±0.13, ±0.22, 0.023) m，桨半径 0.128 m，球包络覆盖机身和转子，不随姿态缩小。不是用户实机标定值。
- 每个森林 PCD 点采用半径 0.2 m 的闭球障碍代理，保守包围当前雷达渲染的 0.2 m surfel 支撑圆盘。含地图中所有点，不偷偷删除地面、枝叶或起飞区域；可能比实际树面更保守。
- 在与雷达渲染一致的局部 world 坐标检查物理轨迹，相邻真值样本间采用直线扫掠球。使用 KD-tree 候选查询和精确点到线段距离，不抽稀轨迹、不只检查离散端点；切触也判相交。
- 最小净间距 = 整条轨迹到最近 PCD 点的距离 − 机体半径 − 障碍代理半径。负值表示包络相交，不是实际撞击深度。
- 记录首次进入相交的时间、机体中心和障碍点。真值默认 250 Hz；超过 0.05 s 的采样间隔、缺失/无效/乱序轨迹不能证明通过。

两种距离模式均检查完整轨迹，不抽稀、不只检测端点；真值仅供独立验收。距离判断在飞行结束后执行，另有默认启用的 [森林物理碰撞](forest_physical_collision.md) 会在真实接触时暂停并判失败。PCD 缺失区域不证明自由空间，几何拟合也未覆盖所有枝叶。接触机制的独立验证见 [单树干场景](single_trunk.md)。

## 使用与证据

正常入口自动开启，无需额外选项：

```bash
bash Simulation/start.sh short --headless
bash Simulation/start.sh small --headless
```

默认 `--collision-model ego`。`--body-radius`、`--obstacle-radius` 只在 `--collision-model proxy` 中生效，proxy 障碍半径不允许小于当前 surfel 的 0.2 m。`--max-truth-gap` 在两种模式都生效。所有模式/参数进入 manifest，实际使用的距离写入 collision_evidence.json。

输出 `collision_evidence.json`，同时嵌入 result.json 的 collision 和 report.md。status 为 clear / collision / unavailable；ego 模式中的 collision 表示距离不足，不是已证实的树木接触。保存首次相交、最小间距位置、连续相交区间；区间数量不是撞树次数。整次飞行仍须通过其他导航验收。

可以复核已有实验，写入新证据文件，不修改旧 result.json：

```bash
python3 Simulation/run.py collision artifacts/px4_forest/grid_strict_visual \
  --output artifacts/px4_forest/new_collision_evidence.json
# 重现上一阶段的保守包络标准时显式选择 proxy：
python3 Simulation/run.py collision artifacts/px4_forest/grid_strict_visual \
  --model proxy --output artifacts/px4_forest/new_proxy_evidence.json
```

从项目根目录执行；原场景/地图仍需存在。此命令依据旧 result 的 flight_end_sim_s 截取 truth.tum，排除结束后节点清理期间的运动。证据保存转换后的完整地图 SHA-256，便于比较几何来源；旧地图若已更换，此次只能证明当前地图上的复核结果。碰撞或证据不可用返回非零，禁止覆盖已有输出。

历史导航任务的 ok 不自动升级为碰撞验收通过。新增测试包含两端安全但中间穿越、切触、机体侧面相交、初始相交、静止运动、首次接触顺序、采样缺失，以及与全量几何计算对照。

按 EGO 参数的两条历史路线诊断见 [原因定位](clearance_diagnosis.md)：六点通过；15 点存在一个名义距离不足区间。旧 proxy 结果仍保留，不能与当前标准混称。
