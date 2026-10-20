# 森林 PCD 物理碰撞接入

森林入口默认开启简化树木物理碰撞。雷达继续使用完整原始 PCD/BVH；物理引擎使用拟合圆柱以及 Iris 原机身/转子碰撞形状。EGO 名义距离验收继续读取实际 obstacles_inflation，真实树木接触是另一项独立失败条件。

## 几何与坐标

当前使用[树干提取](trunk_extraction.md)：法线与离群筛选、截面 RANSAC、跨层关联、允许倾斜的三维圆柱拟合；弯曲树按相连圆柱段表达。地面缺失不删除可靠观测段，根部仅补到有支撑的局部地面；稀疏补检与标准尺度去重，粗枝需跨块合并并连接到可靠树干，不输出独立 PCA 矮干碎段。

当前固定地图接受 71 条树干假设、159 个树干段，其中 24 条有地面锚定、47 条只保留观测区间；数量不等于实测树木总数。独立短段为 0，本次没有粗枝通过完整筛选。参数、支撑、拒绝记录、地图 SHA-256 保存到 forest_geometry.json。默认预算 500 个形状，超限失败，不静默截断。

先按 scene 的平移/旋转转换到局部 world。Gazebo 机体初始位置稳定后，再平移几何到实际物理起点，通过 spawn_sdf_model 加入世界；读取 get_model_state 验证位置，保存 forest_geometry_alignment.json。forest.world 和 forest_collisions.sdf 保存本次实际模型。visual 与 collision 使用一致的尺寸和姿态；雷达细致几何仍使用原完整 PCD。

## 接触与验收

forest_plant 从 Gazebo ContactManager 读取真实接触，发布 `/sim/plant/tree_contacts`，记录对象、Gazebo 世界坐标、时刻、深度、力和力矩。接触响应由物理求解器产生，不人为固定机体。空帧证明观察器存活；没有帧或时间覆盖不足不能判无接触。

接触树木即暂停物理、任务 failed、退出码非零。`--keep-contact-open` 在有 GUI 时保留现场，关窗口或 Ctrl+C 后保存证据并清理本次进程。提前接触可能尚无 LIO/EKF 融合记录，失败报告仍生成并说明缺失。

新增输出：forest_geometry.json、forest_collisions.sdf、forest_geometry_alignment.json、tree_contacts.json、tree_contact_evidence.json，以及 result.json.tree_contact 和 report.md。

## 运行

Ubuntu/WSL Bash，已有插件时：

```bash
cd ~/workspace/mspace
bash Simulation/start.sh short --speed .25 --keep-contact-open
```

默认打开 Gazebo/RViz 并自动提取。首次使用前构建插件并运行四组单树场景：

```bash
source /opt/ros/noetic/setup.bash
source ~/workspace/ws_livox/devel/setup.bash
source devel/setup.bash
python3 Simulation/run.py single-trunk --build \
  --output artifacts/px4_forest/trunk_$(date +%Y%m%d_%H%M%S)
```

可先检查/缓存几何：

```bash
python3 Simulation/run.py forest-geometry \
  --scene artifacts/forest_lio/route_125_155.json \
  --output artifacts/px4_forest/geometry_$(date +%Y%m%d_%H%M%S)
```

生成 geometry.json 和 geometry_overview.png；用 --forest-geometry 复用时，文件必须匹配当前局部地图哈希。--no-tree-collision 只用于明确标记的无树木物理碰撞诊断运行。

## 实跑验证

- 单树正面、擦边、旁路和 6 m/s 撞击夹具验证了接触求解器响应；夹具不代表森林模型覆盖率，也不推广到任意速度或障碍尺寸。
- 当前提取的 159 个树干段完成双窗口加载与坐标核验，接触监测正常。该次运行因时限未完成任务，不能作为完整路线通过证据。
- 初始接触负向夹具验证真实接触会触发 failed/非零退出，并保存接触对象、位置、接触力及报告。

证据：

- [提取几何](../../../artifacts/px4_forest/forest_geometry_final_20261012/geometry.json)
- [几何对照图](../../../artifacts/px4_forest/forest_geometry_final_20261012/geometry_overview.png)
- [四组物理场景](../../../artifacts/px4_forest/single_trunk_four_cases_20261012/report.md)
- [森林最终短程](../../../artifacts/px4_forest/forest_physical_final_20261012/report.md)
- [真实接触失败](../../../artifacts/px4_forest/forest_physical_negative_20261012/report.md)
- [完整编译日志](../../../artifacts/px4_forest/forest_collision_compile_20261012.log)

## 边界

圆柱补全背面/内部是拟合假设，不等于实测完整树体。低支撑、非管状、被拒绝的障碍和部分枝叶仍无物理碰撞；PCD 距离验收独立检查完整点云。

圆柱拟合与地面估计不是语义真值。没有木材柔性、桨叶断裂、缠绕、结构损伤或真机接触参数标定。当前通过的是接触夹具与单机短程 0.25 倍速森林闭环，不代表完整网格、多机、1 倍速或 HIL 已验证。
