# 森林 PCD 物理碰撞接入

森林入口默认开启简化树木物理碰撞。雷达继续使用完整原始 PCD/BVH；物理引擎使用拟合圆柱以及 Iris 原机身/转子碰撞形状。EGO 名义距离验收继续读取实际 obstacles_inflation，真实树木接触是另一项独立失败条件。

## 几何与坐标

forest_geometry.py 对 PCD 做 0.07 m 拟合抽稀，雷达不抽稀。竖直柱状候选需满足至少 1.5 m 高度、圆残差、角度支撑和内点比例。剩余点的局部 PCA 管状支撑用于矮干片段和粗枝候选；并非把所有枝叶转成实体。

当前约 100.6 万点生成 29 个树干代理、76 个矮干片段、11 个粗枝候选，共 116 个圆柱。模型/片段数量不等于树木数量。参数、支撑、拒绝记录、地图 SHA-256 保存到 forest_geometry.json。默认预算 500 个形状，超限失败，不静默截断。

先按 scene 的平移/旋转转换到局部 world。Gazebo 机体初始位置稳定后，再平移几何到实际物理起点，通过 spawn_sdf_model 加入世界；读取 get_model_state 验证位置，保存 forest_geometry_alignment.json。forest.world 和 forest_collisions.sdf 保存本次实际模型。collision 与 visual 独立定义，树干显示为棕色，细致森林仍在 RViz 显示。

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

默认打开 Gazebo/RViz 并自动提取。旧版插件需要先构建并跑四组小场景：

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

生成 geometry.json 和 geometry_overview.png；用 --forest-geometry 复用，要求匹配当前局部地图哈希。--no-tree-collision 仅作明确旧模式对照，报告标记 disabled，不能称物理接触验收通过。

## 实跑验证（2026-10-12）

- 正面、擦边、旁路、6 m/s 正面撞击四组均通过。高速测试使用 2 ms 步长，断言机体中心未穿过树干，不推广到任意速度/任意薄障碍。
- 最终 116 形状全场景、单机短程、无窗口 0.25 倍速通过：接触监测 9,809 帧，最大间隔 4 ms；传感器整帧处理 P95≈41.7 ms，最大仿真积压 0.224 s，小于 0.4 s 保护。实际模型平移、发布者与真值隔离审计通过。
- 负向接触夹具显式放置与初始机体相交的圆柱，标为 validation_fixture，不冒充 PCD 提取。真实接触触发 failed/非零退出，保存对象、位置、接触力及报告。
- 69 项测试、两个 Gazebo 插件构建通过；./compile.sh 六模块汇总全部 OK。

证据：

- [提取几何](../../../artifacts/px4_forest/forest_geometry_final_20261012/geometry.json)
- [几何对照图](../../../artifacts/px4_forest/forest_geometry_final_20261012/geometry_overview.png)
- [四组物理场景](../../../artifacts/px4_forest/single_trunk_four_cases_20261012/report.md)
- [森林最终短程](../../../artifacts/px4_forest/forest_physical_final_20261012/report.md)
- [真实接触失败](../../../artifacts/px4_forest/forest_physical_negative_20261012/report.md)
- [完整编译日志](../../../artifacts/px4_forest/forest_collision_compile_20261012.log)

## 边界

圆柱补全背面/内部是拟合假设，不等于实测完整树体。低支撑、非管状、被拒绝的障碍和部分枝叶仍无物理碰撞。历史 15 点危险位置没有通过当前可靠树干条件，不能声称该处已由物理模型修复；PCD 距离验收仍检查此处。

粗枝/矮干 PCA 分类不是语义真值。没有木材柔性、桨叶断裂、缠绕、结构损伤或真机接触参数标定。当前通过的是接触夹具与单机短程 0.25 倍速森林闭环，不代表完整网格、多机、1 倍速或 HIL 已验证。
