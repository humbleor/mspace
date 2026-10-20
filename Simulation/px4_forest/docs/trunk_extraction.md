# 树干提取与覆盖补检

当前方法从森林 PCD 提取有观测支撑的树干与连接枝段，生成 Gazebo 物理碰撞几何。保持原 PCD 雷达、坐标转换、Iris 机体碰撞与接触参数，不改变导航算法。tree_id 代表几何树干假设，不保证每个 ID 都是一棵不同的实树。

## 方法

- 标准截面 0.5 m，并增加 0.9 m 稀疏补检截面。补检采用至少 8 个截面点、至少 3/12 个角度区间和至少 2 m 高度；标准尺度的可靠拟合保留原参数。
- 只有平均邻距大于 0.12 m 的稀疏点，才允许曲率阈值从 0.09 调到 0.18；法线方向与拟合后的一致性检查仍保留。没有把整个森林的筛选条件统一放宽。
- 按轴线、半径、相交高度去重两个尺度的观测。高度层占用率下限为 0.5；标准尺度最大层间距 1 m，稀疏尺度 1.5 m。大缺口不生成没有观测支撑的长连接。
- 地面支撑只控制补根，不再决定是否保留已通过圆柱检查的实测树段。ground_status=anchored 表示根部接到局部拟合坡地；unobserved / extension_limit 表示保留观测区间，不补未知根部。已知地面仍最多延长 1.5 m，不强行补到 Gazebo 平面 z=0。
- 径向内点门槛仍为 0.05 m，分段连接后重新检查。粗枝仍需可靠表面及树干连接，本轮没有粗枝通过筛选。
- 每条关联轨迹均记录接受、拟合失败、支撑失败、重复或高度不足等原因；不再静默跳过早期短候选。段数上限仍为 500，超限失败。

当前使用 NumPy/SciPy 离线处理累积 PCD，无新增 PCL/C++ 编译依赖。几何结果记录根部状态、补检来源、逐候选诊断和分组计数；模型使用尺寸一致的 visual / collision。缓存几何应与对应地图哈希匹配。

## 当前地图结果

固定地图接受 71 条树干假设、159 个树干段，其中 24 条有地面锚定，47 条只保留观测区间。数量不是人工标注的树木总数或召回率。

对固定的 221580 个竖直表面候选点，以距有限圆柱侧面 0.08 m 为同一门槛：
- 当前模型支撑 12279 点。此集合也包含枝叶/墙面等，数值只是观测支撑代理，不能解释成真实森林覆盖率。
- 段内支撑径向残差中位数约 0.0187 m；没有人工标注，不能据此声称几何精度或完整召回率。
- 提取耗时约 152.6 s；离线准备后可复用与地图哈希匹配的缓存。

## 运行与检查

复用本次已经生成并核对地图哈希的新几何：

```bash
RUN="artifacts/px4_forest/trunks_recall_$(date +%Y%m%d_%H%M%S)"
bash Simulation/start.sh full --speed .25 --instance 67 \
  --forest-geometry artifacts/px4_forest/trunks_recall_checked_20261016/geometry.json \
  --output "$RUN"
```

无需重新编译。不指定 --forest-geometry 时自动执行当前提取流程，启动日志会提示正在离线提取；首次准备需要等待处理完成。

离线重新提取并比较：

```bash
OUT="artifacts/px4_forest/trunks_geometry_$(date +%Y%m%d_%H%M%S)"
python3 Simulation/px4_forest/scripts/forest_geometry.py \
  --scene artifacts/forest_lio/route_125_155.json --output "$OUT"
```

输出 geometry.json、forest_collisions.sdf、geometry_overview.png、tree_closeups.png，以及：
- candidate_diagnostics.csv：每条轨迹的位置、接受/拒绝原因、尺度、根部状态。
- candidate_map.png：接受位置及拒绝阶段统计，棕色有根部支撑，蓝色仅观测段；这些颜色用于诊断，Gazebo 仍显示棕色树干。
- extraction_summary.json：模型数、各阶段统计与拟合残差。
- coverage_comparison.json / png：使用 --compare-geometry 时生成，必须使用相同局部地图哈希。

运行时同样把候选与拒绝记录保存在 forest_geometry.json。

## 验证与边界

最终 159 个 visual / collision 在双窗口 15 s 加载验证中逐个核对一致，坐标验证通过；雷达 148 帧，OFFBOARD 解锁，接触监测正常。报告只因限时未完成任务而 failed，完整路线没有验收。之前扩大模型的 35 s 检查也正常开始任务。

93 项测试覆盖稀疏法线补检、跨尺度去重、无地面保留且不虚构根部、超长缺口不桥接、相邻树、倾斜/弯曲、枝叶与墙面误检，以及覆盖指标的有限侧面/掩码/并集语义。

候选及局部叠加图有助于排查漏检，但尚无逐树人工标注，仍不能声称完整森林碰撞覆盖。地面不确定时可能保留悬空外观的实测树段；不会为外观任意延长。Gazebo 的原物理地面仍是平面，本次没有重建完整坡地、冠层或木材响应。

证据：
- [最终提取与候选诊断](../../../artifacts/px4_forest/trunks_recall_checked_20261016/candidate_map.png)
- [覆盖对照](../../../artifacts/px4_forest/trunks_recall_checked_20261016/coverage_comparison.png)
- [逐项对比数据](../../../artifacts/px4_forest/trunks_recall_checked_20261016/coverage_comparison.json)
- [几何](../../../artifacts/px4_forest/trunks_recall_checked_20261016/geometry.json)
- [最终双窗口加载验证](../../../artifacts/px4_forest/trunks_recall_checked_sitl_20261016/report.md)
- [35 秒扩大模型运行](../../../artifacts/px4_forest/trunks_recall_sitl_20261016/report.md)
