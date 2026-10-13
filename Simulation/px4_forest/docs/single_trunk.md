# 单根树干 Gazebo 物理碰撞场景

这是独立接触机制测试，已验证正面、擦边、旁路与 6 m/s 撞击。不依赖原森林 PCD；森林简化几何接入见 [森林物理碰撞](forest_physical_collision.md)。

## 场景与物理链路

配置位于 config/single_trunk.json：树干圆柱固定在 x=2 m、y=0，半径 0.2 m、高 3 m。探针机体使用已固定版本 Iris 的机身质量/惯性、机身和四转子碰撞形状，刚性组合。仅给一次 1.5 m/s 初始速度，不持续强制位姿/速度；后续运动由 Gazebo ODE 接触求解器决定。

采用零重力隔离接触机制、2 ms 步长、3 s 模拟时长，没有 PX4、Fast-LIO2、EGO 或电机控制。接触插件记录位姿、速度、接触对象、位置、深度和力；不是用距离判定后人为停住模型。

- impact：y=0 正面撞击，要求有真实接触且机体停在树干前、速度显著下降。
- bypass：y=1 m 旁路，要求无接触并保持预期匀速运动。
- glancing：y=0.4 m 擦边，要求接触且产生速度、侧向位置或姿态响应。
- fast_impact：y=0、6 m/s，要求接触、减速且机体中心未穿过树干。

## 构建与运行

Ubuntu/WSL Bash，项目根目录：

```bash
source /opt/ros/noetic/setup.bash
source ~/workspace/ws_livox/devel/setup.bash
source devel/setup.bash
python3 Simulation/run.py single-trunk --build \
  --output artifacts/px4_forest/my_single_trunk
```

`--build` 只构建本项目 Gazebo 插件，不重编 PX4。已有 libmspace_single_trunk_probe.so 时可省略；输出目录必须不存在。每组 Gazebo 使用独立 Master，仅清理本次启动的进程，无窗口运行。

输出保存 single_trunk.world、states.csv、contacts.csv、gazebo.log、response.png、result.json 和 report.md。

## 2026-10-12 实跑

正面测试首次接触约 1.046 s，终点 x≈1.547 m（树干中心 x=2 m），速度从 1.5 m/s 降到约 0.00923 m/s。旁路无接触，3.002 s 时 x≈4.503 m，速度仍为 1.5 m/s。两组均满足各自断言。

- [物理响应报告](../../../artifacts/px4_forest/single_trunk_verified_20261012/report.md)
- [实跑结果](../../../artifacts/px4_forest/single_trunk_verified_20261012/result.json)

新增四组最终实跑均通过，证据见 [四组报告](../../../artifacts/px4_forest/single_trunk_four_cases_20261012/report.md)。擦边有 5 个接触点，末速度约 0.923 m/s；6 m/s 测试有 12 个接触点，末速度约 0.00954 m/s。

## 边界

树干刚性固定，参数是测试用接触参数，没有标定木材柔性或真机撞击；碰撞力数值不作为实机载荷预测。没有桨叶断裂、枝叶缠绕或机体损伤模型。森林入口另有短程集成证据，不代表完整网格、多机或 HIL 已通过。
