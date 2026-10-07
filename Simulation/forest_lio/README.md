# 森林 PCD / Mid-360 / Fast-LIO2 / EGO 仿真模块

完整流程与统一入口见 [Simulation 使用说明](../README.md)。本目录提供数据准备、传感器和轻量闭环；PX4 物理闭环由同目录下的 [px4_forest](../px4_forest/README.md) 调用这些工具。

在项目根目录的 Ubuntu/WSL Bash 执行。依赖 ROS Noetic、已构建的 livox_ros_driver2、
项目工作区，以及 Python numpy、scipy、matplotlib、yaml、rosbag。不需要 Gazebo。

```bash
source /opt/ros/noetic/setup.bash
source ~/workspace/ws_livox/devel/setup.bash
source devel/setup.bash
```

## 全量审计

```bash
python3 Simulation/run.py audit ~/bagfiles/yxb_20260208-104116.bag \
  --output artifacts/forest_lio/my_audit
```

导出 report.md、audit.json、雷达/IMU CSV、曲线、记录定位轨迹、实际回波角度直方图。
检查时序、序号间断、tag/line、零坐标、有效点、逐点时间和首尾IMU覆盖。
静止候选不等于确认静止，不能直接作为完整噪声密度/随机游走标定。
角度统计只含实际回波，无法恢复零坐标点的原始射线方向。

默认设备话题后缀 .181；用 --lidar-topic、--imu-topic、--reference-topic 改写。
每次使用新的空输出目录，脚本拒绝覆盖已有实验。

## 隔离回放

```bash
catkin_make --source Modules/fast_lio2 --build build/fast_lio2 -j1
python3 Simulation/run.py replay ~/bagfiles/yxb_20260208-104116.bag \
  --output artifacts/forest_lio/my_replay --rate 0.5
```

创建私有本地ROS master，只播放两个原始话题，不播放旧LIO、TF、MAVROS、规划或控制
指令，不启动实机控制节点。只清理脚本创建的进程组，关闭时推进停止的仿真时钟，
使LIO能保存日志。

输出 replayed_lio.tum、recorded_lio.tum、replay_result.json、forest_map.pcd、
有效参数、来源记录和内部日志。定位对比使用相同时间/坐标系，没有轨迹拟合。
记录LIO只用于重复性比较，不是独立真值。运行成功不表示定位性能达标。

--duration 30 表示从首个原始传感器记录时刻起回放30秒；--no-map 关闭额外地图积累。
--map-voxel 控制导出地图分辨率（默认0.15m）。map来自新回放配准点云，受漂移和
观测覆盖影响，异常区间可能污染地图。

独立入口为 Simulation/forest_lio/launch/mapping_mid360_replay.launch；手动使用时需先创建
output_root/Log 和 output_root/PCD。output_root 默认保持原来的 ROOT_DIR，
已有实机参数不变。建议使用runner保证时钟、ROS master和结果文件隔离。

## 生成模拟雷达与IMU

历史短距离验证采用前30秒局部地图；新增正常飞行段完整地图见后文。
地图原点必须对应解析轨迹的IMU初始原点，初始姿态为单位旋转。
不同来源PCD必须先确认坐标变换。

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=4 \
python3 Simulation/run.py generate \
  --map artifacts/forest_lio/my_replay/forest_map.pcd \
  --template-bag ~/bagfiles/yxb_20260208-104116.bag \
  --output artifacts/forest_lio/my_sim --duration 30

python3 Simulation/run.py replay artifacts/forest_lio/my_sim/sensors.bag \
  --lidar-topic /sim/mid360/lidar --imu-topic /sim/mid360/imu \
  --truth artifacts/forest_lio/my_sim/truth.tum \
  --output artifacts/forest_lio/my_sim_replay --rate 1
```

输出 CustomMsg、200Hz IMU、模拟已知位姿、truth.tum、参数和限制说明。
默认3秒静止后执行连续解析轨迹，平移幅度0.5m，包含转弯/升降。
IMU由同一轨迹的角速度与 R^T(a-g) 得到，加速度按Livox原始约定输出g量级。
生成与回放的 --config 必须一致。

默认 --renderer bvh 使用世界坐标空间索引，沿射线筛选全部可能命中的有限面片并求最近交点。
使用实测回波方向/时间模板，逐点计算解析位姿；--render-threads 控制原生查询线程数。
--surface-radius 控制面片模型几何。--renderer raster 保留旧角度栅格对照；
--subscan-ms、--angular-resolution-deg 仅对 raster 生效。
支持 range-noise-m、gyro-noise、acc-noise-g 和固定随机种子；默认零噪声，
这些参数是实验输入，尚未完成实测标定。

限制：
- 无回波原始方向不可恢复，不能称为完整Mid-360光学模型。
- 反射率来自模板，未模拟材质、植被透射、风动和多次回波。
- 点图孔洞不证明自由空间；表面片不能补回未采集的树木背面。
- 子扫描近似需要用更细时间分辨率检验。
- 解析运动没有动力学、控制、碰撞响应和地图覆盖区域判定。
- 同份数据建图再合成仅验证基础自洽，需要独立采集验证外部真实性。
- 模拟真值只描述固定PCD世界中的运动，不是原始森林飞行真值。

## 验证

```bash
python3 Simulation/run.py test
python3 -m py_compile Simulation/forest_lio/scripts/*.py
git diff --check
```

当前 28 项测试覆盖运动导数、逐点位姿、机体系角速度、静止比力、消息时间、
平面/孔洞/薄圆柱遮挡、盲区/量程、随机面片独立求交、多线程、传感器位移及定位缺口插值。
历史阶段报告位于 artifacts/forest_lio/historical_reports，新增渲染结果见 [候选面与性能报告](docs/forest_lio_renderer.md)。

## 在线 LIO / EGO 闭环（第二阶段）

先提取本次 bag 的分段及静止 IMU 统计：

```bash
python3 Simulation/run.py calibrate \
  --audit artifacts/forest_lio/audit_final \
  --output artifacts/forest_lio/my_calibration

OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=4 \
python3 Simulation/run.py closed-loop \
  --map artifacts/forest_lio/replay_smoke/forest_map.pcd \
  --template-bag ~/bagfiles/yxb_20260208-104116.bag \
  --output artifacts/forest_lio/my_closed_loop --duration 12

OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=4 \
python3 Simulation/run.py closed-loop \
  --map artifacts/forest_lio/replay_smoke/forest_map.pcd \
  --template-bag ~/bagfiles/yxb_20260208-104116.bag \
  --output artifacts/forest_lio/my_closed_loop_noise --duration 12 \
  --sensor-model artifacts/forest_lio/my_calibration/sensor_model.json --noise
```

calibrate_model.py 的时间窗口针对 yxb_20260208-104116.bag，其他 bag 需要重新确认分段。
用户确认 201.300 s 后主动切 MANUAL；手动模式不计作自主导航错误。
噪声输入是各轴样本标准差加陀螺均值，尚非噪声密度/随机游走完整标定；
不将静止加速度均值当作偏置，不自动推断距离噪声。

run_closed_loop.py 创建私有 ROS master，启动 Fast-LIO2、EGO 和轨迹服务器。
完整 PCD 只在传感器进程内使用，EGO 接收 /replay/lio/odom 和 /replay/lio/cloud。
真值只保存用于评估。扫描模板取 40–120 s 自主飞行训练段。
默认静止约 4 s 后向 /uav1/prometheus/ego/goal 发送 (-0.7,0,0.5) m 目标。
使用 --goal x y z 修改；坐标是地图对应的 IMU 初始世界坐标，不是 GNSS 坐标。
脚本只适用于此坐标约定，换地图必须先确定初始位置和姿态变换。
200 Hz 二阶跟踪器具有 1.5 m/s² 加速度限制，姿态固定；
IMU 比力由实际模拟加速度产生。当前不包含 PX4、旋翼姿态或碰撞接触模型。

默认 --renderer bvh：世界坐标索引固定，每条射线按当前点时刻位姿重新求交，不复用旧遮挡。
固定姿态在线运动模型按 200 Hz 历史对每个点插值位置，--render-threads 默认 4。
--renderer conservative 为原有静止缓存、运动每 10 ms 重建的对照模式。
--renderer full 全程重建；--renderer cached 是有明显运动遮挡差异的实验模式。
仿真时间按 200 Hz 推进，等待 LIO 处理后产生下一帧，并在下一 IMU 样本之前小幅推进时钟以唤醒 ROS。
当前 BVH 短距离测试已接近实时；渲染预算不等于完整系统实时或硬件在环保证。
控制指令回到传感器运动模型，形成在线反馈，不再是预生成 bag 的固定轨迹。

输出 result.json、lio.tum、truth.tum、graph.json、commands.csv、frames.csv、
配置快照和节点日志。目标误差阈值 0.2 m 为此短距离冒烟测试的通过条件，
不表示飞行性能要求；地图最近点距离也不是未观测空间的安全证明。
输出目录必须为空，只关闭本次创建的进程组，不启动 MAVROS 或实机控制节点。

渲染敏感性实验：

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=4 \
python3 Simulation/run.py benchmark \
  --map artifacts/forest_lio/replay_smoke/forest_map.pcd \
  --bag ~/bagfiles/yxb_20260208-104116.bag \
  --output artifacts/forest_lio/my_renderer_benchmark.json
```

对相同射线/解析运动比较子扫描时序、缓存、地图密度、角度分辨率和表面片半径。
结果衡量数值模型敏感性，不是对真实森林激光精度的验证。
实跑结果见 [第二阶段报告](../../artifacts/forest_lio/historical_reports/forest_lio_phase2.md)。

## 世界坐标候选面查询与剖析

默认 BVH 内核源文件位于 `native/surfel_bvh.cpp`。首次调用需要带 OpenMP 支持的 g++，
自动构建产物保存于 `artifacts/forest_lio/native/`，按源码与编译选项散列区分。
模型将面片视为不透明：先找最近的非负距离交点，再应用输出盲区门限；
盲区内近处物体仍会挡住远处面片，不把门限过滤当作透明性。
固定地图索引可复用，地图修改必须重建；PCA 法向、面片半径及采集覆盖仍限制物理真实性。

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=4 \
python3 Simulation/run.py profile \
  --map artifacts/forest_lio/replay_smoke/forest_map.pcd \
  --bag ~/bagfiles/yxb_20260208-104116.bag \
  --output artifacts/forest_lio/my_bvh_profile
```

输出 `profile.json`，记录模型内独立参考检查及位姿变换、输入整理、原生查询、
包围盒/面片测试数量和 1/2/4 线程耗时。报告只适用于指定地图和测量机器。
同样测试了面片半径与地图体素变化，仍会出现明显回波变化，不能宣称解决全部几何敏感性。
完整实跑结果见 [渲染改进报告](docs/forest_lio_renderer.md)。

## 正常飞行地图与观测证据

本次地图使用原 bag 起点后 40–195 s 的保守窗口。用户确认尾段切 MANUAL 为主动操作；
选取窗口只用于构建稳定实验场景。LIO 仍从首个原始传感器记录开始初始化。

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
python3 Simulation/run.py replay ~/bagfiles/yxb_20260208-104116.bag \
  --output artifacts/forest_lio/my_normal_map --duration 192.8 --rate 0.5 \
  --capture-clouds --map-window 40 195

OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
python3 Simulation/run.py coverage artifacts/forest_lio/my_normal_map/observations.bag \
  --output artifacts/forest_lio/my_coverage

OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=4 \
python3 Simulation/run.py surfaces \
  --map artifacts/forest_lio/my_normal_map/forest_map.pcd \
  --output artifacts/forest_lio/my_surfaces
```

--duration 从首个原始记录计时；本 bag 首记录约在起点后 2.132 s，192.8 s 回放结束于约 194.932 s。
--map-window 按原 bag 起点计时，只限制导出地图和 observations.bag，不裁掉 LIO 初始化数据。
observations.bag 保存窗口内新估计的 /lio/odom 与世界坐标 /lio/cloud；它不包含原飞行独立真值。

coverage 输出 coverage.npz、sensor_origins.csv 与 coverage.json。默认 0.3 m 体素、
每扫描最多抽样 1200 条实际回波方向、30 m 量程。稀疏标志位可重叠：

| 标志位 | 含义 |
| --- | --- |
| 1 | 射线中心线经过证据 |
| 2 | 实际回波端点 |
| 4 | 估计传感器所在体素 |
| 0 / 不存在 | 未知 |

**射线经过不证明整个体素或无人机体积可通行。** 无回波方向、遮挡后方及未抽样射线不补造。
配准点云采用扫描末端估计位姿近似传感器原点，覆盖受运动、LIO 漂移与配准误差影响。
当前只记录证据，不把它发布为规划自由空间。

surfaces 对已知薄面、细圆柱、孔洞以及同地图半径/密度变化进行验证。
输出 surfaces.json、fixed_surfaces.npz 与 adaptive_surfaces.npz；
后者保存逐点位置、法向、半径及 kinds（0=圆盘、1=法向不可靠的小球代理）。
自适应支持范围由局部间距、平面性和切向分布决定，是点云采样模型，尚非实测物理表面标定。

generate / closed-loop 可显式加 --surface-model adaptive，仅适用于 --renderer bvh；
默认 fixed 保留。closed-loop 的 --coverage 可加载覆盖文件，对模拟真值位置报告未知体素数量，
仅作评估，不馈入 LIO 或规划器，也不作碰撞安全判据。
使用新地图前仍需确认初始位置、姿态及局部坐标变换；原点附近的短距离冒烟目标不是完整森林航线。

报告汇总：

```bash
python3 Simulation/run.py report-map \
  --replay artifacts/forest_lio/my_normal_map \
  --coverage artifacts/forest_lio/my_coverage \
  --surfaces artifacts/forest_lio/my_surfaces \
  --output artifacts/forest_lio/my_map_report
```

实际结果见 [地图与几何报告](docs/forest_lio_map_geometry.md)。

## 在完整地图上执行实测参考路线

`prepare-scene` 从新回放 LIO 轨迹预先生成任务目标；默认窗口为本 bag 的 125–155 s。
保留起始位置和航向，用保持重力方向的平移/绕 Z 旋转建立局部坐标；
固定姿态模型将实测横滚/俯仰替换为水平姿态，原始姿态另存用于说明限制。

```bash
OPENBLAS_NUM_THREADS=1 python3 Simulation/run.py prepare-scene \
  --replay artifacts/forest_lio/normal_map_replay_v2 \
  --coverage artifacts/forest_lio/normal_map_coverage/coverage.npz \
  --window 125 155 --spacing 1.5 \
  --output artifacts/forest_lio/my_route.json

OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=4 \
python3 Simulation/run.py closed-loop \
  --map artifacts/forest_lio/normal_map_replay_v2/forest_map.pcd \
  --template-bag ~/bagfiles/yxb_20260208-104116.bag \
  --scene artifacts/forest_lio/my_route.json \
  --output artifacts/forest_lio/my_route_fixed --duration 80

OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=4 \
python3 Simulation/run.py closed-loop \
  --map artifacts/forest_lio/normal_map_replay_v2/forest_map.pcd \
  --template-bag ~/bagfiles/yxb_20260208-104116.bag \
  --scene artifacts/forest_lio/my_route.json --surface-model adaptive \
  --output artifacts/forest_lio/my_route_adaptive --duration 80
```

--bag-start 默认是本次原 bag 的起点，换数据必须显式设置。
scene 保存原地图/覆盖来源、原点、旋转和目标；运行器要求 --map 与 scene 一致。
scene 模式按目标范围和局部地图更新范围自动扩展 EGO 的 XY 地图尺寸，
写入本次 planner_config.yaml，并检查目标及机体代理未超出地图或地面/虚拟顶界。
原单目标默认地图尺寸保持原配置。
完整地图只在传感器渲染端转换为局部坐标，雷达仍按同一外参生成；
覆盖评估将模拟位置转回原地图坐标。truth.tum/LIO 位于局部坐标，
truth_world.tum 位于原地图坐标。没有将实测参考轨迹注入在线里程计或轨迹命令。

默认每 1.5 m 设置一个目标；切换目标依据 LIO 估计位置，中间到达范围 0.25 m、
最终 0.20 m。--duration 是运行上限；scene 任务全部到达后稳定 3 s 自动结束。
--scene 使用场景中的目标，替代单目标 --goal。原来的单目标入口仍可使用。

footprint_evidence.csv 每 0.1 s 保存球形机体代理相交体素的中心未知、体素总数、
未知数及回波端点证据数。--body-radius 默认 0.25 m，是实验代理尺寸；
球与体素包围盒相交只用于观测证据评估，未知比例为零也不证明已确认安全。
场景准备时的点图距离用于筛查候选路线，实际运行仍独立记录最近点距离，
没有碰撞接触或安全控制响应。

```bash
python3 Simulation/run.py report-route \
  --scene artifacts/forest_lio/my_route.json \
  --runs artifacts/forest_lio/my_route_fixed artifacts/forest_lio/my_route_adaptive \
  --output artifacts/forest_lio/my_route_report
```

实跑结果见 [较长路线闭环报告](../../artifacts/forest_lio/historical_reports/forest_lio_route.md)。
