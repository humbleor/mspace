# 仿真规划地图内存修复（2026-10-13）

## 原因

旧仿真复制实机模板的 200 × 200 × 10 m 地图，分辨率 0.1 m，共 4 亿体素。GridMap 的六个稠密缓冲每体素约 15 字节，合计约 5.59 GiB，不含定位、点云、Gazebo/RViz 和其他规划开销。用户失败日志中 EGO 退出码 −9，内核另有同名进程 OOM-killer 记录（约 5.7 GiB RSS）。降速不改变这些缓冲大小。

## 实现

仅在 Simulation/px4_forest/scripts/module_launch.py 生成的 planner_modules.launch 中覆盖地图尺寸；原 Modules launch 不改。

- 纳入原点、全部参考航点，以及网格 min/max；不会只按第一个点生成地图。
- XY 保持 GridMap 的对称原点。每侧余量取实际 local_update_range + local_map_margin × resolution + planning_horizon + 1 m。
- 按约 8 m 的偶数体素块向上取整，XY 最小 32 m；当前 0.1 m 分辨率下维持格点相位。
- Z 保留模板高度，需要时扩展以覆盖任务；低于原 ground_height 的目标明确报错。
- 分辨率、障碍膨胀、规划速度、控制器、雷达和物理碰撞参数保持原值。
- 原尺寸、任务边界、余量、实际尺寸和缓冲内存估算保存到 module_alignment.json 的 simulation_map_layout，也进入 result.json。
- 必需进程退出时显示父进程日志名、PID、退出码和日志路径；若 ROS launch 子节点失败，继续查看该日志中的子节点原因。

完整 32 × 16 m 网格生成 **96 × 64 × 10 m**，6,144 万体素；六个缓冲估算 **921,600,000 字节，约 0.86 GiB**。地图范围覆盖全部目标与每侧 14 m 规划余量；物理森林和雷达仍使用完整 PCD。任务很大时此方案仍会增加稠密地图内存，不是滚动/稀疏地图实现。

## 验证

- 全部 74 项测试通过：森林 28、PX4/地图等 46。新增覆盖完整网格、负坐标、实际局部范围依赖、非法目标、Z 扩展与原模板文件未被修改。
- 完整网格配置，Gazebo + RViz，0.25 倍速，40 s 限定时长验证：实际 ROS 参数为 96/64/10、resolution=0.1、obstacles_inflation=0.25；EGO 进入任务后实测 RSS 1,043,224 kB（约 1.0 GiB），VmSwap=0。
- 398 帧雷达，保持解锁 OFFBOARD，导航发布者/真值隔离及物理接触观察器检查完成。没有 OOM、必需进程退出、位姿历史不足或传感器积压保护错误。
- **该运行不是完整网格通过记录**。40 s 时只到达起飞与首个目标，result.status=failed，唯一错误为任务未在限定时间内到终点；这是启动/内存验证，未放宽任务验收。

证据：

- [启动验证报告](../../../artifacts/px4_forest/map_memory_full_startup_20261013/report.md)
- [实际参数和内存观测](../../../artifacts/px4_forest/map_memory_full_startup_20261013/memory_validation.json)
- [模块绑定与估算](../../../artifacts/px4_forest/map_memory_full_startup_20261013/module_alignment.json)

## 用户运行

项目根目录 Ubuntu/WSL Bash，输出新目录：

```bash
RUN="artifacts/px4_forest/full_grid_$(date +%Y%m%d_%H%M%S)"
bash Simulation/start.sh full --speed .25 --instance 53 --output "$RUN"
```

默认 600 s 上限、双窗口。参数变更属于 Python/生成 launch，无需重编 C++。完整 15 点物理航线仍需本次新配置的实际验收；内存修复不代表所有障碍、定位或轨迹问题都已解决。
