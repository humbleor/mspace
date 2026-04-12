# Drone Detect LiDAR

基于 LiDAR 点云特征配准的多无人机相对定位系统。

## 概述

本模块通过提取 LiDAR 点云中的特征（角点、平面点），并在多机之间进行特征共享和 ICP 配准，实现无人机之间的相对定位和位姿误差估计。

### 核心优势

| 特性 | 视觉方案 (drone_detect) | LiDAR 方案 (本模块) |
|------|----------------------|-------------------|
| 视场角 | 有限 (~90°) | 全向 360° |
| 光照敏感性 | 高 | 无 |
| 作用距离 | 近场 (<10m) | 中远场 (50m+) |
| 特征稳定性 | 中等 | 高 |
| 计算负载 | 像素级搜索 | ICP 配准 |

---

## 目录结构

```
drone_detect_lidar/
├── CMakeLists.txt              # 构建配置
├── package.xml                 # ROS 包依赖
├── README.md                   # 本文档
├── DETAILED_DOC.md             # 详细设计文档
├── config/
│   └── params.yaml             # 参数配置
├── include/
│   └── drone_detect_lidar/
│       ├── feature_extractor.h  # 特征提取器
│       ├── icp_wrapper.h        # ICP 配准封装
│       └── pose_fusion.h        # 位姿融合
├── src/
│   ├── feature_extractor.cpp    # 特征提取实现
│   ├── icp_wrapper.cpp          # ICP 实现
│   ├── pose_fusion.cpp          # 位姿融合实现
│   ├── feature_extractor_node.cpp  # 特征提取节点
│   └── relative_loc_node.cpp    # 相对定位节点
├── launch/
│   ├── drone_detect_lidar.launch       # 单无人机启动
│   └── 2uav_lidar_detect_sim.launch    # 双机仿真启动
└── rviz/
    └── lidar_detect.rviz      # RViz 配置
```

---

## 快速开始

### 1. 编译

```bash
cd ~/mspace
./compile.sh
```

### 2. 双机仿真

```bash
roslaunch drone_detect_lidar 2uav_lidar_detect_sim.launch
```

两架无人机初始位置：
- UAV1: (0, 0, 0.1)
- UAV2: (3, -3, 0.1)

### 3. 单机启动 (接入已有仿真/实机)

```bash
# 启动仿真环境
roslaunch ego_planner 2uav_mid360_sim.launch

# 启动 LiDAR 检测模块
roslaunch drone_detect_lidar drone_detect_lidar.launch drone_id:=1
```

---

## 算法流程

```
┌─────────────────────────────────────────────────────────────────┐
│                    Drone A                                        │
│  LiDAR 点云 → 体素降采样 → 法向量估计 → 特征提取 → 200 个特征点   │
│                              │                                    │
│                              ▼                                    │
│                    UDP 广播 (sensor_msgs::PointCloud2)            │
│                              │                                    │
└──────────────────────────────┼────────────────────────────────────┘
                               │
                               ▼
┌─────────────────────────────────────────────────────────────────┐
│                    Drone B                                        │
│  接收特征点云 → ICP 配准 → 位姿解算 → 误差计算 → 发布             │
│                              │                                    │
│  自身 LiDAR → 局部地图 ──────┘                                    │
└─────────────────────────────────────────────────────────────────┘
```

### ICP 配准策略

两个点云都在 world 系下。ICP 的目标是检验两套世界系点云是否自然对齐：

1. **Source**: 邻居特征点（最多 200 个），裁剪到自身 position 附近
2. **Target**: 自身完整降采样局部地图（数千点），**不裁剪**，提供充足几何约束
3. **初始猜测**: 单位阵（两个点云已在同一坐标系）
4. **方法**: point-to-plane ICP
5. **质量验证**（4 项检查全部通过才输出）:
   - 收敛 + fitness < 1.0
   - 内点比例 > 40%
   - 平移量 < 2.0m
   - yaw 旋转 < 15°

---

## 配置参数

### 特征提取参数 (`config/params.yaml`)

| 参数 | 默认值 | 说明 |
|------|-------|------|
| `voxel_size` | 0.2 | 体素降采样尺寸 (m) |
| `max_features` | 200 | 最大特征点数 |
| `corner_threshold` | 0.1 | 角点曲率阈值 |
| `planar_threshold` | 0.05 | 平面点阈值 |
| `publish_frequency` | 5.0 | 发布频率 (Hz) |

### ICP 配准参数

| 参数 | 默认值 | 说明 |
|------|-------|------|
| `icp_max_iterations` | 50 | 最大迭代次数 |
| `icp_trans_epsilon` | 0.0001 | 收敛阈值 |
| `icp_fitness_thresh` | 1.0 | 适配度阈值 |
| `icp_corr_dist` | 1.5 | 对应点最大距离 |
| `icp_use_point_to_plane` | true | 使用 point-to-plane |
| `icp_overlap_radius` | 8.0 | 重叠搜索半径 (m)，决定 source 裁剪范围 |
| `icp_min_overlap_points` | 15 | 最小重叠点数 |

---

## ROS 接口

### 订阅 Topics

| Topic | 类型 | 说明 |
|-------|------|------|
| `~lidar_topic` | `sensor_msgs/PointCloud2` | 原始 LiDAR 点云 |
| `~odom_topic` | `nav_msgs/Odometry` | 自身里程计 |
| `~received_feature_cloud` | `sensor_msgs/PointCloud2` | 邻居特征云 |
| `~neighbor_odom` | `nav_msgs/Odometry` | 邻居里程计 |
| `~local_map` | `sensor_msgs/PointCloud2` | 自身局部地图 |

### 发布 Topics

| Topic | 类型 | 说明 |
|-------|------|------|
| `~feature_cloud` | `sensor_msgs/PointCloud2` | 提取的特征云 |
| `~relative_pose` | `geometry_msgs/PoseStamped` | 相对位姿 |
| `~pose_error` | `geometry_msgs/Vector3` | 位置误差 |
| `~icp_fitness` | `std_msgs/Float32` | ICP 适配度 |

---

## 性能指标

### 计算性能 (Intel i7-8700K)

| 阶段 | 耗时 |
|------|------|
| 特征提取 (5000 点→200 点) | ~15ms |
| 点云预处理 (去 NaN + 降采样) | ~5ms |
| 重叠搜索 + 法线计算 | ~10ms |
| ICP 配准 (100 点 vs 1500 点) | ~5-20ms |
| 位姿解算 | <1ms |
| **总计** | **<50ms** |

### 通信带宽

| 消息类型 | 大小 | 频率 | 带宽 |
|---------|------|------|------|
| 特征云 (200 点) | ~8KB | 5Hz | 40KB/s |
| 里程计 | ~500B | 100Hz | 50KB/s |

---

## 注意事项

1. **初始位置要求**: 两机初始相对位置误差应 < 3m，否则 ICP 可能收敛到局部最优。启用 `icp_use_hierarchical` 可以放宽此限制。

2. **特征退化场景**: 在长走廊、空旷场地、均匀草坪等特征稀少环境中，特征提取质量会下降，导致配准精度降低。

3. **时间同步**: 建议所有无人机使用 NTP 或 PTP 时间同步。当前模块内置了运动补偿来处理时间差，但补偿精度有限。

4. **动态障碍物**: 当前版本未处理动态物体（行人、移动车辆等），可能在动态物体丰富的场景中导致配准偏差。

5. **ICP 质量过滤**: 当前配置下约 64% 的配准结果位置误差 < 1m（仿真环境），剩余约 36% 的结果因 ICP 漂移被过滤丢弃。进一步改善需要改用完整点云代替特征点做 source（架构级改动）。

---

## 调试建议

### 查看特征点质量

```bash
rviz -d $(rospack find drone_detect_lidar)/rviz/lidar_detect.rviz
# 添加 Feature Cloud 点云查看
```

### 查看 ICP 收敛情况

```bash
rostopic echo /uav1/icp_fitness
# fitness < 0.5 表示配准良好
# fitness > 1.0 表示可能失败
```

### 查看位姿误差

```bash
rostopic echo /uav1/pose_error
# 理想情况下误差应 < 0.3m
```

### 运行时调整日志级别

```bash
rosrun rqt_logger_level rqt_logger_level
```

---

## 参考资料

1. Besl, P.J. and McKay, N.D., "A method for registration of 3-D shapes", TPAMI 1992.
2. Rusinkiewicz, S. and Levoy, M., "Efficient variants of the ICP algorithm", 3DIM 2001.
3. Fast-Planner: https://github.com/HKUST-Aerial-Robotics/Fast-Planner

---

## License

GPLv3
