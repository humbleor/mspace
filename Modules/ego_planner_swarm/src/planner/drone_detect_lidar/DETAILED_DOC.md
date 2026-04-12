# drone_detect_lidar 模块详解

## 概述

`drone_detect_lidar` 是一个基于 LiDAR 点云特征配准的多无人机相对定位 ROS 包。该模块通过提取各无人机 LiDAR 点云中的几何特征（角点、平面点），在无人机之间共享特征信息，并利用 ICP（Iterative Closest Point）算法进行点云配准，从而实现无人机之间的相对位姿估计和位置误差计算。

### 与视觉方案对比

| 特性 | 视觉方案 (drone_detect) | LiDAR 方案 (本模块) |
|------|----------------------|-------------------|
| 视场角 | 有限 (~90°) | 全向 360° |
| 光照敏感性 | 高 | 无 |
| 作用距离 | 近场 (<10m) | 中远场 (50m+) |
| 特征稳定性 | 中等 | 高 |
| 计算方式 | 像素级搜索 | ICP 配准 |

---

## 系统架构

模块由 **两个 ROS 节点** 和 **三个核心库** 组成：

```
┌──────────────────────────────────────────────────────────────────┐
│                        单架无人机内部                              │
│                                                                  │
│  LiDAR 点云 ──► feature_extractor_node ──► feature_cloud (发布)   │
│       │                        │                                 │
│       │                        └─► 特征提取库 (feature_extractor)  │
│       │                              体素降采样 → 法向量估计        │
│       │                              → 曲率计算 → 角点/平面点提取   │
│       │                                                              │
│       ▼                                                              │
│  local_map ──► relative_loc_node ◄── received_feature_cloud (接收)  │
│                     │                       │                        │
│                     ├─► pose_fusion 库 ◄─────┘                        │
│                     │     邻居状态缓存 → ICP配准 → 位姿误差计算        │
│                     │           ↑                                    │
│                     │           └─► icp_wrapper 库                    │
│                     │               point-to-point / point-to-plane   │
│                     │               分层配准 / 质量评估                │
│                     │                                                │
│                     ▼                                                │
│         relative_pose / pose_error / icp_fitness (发布)              │
└──────────────────────────────────────────────────────────────────┘
```

### 跨机数据流

```
┌──────────────────────────┐          ┌──────────────────────────┐
│        Drone A           │          │        Drone B           │
│                          │          │                          │
│  LiDAR → 特征提取(200点)  │──Topic──►  接收特征云                │
│                          │          │         │                │
│                          │◄──Topic──  接收里程计                 │
│                          │          │         │                │
│  接收B的特征云 ◄──────────┼──────────┼─────────┘                │
│       │                  │          │                            │
│       ▼                  │          │                            │
│  ICP配准(A特征 vs B地图)  │          │                            │
│       │                  │          │                            │
│       ▼                  │          │                            │
│  相对位姿 + 位置误差       │          │   (B端同样流程)             │
└──────────────────────────┘          └──────────────────────────┘
```

---

## 核心组件

### 1. FeatureExtractor（特征提取库）

**头文件**: `include/drone_detect_lidar/feature_extractor.h`
**实现文件**: `src/feature_extractor.cpp`

#### 功能

从原始 LiDAR 点云中提取显著的几何特征点，压缩数据量以减小通信带宽。

#### 处理管线 (7 步)

```
原始点云 ──► 1.体素降采样 ──► 2.法向量估计 ──► 3.曲率计算
                                              │
                              ┌───────────────┴───────────────┐
                              ▼                               ▼
                        4.角点提取                       5.平面点提取
                    (曲率 > threshold)              (曲率低 + 法向主轴)
                              │                               │
                              └───────────────┬───────────────┘
                                              ▼
                                    6.特征点筛选
                                (非极大值抑制 + 距离过滤)
                                              ▼
                                    7.限制最大数量
                                  (按显著性排序裁剪)
```

#### 特征类型

| 类型 | 判定条件 | 物理含义 |
|------|---------|---------|
| CORNER (角点) | 曲率 > `corner_threshold` (默认 0.1) | 边缘、墙角、柱子等锐利结构 |
| PLANAR (平面点) | 曲率 < `planar_threshold` 且法向接近垂直/水平 | 墙面、地面、天花板等平面 |
| INVALID | 不满足以上条件 | 丢弃 |

#### 关键算法细节

- **体素降采样**: 使用 `pcl::VoxelGrid`，默认叶子尺寸 0.2m
- **法向量估计**: `pcl::NormalEstimation`，搜索半径为 `search_radius * 3` 个邻域点
- **曲率计算**: 基于邻域法向量的变化程度（非 PCA 曲率），计算公式为 `mean(1 - |dot(n_ref, n_nb)|)`
- **平面点筛选**: 额外要求法向 z 分量 `|nz| > 0.85`（水平面）或 `|nz| < 0.15`（垂直面）
- **距离过滤**: 特征点之间最小间距 `min_distance` (默认 0.3m)，保证特征空间分布均匀
- **Ring ID**: 根据点的垂直角度估算 LiDAR 线束编号，用于后续可能的线束滤波

#### 输出

最多输出 `max_features` (默认 200) 个特征点，以 `sensor_msgs::PointCloud2` 格式发布，其中：
- `x, y, z`: 三维坐标
- `normal_x, normal_y, normal_z`: 法向量
- `curvature`: 曲率值
- `intensity`: 特征类型编码 (0=CORNER, 1=PLANAR)

---

### 2. ICPWrapper（ICP 配准库）

**头文件**: `include/drone_detect_lidar/icp_wrapper.h`
**实现文件**: `src/icp_wrapper.cpp`

#### 功能

封装 PCL 的 ICP 算法，实现两帧点云之间的位姿配准。

#### 支持的配准模式

| 模式 | 适用场景 | 特点 |
|------|---------|------|
| Point-to-Point | 目标点云无法向或快速模式 | 点到点距离最小化 |
| Point-to-Plane | 目标点云有法向量（推荐） | 点到面距离最小化，精度更高 |
| Hierarchical (分层) | 初始位姿偏差较大 | 粗配准(大搜索范围) → 精配准(小搜索范围) |

#### 配准流程

```
输入: source_cloud (邻居特征点), target_cloud (自身局部地图), initial_guess (单位阵)
  │
  ├─ 检查点云有效性 (非空、点数 >= 4)
  │
  ├─ 检查目标点云法向量是否存在
  │   └─ 不存在 → 回退到 point-to-point
  │
  ├─ [可选] 分层配准
  │   ├─ 粗配准: 2x 对应距离，coarse_iterations 次迭代
  │   └─ 精配准: 正常参数，更严收敛阈值
  │
  └─ 单次配准
      ├─ 执行 ICP align
      │
      └─ 质量评估 (4 项指标)
          ├─ fitness_score: 平均最近邻距离 (越小越好)
          ├─ RMSE: 均方根误差
          ├─ inlier_count: 内点数量
          └─ inlier_ratio: 内点比例
```

#### 配准有效性判定

一个 ICP 结果被认为是有效的，需要同时满足：
1. `converged == true` (算法收敛)
2. `fitness_score < fitness_score_thresh` (默认 1.0)
3. `inlier_count >= min_inlier_count` (默认 5)

#### 质量评估

- **Fitness Score**: source 点经变换后到 target 最近点的平均距离
- **RMSE**: 均方根误差，比 fitness score 对异常值更敏感
- **Inlier Ratio**: 距离小于对应距离阈值 50% 的点占总点的比例

---

### 3. PoseFusion（位姿融合库）

**头文件**: `include/drone_detect_lidar/pose_fusion.h`
**实现文件**: `src/pose_fusion.cpp`

#### 功能

整合邻居无人机的特征云和里程计信息，与自身局部地图进行 ICP 配准，计算相对位姿误差。

#### 核心数据结构

```cpp
struct NeighborState {
  int drone_id;                      // 邻居 ID
  Eigen::Vector3d position;          // 邻居世界系位置
  Eigen::Quaterniond orientation;    // 邻居世界系姿态
  Eigen::Vector3d velocity;          // 邻居世界系速度
  ros::Time last_update_time;        // 最后更新时间
  std::vector<FeaturePoint> feature_cloud; // 邻居特征点
  bool has_feature;                  // 是否收到特征
};

struct PoseError {
  Eigen::Vector3d delta_position;    // 位置误差 (ICP 平移量)
  Eigen::Vector3d delta_euler;       // 欧拉角误差 (roll/pitch/yaw)
  Eigen::Vector3d corrected_position; // ICP 修正后的位置
  double fitness_score;              // ICP 适配度分数
  bool valid;                        // 是否有效
};

struct RelativePose {
  Eigen::Vector3d position;          // 相对位置
  Eigen::Quaterniond orientation;    // 相对姿态
  int sender_id, receiver_id;        // 发送者和接收者 ID
  double confidence;                 // 置信度 [0-1]
  bool valid;
};
```

#### 位姿计算流程

```
1. 接收邻居特征云 + 邻居里程计
   │
2. 更新 NeighborState 缓存
   │
3. 点云预处理:
   ├─ 邻居特征: FeaturePoint[] → PointCloudT (提取 xyz + 法线)
   ├─ 自身地图: ROS PointCloud2 → PCL
   ├─ 去 NaN/Inf 点 (removeNaNFromPointCloud)
   └─ VoxelGrid 降采样 (0.3m)
   │
4. 重叠区域裁剪:
   以自身 position 为中心，用 overlap_radius 裁剪邻居特征
   → overlap_source (在我附近的邻居特征点)
   target 用完整降采样自身地图，不裁剪（提供充足几何约束）
   │
5. 法线计算:
   对 target 点云计算法线 (KSearch=10)
   point-to-plane ICP 必需
   │
6. 精配准 ICP:
   输入: overlap_source (数十-数百点) vs my_map (数千点)
   对应点距离: max(0.5, overlap_radius×0.3) (收紧)
   方法: point-to-plane ICP
   初始猜测: 单位阵 (两点云已在同一 world 系)
   最大迭代: 50 次, 收敛阈值: trans_epsilon=0.0001
   │
7. 结果判定 (基础):
   converged=true AND fitness < 1.0 → 进入质量检查
   │
8. 质量验证 (4 项全部通过才输出):
   ├─ 基础: converged && fitness < threshold
   ├─ 内点比例: inlier_ratio > 40%
   ├─ 平移量上限: |translation| < 2.0m
   └─ 旋转量上限: |yaw| < 15°
   │
9. 输出:
   delta_position = icp_translation (ICP 平移量)
   delta_euler = icp_rotation 转欧拉角
   corrected_position = neighbor.position + icp_translation
   │
10. 计算置信度:
    confidence = converged_factor × fitness_factor × inlier_ratio × iter_factor
```

#### 置信度计算模型

```
confidence = converged × fitness_factor × inlier_ratio × (0.5 + 0.5 × iter_factor)

其中:
  converged = 0 (未收敛) 或 1 (收敛)
  fitness_factor = max(0, 1 - fitness_score / threshold)
  iter_factor = max(0, 1 - iterations / max_iterations)
```

#### 过期邻居清理

超过 `timeout` (默认 5s) 未更新的邻居状态会被自动清除，防止使用过期数据进行配准。

---

### 4. FeatureExtractorNode（特征提取节点）

**实现文件**: `src/feature_extractor_node.cpp`

#### 功能

封装 FeatureExtractor 库为 ROS 节点，订阅 LiDAR 点云和里程计，发布特征点云。

#### Topic 接口

| 方向 | Topic | 类型 | 说明 |
|------|-------|------|------|
| 订阅 | `~lidar_topic` | `sensor_msgs/PointCloud2` | 原始 LiDAR 点云 |
| 订阅 | `~odom_topic` | `nav_msgs/Odometry` | 自身里程计 (用于数据完整性检查) |
| 发布 | `~feature_cloud` | `sensor_msgs/PointCloud2` | 提取的特征点云 |
| 发布 | `~feature_cloud_debug` | `sensor_msgs/PointCloud2` | 调试用特征点云 (同 feature_cloud) |

#### 行为逻辑

- 仅在收到里程计后才开始特征提取（防止无位姿参考时发布无效特征）
- 按 `publish_frequency` (默认 5Hz) 频率处理并发布

---

### 5. RelativeLocNode（相对定位节点）

**实现文件**: `src/relative_loc_node.cpp`

#### 功能

封装 PoseFusion 库为 ROS 节点，接收邻居无人机的特征和里程计，与自身地图配准并发布位姿误差。

#### Topic 接口

| 方向 | Topic | 类型 | 说明 |
|------|-------|------|------|
| 订阅 | `~received_feature_cloud` | `sensor_msgs/PointCloud2` | 邻居特征云 |
| 订阅 | `~neighbor_odom` | `nav_msgs/Odometry` | 邻居里程计 |
| 订阅 | `~local_map` | `sensor_msgs/PointCloud2` | 自身局部地图 |
| 订阅 | `~odom_topic` | `nav_msgs/Odometry` | 自身里程计 |
| 发布 | `~relative_pose` | `geometry_msgs/PoseStamped` | 相对位姿 |
| 发布 | `~pose_error` | `geometry_msgs/Vector3` | 位置误差 (x, y, z) |
| 发布 | `~icp_fitness` | `std_msgs/Float32` | ICP 适配度分数 |

#### 行为逻辑

- 收到邻居特征云 + 邻居里程计时，立即送入 PoseFusion 缓存
- 定时器以 `compute_frequency` (默认 5Hz) 触发配准计算
- 数据完整性检查：需要 local_map、自身 odom、邻居特征 三者都到位才计算
- 邻居 ID 推导：双机场景下，UAV1 的邻居是 UAV2，UAV2 的邻居是 UAV1

---

## 算法完整流程

以 UAV2 接收 UAV1 特征为例，完整描述从 LiDAR 到相对位姿的数据流：

```
Step 1: 特征提取 (UAV1 自身)
─────────────────────────────
  LiDAR 点云 (数千点)
    │
    ▼
  FeatureExtractor::extractFromROS()
    ├── VoxelGrid filter (0.2m)     → 降采样到数百点
    ├── Normal estimation (K=15)    → 估计法向量
    ├── Curvature computation       → 计算法向变化曲率
    ├── Corner extraction           → 曲率 > 0.1 的点
    ├── Planar extraction           → 曲率 < 0.05 + 法向主轴
    ├── Distance filtering          → 最小间距 0.3m
    └─ Top-N selection             → 最多 200 个点
    │
    ▼
  /uav1/feature_cloud (发布)

Step 2: 跨机数据传输
─────────────────────────────
  /uav1/feature_cloud ──relay──► /uav2/received_feature_cloud
  /uav1/lidar_slam/odom ──relay──► /uav2/neighbor_odom

Step 3: 相对定位解算 (UAV2 端)
─────────────────────────────
  RelativeLocNode::computeTimerCallback():
    │
    ├─ 读取 /uav2/local_map (自身 LiDAR 点云, ~5000+ 点)
    ├─ 读取 /uav2/lidar_slam/odom (自身里程计)
    ├─ 读取 received_feature_cloud (UAV1 的 200 个特征点)
    ├─ 读取 neighbor_odom (UAV1 的里程计)
    │
    ▼
  PoseFusion::computePoseError(neighbor_id=1):
    │
    ├─ 点云预处理:
    │   ├─ 自身地图: 去 NaN → VoxelGrid(0.3m) 降采样
    │   └─ 邻居特征: FeaturePoint[] → PointCloudT
    │
    ├─ 重叠区域裁剪:
    │   以自身 position 为中心，裁剪邻居特征到 overlap_radius 内
    │   target 用完整降采样地图，不裁剪 (提供充足几何约束)
    │
    ├─ 精配准 ICP:
    │   source = 裁剪后的邻居特征 (数十-数百点)
    │   target = 降采样后的自身地图 (数千点)
    │   对应距离: max(0.5, overlap_radius×0.3)
    │   最大迭代 50 次, trans_epsilon=0.0001
    │   初始猜测: 单位阵 (两点云已在 world 系)
    │   → T_icp (ICP 变换矩阵)
    │
    ├─ 质量验证 (4 项):
    │   ├─ converged && fitness < 1.0
    │   ├─ inlier_ratio > 40%
    │   ├─ |translation| < 2.0m
    │   └─ |yaw| < 15°
    │
    ├─ 误差计算:
    │   delta_pos = icp_translation
    │   delta_euler = icp_rotation 转欧拉角
    │
    └─ 置信度: confidence = f(converged, fitness, inliers, iterations)
    │
    ▼
  发布:
    /uav2/relative_pose  → ICP 估计的 UAV1 相对 UAV2 的位姿
    /uav2/pose_error    → ICP 平移量 (odometry 漂移估计)
    /uav2/icp_fitness   → 配准质量分数
```

---

## 编译

```bash
cd ~/mspace
./compile.sh
```

### 依赖

- **ROS**: roscpp, std_msgs, sensor_msgs, geometry_msgs, nav_msgs, pcl_ros, pcl_conversions, message_filters, tf2, tf2_ros, eigen_conversions
- **第三方库**: Eigen3, PCL (common, io, filters, kdtree, search, registration), OpenCV

### 构建产物

| 产物 | 类型 | 说明 |
|------|------|------|
| `feature_extractor_node` | 可执行文件 | 特征提取节点 |
| `relative_loc_node` | 可执行文件 | 相对定位节点 |
| `libdrone_detect_lidar_feature_extractor` | 库 | 特征提取算法 |
| `libdrone_detect_lidar_icp_wrapper` | 库 | ICP 配准封装 |
| `libdrone_detect_lidar_pose_fusion` | 库 | 位姿融合 |

---

## 使用方法

### 方式一：双机仿真 (一键启动)

```bash
roslaunch drone_detect_lidar 2uav_lidar_detect_sim.launch
```

此 launch 文件会启动：
1. 地图发布器 (`lidar_map_generator`)
2. UAV1 完整栈 (LIO + 路径规划 + LiDAR 检测)
3. UAV2 完整栈 (LIO + 路径规划 + LiDAR 检测)
4. 通信桥接 (`rosmsg_tcp_bridge`)
5. RViz 可视化

两架无人机初始位置：
- UAV1: (0, 0, 0.1)
- UAV2: (3, -3, 0.1)

### 方式二：单机启动 (接入已有仿真/实机)

```bash
roslaunch drone_detect_lidar drone_detect_lidar.launch \
  drone_id:=1 \
  lidar_topic:=/uav1/pcl_render_node/cloud \
  odom_topic:=/uav1/lidar_slam/odom
```

此 launch 文件为每架无人机启动：
1. `feature_extractor_node` - 特征提取
2. `relative_loc_node` - 相对定位
3. topic relay - 邻居特征云和里程计的转发

### 方式三：实机部署

在实机上，需要修改通信方式（当前仿真使用 topic relay）：

1. 修改 `params.yaml` 中的 `broadcast_ip` 为实际局域网广播地址
2. 修改 `next_drone_ip` 为下一架无人机的 IP
3. 使用 `rosmsg_tcp_bridge` 或自定义 UDP 广播节点转发特征云和里程计
4. 确保各无人机 NTP 时间同步

---

## 参数配置

### 特征提取参数

| 参数 | 默认值 | 说明 | 调整建议 |
|------|--------|------|---------|
| `voxel_size` | 0.2 | 体素降采样尺寸 (m) | 特征太少→减小; 计算慢→增大 |
| `max_features` | 200 | 最大特征点数 | 增加提高精度但增加带宽 |
| `corner_threshold` | 0.1 | 角点曲率阈值 | 角点太多→增大; 太少→减小 |
| `planar_threshold` | 0.05 | 平面点法向方差阈值 | 平面点太多→减小 |
| `search_radius` | 5 | 法向量估计邻域点数 | 影响法向量估计质量 |
| `min_distance` | 0.3 | 特征点最小间距 (m) | 控制特征分布均匀性 |
| `scan_ring_num` | 32 | LiDAR 线数 | 根据实际 LiDAR 修改 |
| `publish_frequency` | 5.0 | 特征发布频率 (Hz) | 与 ICP 计算频率保持一致 |

### ICP 配准参数

| 参数 | 默认值 | 说明 | 调整建议 |
|------|--------|------|---------|
| `icp_max_iterations` | 50 | 最大迭代次数 | 不收敛→增大 |
| `icp_trans_epsilon` | 0.0001 | 变换收敛阈值 (m) | 通常不需要修改 |
| `icp_fitness_thresh` | 1.0 | 适配度失败阈值 | 误匹配多→减小 |
| `icp_corr_dist` | 1.5 | 对应点最大距离 (m) | 初始误差大→增大 |
| `icp_use_point_to_plane` | true | 使用 point-to-plane ICP | 推荐 true，精度更高 |
| `icp_min_inliers` | 5 | 最小内点数量 | 误匹配多→增大 |
| `icp_overlap_radius` | 8.0 | 重叠搜索半径 (m)，裁剪邻居特征 | 两机间距大→增大 |
| `icp_min_overlap_points` | 15 | 最小重叠点数 | 误匹配多→增大 |
| `icp_use_hierarchical` | false | 使用分层配准 | 初始误差大→启用 |

### 位姿融合参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `compute_frequency` | 5.0 | 位姿误差计算频率 (Hz) |
| `neighbor_timeout` | 5.0 | 邻居状态超时时间 (s) |
| `enable_motion_compensation` | true | 是否启用运动补偿 |
| `motion_comp_vel_threshold` | 0.1 | 速度阈值 (低于不补偿) |

---

## 可视化与调试

### RViz 查看

```bash
rviz -d $(rospack find drone_detect_lidar)/rviz/lidar_detect.rviz
```

预配置的 RViz 可以查看：
- 全局地图 (`/map_generator/global_cloud`)
- 各机 LiDAR 点云 (`/uav*/pcl_render_node/cloud`)
- 特征点云（角点/平面点用不同颜色）
- 相对位姿箭头
- 位置误差数值

### 查看 ICP 收敛情况

```bash
rostopic echo /uav1/icp_fitness
```

- `fitness < 0.5`：配准良好
- `fitness > 1.0`：配准失败（超过阈值被丢弃）

### 查看位姿误差

```bash
rostopic echo /uav1/pose_error
```

理想情况下误差应 `< 0.3m`。当前仿真环境下约 64% 的数据 < 1.0m，主要受限于特征提取的视角依赖性。

### 日志级别

```bash
# 启用详细日志
roslaunch drone_detect_lidar drone_detect_lidar.launch verbose_logging:=true

# 运行时调整
rosrun rqt_logger_level rqt_logger_level
```

---

## 注意事项

1. **初始位置要求**: 两机初始相对位置误差应 < 3m，否则 ICP 可能收敛到局部最优。启用 `icp_use_hierarchical` 可以放宽此限制。

2. **特征退化场景**: 在长走廊、空旷场地、均匀草坪等特征稀少环境中，特征提取质量会下降，导致配准精度降低。

3. **时间同步**: 建议所有无人机使用 NTP 或 PTP 时间同步。当前模块内置了运动补偿来处理时间差，但补偿精度有限。

4. **动态障碍物**: 当前版本未处理动态物体（行人、移动车辆等），可能在动态物体丰富的场景中导致配准偏差。

5. **通信带宽**: 特征云约 8KB/帧（200 点），5Hz 频率下约 40KB/s 带宽。里程计约 500B/帧，100Hz 下约 50KB/s。总体通信需求较低。

6. **计算性能** (Intel i7-8700K):

   | 阶段 | 耗时 |
   |------|------|
   | 特征提取 (5000 点→200 点) | ~15ms |
   | 点云预处理 (去 NaN + 降采样) | ~5ms |
   | 重叠搜索 + 法线计算 | ~10ms |
   | ICP 配准 (100 点 vs 1500 点) | ~5-20ms |
   | 位姿解算 | <1ms |
   | **总计** | **<50ms** |

7. **ICP 质量过滤**: 当前版本包含 4 项质量检查（收敛/内点比/平移量/旋转量），约 36% 的配尝试会被过滤。这是预期行为——宁可丢弃不可信结果，也不输出误导性数据。

---

## 目录结构

```
drone_detect_lidar/
├── CMakeLists.txt              # CMake 构建配置
├── package.xml                 # ROS 包依赖声明
├── README.md                   # 简要说明
├── DETAILED_DOC.md             # 详细设计文档
├── config/
│   └── params.yaml             # 所有可配置参数
├── include/
│   └── drone_detect_lidar/
│       ├── feature_extractor.h  # FeatureExtractor 类定义
│       ├── icp_wrapper.h        # ICPWrapper 类定义
│       └── pose_fusion.h        # PoseFusion 类定义
├── src/
│   ├── feature_extractor.cpp    # 特征提取实现
│   ├── icp_wrapper.cpp          # ICP 配准实现
│   ├── pose_fusion.cpp          # 位姿融合实现
│   ├── feature_extractor_node.cpp  # 特征提取 ROS 节点
│   └── relative_loc_node.cpp    # 相对定位 ROS 节点
├── launch/
│   ├── drone_detect_lidar.launch       # 单机启动文件
│   └── 2uav_lidar_detect_sim.launch    # 双机仿真启动文件
└── rviz/
    └── lidar_detect.rviz        # RViz 预配置
```
