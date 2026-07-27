# Drone Detect LiDAR

基于树干特征匹配的森林场景多无人机相对定位系统。

## 概述

本模块通过检测 LiDAR 点云中的树干位置，并在多机之间进行三角形哈希匹配和 2D 加权 SVD 配准，实现无人机之间的相对定位和位姿误差估计。可选地，通过 iSAM2 增量因子图优化多机对间的相对位姿。

### 核心优势

| 特性 | 视觉方案 (drone_detect) | 特征点 ICP 方案 | 树干匹配方案 (本模块) |
|------|----------------------|----------------|-------------------|
| 视场角 | 有限 (~90°) | 全向 360° | 全向 360° |
| 光照敏感性 | 高 | 无 | 无 |
| 特征稳定性 | 中等 | 低（视角依赖） | 高（视点无关） |
| 配准算法 | 特征匹配 | 迭代优化（易陷局部最优） | SVD 闭式解（无局部最优） |
| 适用场景 | 室内/室外 | 开阔建筑环境 | 森林林冠下 |

### 为什么不用 ICP？

在森林林冠下环境中，角点/面点特征提取存在**视角依赖性**——两架无人机从不同位置观察同一棵树，提取的角点和面点不是同一批物理点，导致 ICP 匹配错误、fitness score 异常。本模块使用**树干质心 XY 坐标 + 最低点 Z** 作为特征，在同一高度飞行时具有较好的一致性。

---

## 目录结构

```
drone_detect_lidar/
├── CMakeLists.txt              # 构建配置
├── package.xml                 # ROS 包依赖
├── README.md                   # 本文档
├── config/
│   └── params.yaml             # 参数配置
├── include/
│   └── drone_detect_lidar/
│       ├── tree_detector.h      # 树干检测器
│       ├── triangle_matcher.h   # 三角形匹配数据结构
│       └── patchworkpp.h        # PatchWork++ 地面剔除
├── msg/
│   ├── Tree.msg                 # 单棵树干消息
│   ├── TreeDetection.msg        # 树检测结果消息
│   ├── TreeRelativePose.msg     # 两机相对位姿测量消息
│   ├── TreeRelativePoseOptimized.msg  # iSAM2 优化后的相对位姿消息
│   └── DriftCorrection.msg      # 漂移修正诊断消息
├── src/
│   ├── tree_detector.cpp        # 树干检测实现（库）
│   ├── tree_detector_node.cpp   # 树干检测 ROS 节点
│   ├── tree_loc_node.cpp        # 树匹配定位节点
│   ├── tree_graph_node.cpp      # iSAM2 因子图优化节点 (GTSAM)
│   └── patchworkpp.cpp          # PatchWork++ 地面剔除
├── Scripts/
│   ├── evaluate_drift_estimation.py      # 漂移估计精度评估
│   ├── evaluate_correction_offline.py    # 修正效果评估
│   ├── run_tree_drift_eval.sh            # 一键测试脚本
│   ├── tree_drift_injector.py            # 漂移注入工具
│   └── analyze_tree_pose_error.py        # tree_pose_error 数值统计
├── launch/
│   ├── drone_detect_lidar.launch       # 单无人机启动（被多机 launch 包含）
│   ├── 2uav_lidar_detect_sim.launch    # 双机仿真启动
│   └── 3uav_lidar_detect_sim.launch    # 三机仿真启动 (含 iSAM2 因子图优化)
└── rviz/
    └── lidar_detect.rviz      # RViz 配置
```

---

## 快速开始

### 1. 编译

```bash
cd ~/workspace/mspace
./compile.sh
```

### 2. 双机仿真

```bash
roslaunch drone_detect_lidar 2uav_lidar_detect_sim.launch
```

两架无人机在林下飞行，初始位置：
- UAV1: (0, 0, 0.1)
- UAV2: (8, -8, 0.1), yaw=90°

### 3. 三机仿真 (含 iSAM2 因子图优化)

```bash
roslaunch drone_detect_lidar 3uav_lidar_detect_sim.launch
```

三架无人机初始位置：
- UAV1 (anchor): (0, 0, 0.1)
- UAV2: (3, -3, 0.1), yaw=90°
- UAV3: (-3, 3, 0.1), yaw=-90°

UAV1 作为 anchor 启动 `tree_graph_node`，使用 iSAM2 对所有 UAV 对的相对位姿进行增量式因子图优化。

### 4. 查看输出

```bash
# 查看树检测结果
rostopic echo /uav1/tree_detection

# 查看树干匹配位姿误差
rostopic echo /uav1/tree_pose_error

# 查看 iSAM2 优化后的相对位姿 (三机仿真)
rostopic echo /uav1/tree_relative_pose_optimized
```

---

## 算法流程

系统由两个或三个独立运行的节点组成：

```
┌─────────────────────────────────────────────────────────────────┐
│                    Drone A (tree_detector_node)                   │
│                                                                  │
│  LiDAR 点云 → 多帧积累 → 体素降采样 → PatchWork++地面剔除         │
│                      → 欧氏聚类 → PCA多条件筛选                  │
│                      → 树干位置计算 (质心XY + 最低Z)              │
│                      → 树干参数估计 (x,y,z_base,height,diameter)  │
│                              │                                    │
│                              ▼                                    │
│           TreeDetection 消息 (通过 topic relay 转发至邻居)         │
└──────────────────────────────┼────────────────────────────────────┘
                               │
                               ▼
┌─────────────────────────────────────────────────────────────────┐
│                    Drone B (tree_loc_node)                        │
│                                                                  │
│  接收邻居 TreeDetection ──────┐                                   │
│  自身 TreeDetection    ───────┤                                   │
│                               ▼                                   │
│  第0步: 距离过滤 — 两机间距 > drone_dist_threshold 则跳过          │
│  第1步: 收集自身 + 邻居树干列表                                    │
│  第2步: 三角形哈希匹配 + 投票验证 (HashReg 思想)                   │
│    2a: 构造非退化三角形 (KNN 三边排序 + 等腰/等边剔除)             │
│    2b: 自身三角形建哈希表，邻居三角形 125-voxel (5³) 粗糙匹配      │
│    2c: 投票验证 + 里程计先验一致性检查                             │
│    2d: 用最佳变换收集所有对应关系                                  │
│  第3步: 数量检查 — 共有树数量 < min_common_trees 则跳过            │
│  第4步: 加权 2D SVD 闭式解 — 求最优 (dx, dy, yaw)                 │
│  第5步: 质量检查 — 平移 + yaw + RMS残差阈值                        │
│  第6步: 发布 tree_pose_error + TreeRelativePose                   │
└─────────────────────────────────────────────────────────────────┘
                               │
                               ▼
┌─────────────────────────────────────────────────────────────────┐
│              Anchor Drone (tree_graph_node, 可选)                │
│                                                                  │
│  收集所有 UAV 的 TreeRelativePose ──→ 滑窗平均                    │
│              ──→ 增量添加 BetweenFactor 到 iSAM2                  │
│              ──→ iSAM2::update() — 仅重新线性化受影响 clique      │
│              ──→ calculateEstimate() 获取当前最优估计              │
│              ──→ 发布 TreeRelativePoseOptimized                   │
└─────────────────────────────────────────────────────────────────┘
```

### 阶段一：树干检测 (tree_detector_node)

**输入**: 原始 LiDAR 点云 `sensor_msgs/PointCloud2` + 里程计 `nav_msgs/Odometry`
**输出**: `TreeDetection` 消息（包含树干列表）

#### 步骤 0: 多帧积累（可选）

当 `accumulation_frame_count > 0` 时（默认 3），积累 N 帧点云拼接后再检测，提高森林场景树干检测鲁棒性。设为 0 时逐帧检测。

#### 步骤 1: 体素降采样

将点云通过 `pcl::VoxelGrid` 体素降采样，leaf size = `tree_voxel_size`（默认 0.15m），减少后续计算量。

#### 步骤 2: PatchWork++ 地面剔除

使用 PatchWork++ 算法进行地面分割，自适应地面估计，提取非地面点，保留树干等垂直结构。参数：
- `patchwork_sensor_height`: 传感器离地高度（默认 1.0m）
- `patchwork_max_range`: 最大探测距离（默认 15.0m）
- `patchwork_min_range`: 最小探测距离（默认 0.5m）

#### 步骤 3: 欧氏聚类

对非地面点使用 `pcl::EuclideanClusterExtraction`：
- 聚类容差：`tree_cluster_tolerance`（默认 0.3m）
- 点数范围：`[tree_min_cluster_size, tree_max_cluster_size]`（默认 [20, 5000]）
- 每个聚类候选为一棵树

#### 步骤 4: PCA 多条件筛选

对每个聚类计算协方差矩阵并进行特征值分解，获取特征值 λ₀ ≤ λ₁ ≤ λ₂ 和主成分方向 **v₂**：

| 条件 | 公式 | 默认阈值 | 用途 |
|------|------|---------|------|
| 线性度 | `(λ₂ - λ₁) / λ₂` | > 0.8 | 判断是否沿单一方向延伸（树干特征） |
| 平面度 | `(λ₁ - λ₀) / λ₂` | < 0.3 | 排除扁平物体（地面残片、树枝平面） |
| 高度延伸 | `z_max - z_min` | > 0.3m | 排除矮灌木 |
| 圆度 | `1 / (1 + CV)` | > 0.4 | 横截面接近圆形（CV = 变异系数） |

四个条件全部满足才判定为树干。

#### 步骤 5: 树干参数估计

**树干位置 — 质心 XY + 最低 Z**:

使用聚类质心的 XY 坐标和最低点的 Z 坐标作为树干位置：

```
tree.x = centroid[0]      # 质心 X
tree.y = centroid[1]      # 质心 Y
tree.z_base = z_min       # 最低点 Z
```

**其他参数**:
- `height`: 聚类点 Z 范围 (z_max - z_min)
- `diameter`: 以质心为中心计算点到中心的平均距离 × 2
- `confidence`: `linearity × 0.5 + roundness × 0.3 + pts_factor × 0.2`

---

### 阶段二：三角形哈希匹配 + 加权 SVD 配准 (tree_loc_node)

**输入**: 自身 `TreeDetection` + 邻居 `TreeDetection` + 两机里程计
**输出**: `tree_pose_error`（dx, dy, yaw）+ `TreeRelativePose` + `tri_descriptors`（三角形可视化）

#### 第 0 步: 距离过滤

两机间距 > `drone_dist_threshold`（默认 10.0m）则跳过本次计算。

#### 第 1 步: 收集树

检查自身和邻居各有至少 3 棵树，否则跳过。

#### 第 2a 步: 三角形构造

对每架无人机的树干列表，使用 PCL KdTreeFLANN 的 KNN（K=min(N,5)）找近邻，遍历所有三元组构成三角形。

对每个三角形的三边长 **a ≤ b ≤ c**（排序后），进行退化过滤：

| 条件 | 公式 | 默认阈值 | 来源 |
|------|------|---------|------|
| 边长范围 | a < min / c > max | [0.3m, 20m] | HashReg descriptor_min/max_len |
| 等腰剔除 | \|a - b\| < threshold | < 0.1m | HashReg descriptor_len_diff |
| 等腰剔除 | \|b - c\| < threshold | < 0.1m | 同上 |
| 等边剔除 | \|a - c\| < threshold | < 0.15m | HashReg 等边剔除 |

保留的三角形记录：三顶点坐标、重心（centroid）、三边长、对应树索引。

#### 第 2b 步: 三角形哈希匹配

**Hash key 生成**: 对三边长 a, b, c，以 **10cm 精度** 取整（`round(a×10)`），作为三元素 tuple key。

**自身三角形建哈希表**: key → 三角形索引列表。

**邻居三角形粗糙匹配** (HashReg `candidate_frames_selector` 思路):
- 对邻居的每个三角形，在 **±2 bin** 范围内搜索（即 5×5×5 = **125 voxel**，覆盖 ±20cm 的边长变化）
- 边长欧式距离 < `边长和 × rough_dis_ratio`（默认 0.05）
- 重心距离 < `max_centroid_dist`（默认 3.0m）
- 满足以上条件 → 记录为候选对

> 注意：10cm bin + ±2 搜索覆盖了 LiDAR 树检测在不同视角下的位置抖动（约 2-5cm 每棵树），足以处理视点依赖的质心漂移。

#### 第 2c 步: 投票验证 + 里程计先验检查

对每个候选三角形对 (HashReg `candidate_frames_verify` 思路):
1. 用 `triangleSolver`（2D SVD）求解变换 (t, R)
2. **里程计先验检查** — 用两机里程计差值作为先验 (t_odom)，过滤差异过大的候选：
   - 平移偏差 > 2.0m → 跳过
   - yaw 偏差 > 30° + max_yaw_deg → 跳过
3. 用 (t, R) 变换自身所有树的位置
4. 检查每棵变换后的树是否在邻居中找到对应树（距离 < `geom_verify_dist`，默认 0.3m）
5. 统计通过的树数量 = **vote**

选 vote 最多且 odom 先验距离最近的 (t_best, R_best) 作为最佳初始变换。如果 max_vote < 3，跳过本次计算。

#### 第 2d 步: 收集对应关系

用最佳变换 (t_best, R_best) 变换自身所有树，对每棵树找最近的未占用邻居树（距离 < `geom_verify_dist`），记录 **1对1** 对应关系。

#### 第 3 步: 数量检查

匹配树对数量 < `min_common_trees`（默认 3）则跳过。

#### 第 4 步: 加权 2D SVD 闭式解

设自身匹配树中心集合为 **P** = {p₁, p₂, ..., pₙ}，邻居对应集合为 **Q** = {q₁, q₂, ..., qₙ}，每个点为 2D (x, y)。

**权重**: 每对匹配的权重 = `√(confidence_self × confidence_neighbor)`，降低低置信度检测点的影响。几何意义：高线性度 + 高圆度 + 多点的树干在配准中权重更大。

**求解**:

1. **加权质心**: μ_P = Σ(wᵢ·pᵢ) / Σwᵢ, μ_Q = Σ(wᵢ·qᵢ) / Σwᵢ
2. **加权中心化**: P_w = √wᵢ · (pᵢ - μ_P), Q_w = √wᵢ · (qᵢ - μ_Q)
3. **H 矩阵**: H = P_w · Q_w^T  (2×2)
4. **SVD**: H = U · Σ · V^T
5. **旋转**: R = V · U^T（若 det(R) = -1，修正 V 的第二列取反）
6. **平移**: t = μ_Q - R · μ_P
7. **yaw**: θ = atan2(R₁₀, R₀₀)

此为**闭式解**，无需迭代，不会出现局部最优问题。

#### 第 5 步: 质量检查

三项检查全部通过才输出结果：

| 检查项 | 条件 | 默认阈值 |
|--------|------|---------|
| 平移量 | ‖t‖ | < 5.0m |
| yaw 角 | \|θ\| | < 20° |
| 加权 RMS 残差 | √(Σ wᵢ·\|qᵢ - (R·pᵢ + t)\|²) | < 0.5m |

#### 第 6 步: 结果发布

发布以下消息：
- `tree_pose_error` (`geometry_msgs/Vector3`): x=dx, y=dy, z=yaw
- `TreeRelativePose`（自定义 msg）: 包含 src/dst drone_id、dx、dy、yaw、协方差、加权 RMS 残差、匹配数
- `tree_relative_pose` (`geometry_msgs/PoseStamped`): 平移距离 + yaw 四元数
- `matched_tree_pairs` (`sensor_msgs/PointCloud2`): 匹配树对可视化（每对 2 个点）
- `tri_descriptors` (`visualization_msgs/MarkerArray`): 三角形描述子可视化（LINE_LIST），每 `tri_rebuild_interval` 帧刷新一次

---

### 阶段三：iSAM2 增量因子图优化 (tree_graph_node, 可选)

**输入**: 所有 UAV 的 `TreeRelativePose` 消息
**输出**: `TreeRelativePoseOptimized` 消息

仅在 anchor drone（默认为 UAV1）上启动，通过 `enable_graph_opt` 参数控制。与批量 Levenberg-Marquardt 每次重建整个因子图不同，iSAM2 持久化 Bayes tree，只对受影响变量增量更新。

#### 滑窗平均

对每对 UAV 的测量值维护一个滑动窗口（默认 `window_size=10`），窗口内取平均：
- 平移量：直接算术平均
- yaw 角：正弦/余弦平均后 `atan2` 求解（避免角度跳变）
- 协方差：对角元素平均

#### iSAM2 增量更新

1. **锚点先验因子** (`PriorFactor`): 首次固定 anchor drone 为 (0, 0, 0)，噪声极小 (1e-6)
2. **BetweenFactor**: 滑窗平均结果发生变化时才增量添加新的 BetweenFactor，噪声由协方差自适应缩放（`√cov × 10`）
3. **iSAM2::update()**: 仅重新线性化受影响 clique，O(1) 每因子
4. **calculateEstimate()**: 获取当前最优估计

```
因子图示例 (3 UAV):

  [uav1] ←── PriorFactor (0,0,0)
    │
    ├── BetweenFactor(uav1→uav2): (dx₁₂, dy₁₂, yaw₁₂)
    │
    └── BetweenFactor(uav1→uav3): (dx₁₃, dy₁₃, yaw₁₃)
         │
         └── BetweenFactor(uav2→uav3): (dx₂₃, dy₂₃, yaw₂₃)  (间接约束)
```

优化后通过 `pose_src.between(pose_dst)` 计算两两相对位姿，发布为 `TreeRelativePoseOptimized`。

#### iSAM2 参数

- `relinearizeThreshold`: 0.01（线性化点偏移超过此阈值时重新线性化）
- `relinearizeSkip`: 1（每个 update 检查）
- `factorization`: QR（比 Cholesky 更稳健，适合小图）

---

## 通信架构

### 仿真（单机多 UAV）

使用 `topic_tools/relay` 在 UAV 间转发 topic：

```
/uav2/lidar_slam/odom         → /uav1/neighbor_odom
/uav2/tree_detection          → /uav1/neighbor_tree_detection
/uav1/lidar_slam/odom         → /uav2/neighbor_odom
/uav1/tree_detection          → /uav2/neighbor_tree_detection
```

三机场景还在 anchor UAV1 上转发各 UAV 的 `tree_relative_pose_msg` 供 `tree_graph_node` 订阅。

### 真机（多主机）

使用 `rosmsg_tcp_bridge` 通过 UDP 广播/TCP 单播跨主机通信（`broadcast_ip`、`broadcast_port`、`next_drone_ip` 参数配置）。

---

## 配置参数 (`config/params.yaml`)

### 树干提取参数

| 参数 | 默认值 | 说明 |
|------|-------|------|
| `tree_voxel_size` | 0.15 | 体素降采样分辨率 (m) |
| `tree_cluster_tolerance` | 0.3 | 欧氏聚类最大间距 (m) |
| `tree_min_cluster_size` | 20 | 最小聚类点数 |
| `tree_max_cluster_size` | 5000 | 最大聚类点数 |
| `tree_linearity_threshold` | 0.8 | PCA 线性度阈值 |
| `tree_planarity_threshold` | 0.3 | PCA 平面度阈值（排除扁平物体） |
| `tree_min_height_spread` | 0.3 | 最小高度延伸 (m)，排除矮灌木 |
| `tree_min_roundness` | 0.4 | 横截面圆度阈值 |
| `patchwork_sensor_height` | 1.0 | PatchWork++ 传感器离地高度 (m) |
| `patchwork_max_range` | 15.0 | PatchWork++ 最大探测距离 (m) |
| `patchwork_min_range` | 0.5 | PatchWork++ 最小探测距离 (m) |
| `accumulation_frame_count` | 5 | 积累 N 帧点云后检测（0 = 逐帧检测） |

### 三角形匹配参数

| 参数 | 默认值 | 说明 |
|------|-------|------|
| `triangle_min_side` | 0.3 | 三角形最小边长 (m) |
| `triangle_max_side` | 20.0 | 三角形最大边长 (m) |
| `isosceles_threshold` | 0.1 | 等腰三角形判定阈值 (m) |
| `equilateral_threshold` | 0.15 | 等边三角形判定阈值 (m) |
| `rough_dis_ratio` | 0.05 | 粗糙匹配比例（边长和 × ratio） |
| `geom_verify_dist` | 0.3 | 几何验证距离 (m) |
| `max_centroid_dist` | 3.0 | 三角形重心距离过滤阈值 (m) |

### 树匹配参数

| 参数 | 默认值 | 说明 |
|------|-------|------|
| `min_common_trees` | 3 | 最少共有树数量 |
| `drone_dist_threshold` | 10.0 | 无人机距离过滤阈值 (m) |
| `max_translation` | 5.0 | 最大平移 (m)，质量检查 |
| `max_yaw_deg` | 20.0 | 最大 yaw (°)，质量检查 |
| `max_match_residual` | 0.5 | 最大加权 RMS 残差 (m) |

### 节点运行参数

| 参数 | 默认值 | 说明 |
|------|-------|------|
| `compute_frequency` | 5.0 | 检测与配准频率 (Hz) |
| `tri_rebuild_interval` | 5 | 三角形可视化刷新间隔 (帧) |

### 通信参数

| 参数 | 默认值 | 说明 |
|------|-------|------|
| `broadcast_ip` | 255.255.255.255 | UDP 广播 IP（多机仿真） |
| `broadcast_port` | 9001 | UDP 广播端口 |
| `next_drone_ip` | 127.0.0.1 | TCP 连接 IP（真机多主机） |

### 因子图优化参数

| 参数 | 默认值 | 说明 |
|------|-------|------|
| `anchor_drone_id` | 1 | 锚点无人机 ID（固定为参考系原点） |
| `optimize_freq` | 5.0 | 优化频率 (Hz) |
| `window_size` | 10 | 测量滑窗大小 |
| `uav_num` | 3 | 无人机总数 |

---

## ROS 接口

### 消息类型

**`Tree.msg`** — 单棵树干

| 字段 | 类型 | 说明 |
|------|------|------|
| `id` | `uint32` | 树的唯一 ID |
| `x, y` | `float64` | 树干质心 XY 坐标 (world frame) |
| `z_base` | `float64` | 树干底部 Z |
| `height` | `float64` | 树高 (m) |
| `diameter` | `float64` | 树干直径 (m) |
| `linearity` | `float64` | PCA 线性度 [0,1] |
| `planarity` | `float64` | PCA 平面度 [0,1] |
| `roundness` | `float64` | 横截面圆度 [0,1] |
| `confidence` | `float64` | 检测置信度 [0,1] |

**`TreeDetection.msg`** — 树检测结果

| 字段 | 类型 | 说明 |
|------|------|------|
| `header` | `Header` | 时间戳 + frame_id |
| `drone_id` | `uint32` | 无人机 ID |
| `trees` | `Tree[]` | 检测到的树干列表 |
| `odometry` | `nav_msgs/Odometry` | 检测时刻的里程计 |

**`TreeRelativePose.msg`** — 两机相对位姿测量

| 字段 | 类型 | 说明 |
|------|------|------|
| `header` | `Header` | 时间戳 + frame_id |
| `src_drone_id` | `uint32` | 源无人机 ID |
| `dst_drone_id` | `uint32` | 目标无人机 ID |
| `dx` | `float64` | 相对 X (m) |
| `dy` | `float64` | 相对 Y (m) |
| `yaw` | `float64` | 相对偏航角 (rad) |
| `covariance` | `float64[9]` | 3x3 协方差矩阵 (行优先) |
| `rms_residual` | `float64` | 匹配加权 RMS 残差 (m) |
| `num_matched_trees` | `int32` | 匹配树干数量 |

**`TreeRelativePoseOptimized.msg`** — iSAM2 优化后的相对位姿

| 字段 | 类型 | 说明 |
|------|------|------|
| `header` | `Header` | 时间戳 + frame_id |
| `src_drone_id` | `uint32` | 源无人机 ID |
| `dst_drone_id` | `uint32` | 目标无人机 ID |
| `dx` | `float64` | 优化后相对 X (m) |
| `dy` | `float64` | 优化后相对 Y (m) |
| `yaw` | `float64` | 优化后相对偏航角 (rad) |
| `rms_residual` | `float64` | 优化前匹配残差 (m) |
| `num_matched_trees` | `int32` | 匹配树干数量 |

### 订阅 Topics（tree_detector_node）

| Topic (remap from → to) | 类型 | 说明 |
|-------|------|------|
| `~lidar_cloud` → `/uav{ID}/pcl_render_node/cloud` | `sensor_msgs::PointCloud2` | 原始 LiDAR 点云 |
| `~odom` → `/uav{ID}/lidar_slam/odom` | `nav_msgs::Odometry` | 自身里程计 |

### 订阅 Topics（tree_loc_node）

| Topic (remap from → to) | 类型 | 说明 |
|-------|------|------|
| `~self_tree_detection` → `/uav{ID}/tree_detection` | `TreeDetection` | 自身树检测结果 |
| `~neighbor_tree_detection` → `/uav{ID}/neighbor_tree_detection` | `TreeDetection` | 邻居树检测结果 |
| `~self_odom` → `/uav{ID}/lidar_slam/odom` | `nav_msgs::Odometry` | 自身里程计 |
| `~neighbor_odom` → `/uav{ID}/neighbor_odom` | `nav_msgs::Odometry` | 邻居里程计 |

### 发布 Topics

| Topic | 类型 | 节点 | 说明 |
|-------|------|------|------|
| `/uav{ID}/tree_detection` | `TreeDetection` | tree_detector_node | 自身树检测结果 |
| `/uav{ID}/tree_cloud` | `sensor_msgs::PointCloud2` | tree_detector_node | 树干位置可视化 (z = z_base + height/2) |
| `/uav{ID}/tree_pose_error` | `geometry_msgs::Vector3` | tree_loc_node | 位姿误差 (dx, dy, yaw) |
| `/uav{ID}/tree_relative_pose` | `geometry_msgs::PoseStamped` | tree_loc_node | 相对位姿 (平移距离 + yaw 四元数) |
| `/uav{ID}/tree_relative_pose_msg` | `TreeRelativePose` | tree_loc_node | 相对位姿测量（供因子图优化使用） |
| `/uav{ID}/matched_tree_pairs` | `sensor_msgs::PointCloud2` | tree_loc_node | 匹配树对可视化 |
| `/uav{ID}/tri_descriptors` | `visualization_msgs::MarkerArray` | tree_loc_node | 三角形描述子可视化 |
| `/uav{ID}/tree_relative_pose_optimized` | `TreeRelativePoseOptimized` | tree_graph_node | iSAM2 优化后的相对位姿 |

---

## 注意事项

1. **适用场景**: 本模块专为森林林冠下环境设计（树干清晰可见）。在开阔建筑环境中，树干数量可能不足，导致匹配失败。

2. **树数量要求**: 至少需要 3 棵树才能构造三角形，从而执行配准。少于 3 棵时跳过计算。

3. **三角形匹配**: 借鉴 HashReg 算法思想，用三棵树构成的三角形三边长作为指纹进行哈希匹配（10cm bin + 125-voxel 搜索），比距离贪心匹配更鲁棒。等腰/等边三角形被剔除以保证独特性。

4. **里程计先验验证**: 每个候选三角形对求解变换后，先用两机里程计差值做先验检查（平移差 < 2m，yaw < 30°+max_yaw_deg），过滤与里程计矛盾的误匹配。

5. **加权 SVD**: 使用检测置信度的几何平均值作为权重，高置信度（高线性度 + 高圆度 + 多点）的树在配准中贡献更大。

6. **投票验证**: 每个候选三角形对求解变换后，用该变换验证所有树的对应关系，选一致性最好的结果。避免单一误匹配导致的错误输出。

7. **2D 输出**: `tree_pose_error` 只提供水平面内的位姿误差 (dx, dy, yaw)。z、roll、pitch 信息需要从里程计获取。

8. **iSAM2 因子图优化**: 仅 anchor drone（默认 UAV1）启动 `tree_graph_node`。通过 `enable_graph_opt:=true` 启用。使用增量 iSAM2 而非批量 LM，持久化 Bayes tree，每个新因子 O(1) 更新。优化后的位姿通过 `/uav{ID}/tree_relative_pose_optimized` 发布，可缓解多对匹配中的累积误差。

9. **依赖**: 因子图优化需要安装 GTSAM 库。未安装时 `tree_graph_node` 不会被编译，但树干检测和匹配功能不受影响。

10. **多帧积累**: `accumulation_frame_count` 默认 5，即积累 5 帧 LiDAR 点云后统一检测，提高森林场景树干检测鲁棒性。

---

## 评估测试

`Scripts/` 目录提供了一套自动化测试工具，用于评估 `tree_pose_error` 的精度和修正效果。

### 测试目标

回答两个核心问题：

| 方案 | 问题 | 对应脚本 |
|------|------|---------|
| B | `tree_pose_error` 估计的漂移量有多准？ | `evaluate_drift_estimation.py` |
| C | 应用修正公式后，邻居位置比修正前准了吗？ | `evaluate_correction_offline.py` |

### 测试原理

```
仿真 (里程计真值)
  → tree_detector_node 检测树干
  → tree_drift_injector 向 UAV 的树位置注入已知漂移 (dx,dy,yaw)
  → tree_loc_node 用树匹配估计两帧间的相对变换
  → 发布 tree_pose_error

方案B: tree_pose_error  vs  注入漂移的理论值        → 估计精度
方案C: 注入漂移 → 模拟 swarm_controller 修正公式
       → 对比 |corrected - gt| vs |raw - gt|        → 修正效果
```

修正公式（与 `swarm_controller::drift_correct_pos_nei` 一致）：

```
corrected = R(yaw) * pos + t
即:  corr_x =  cos(yaw)*raw_x - sin(yaw)*raw_y + dx
      corr_y =  sin(yaw)*raw_x + cos(yaw)*raw_y + dy
```

### 一键测试

```bash
cd Modules/ego_planner_swarm/drone_detect_lidar/Scripts/
./run_tree_drift_eval.sh [时长] [uav1_dx] [uav1_dy] [uav1_yaw_deg] [uav2_dx] [uav2_dy] [uav2_yaw_deg]
```

```bash
# 120秒测试，UAV1 作为 anchor (无漂移)，UAV2 注入 0.5m + 5° 漂移
./run_tree_drift_eval.sh 120 0 0 0 0.5 0.3 5.0
```

脚本自动完成：启动仿真 → 启动 tree_detector + drift_injector + tree_loc → 录制 rosbag → 方案B + 方案C 评估。

### 输出文件

测试结束后在 `drift_eval_YYYYMMDD_HHMMSS/` 生成：

| 文件 | 内容 |
|------|------|
| `drift_est_report.pdf` | 方案B 可视化报告 |
| `drift_est_summary.json` | 方案B 结构化指标 |
| `drift_est_per_frame.csv` | 方案B 逐帧明细 |
| `correction_offline_report.pdf` | 方案C 可视化报告 |
| `correction_offline_summary.json` | 方案C 结构化指标 |
| `correction_offline_per_frame.csv` | 方案C 逐帧明细 |

### 单独运行评估脚本

如果已有 rosbag，可跳过仿真直接评估：

```bash
# 方案B: 评估漂移估计精度
python3 evaluate_drift_estimation.py <日志目录>

# 方案C: 评估修正效果（需要 eval_config.json）
python3 evaluate_correction_offline.py <日志目录>
```

### 评估指标说明

方案C 核心指标：

| 指标 | 含义 |
|------|------|
| 修正前 RMS | 邻居漂移位置 vs 真值的均方根误差 |
| 修正后 RMS | 应用修正公式后的均方根误差 |
| 平均改进 | `(err_raw - err_corr) / err_raw * 100%`，正数 = 修正有效 |
| 正向比例 | 修正后比修正前更准的帧占比，>50% = 总体有效 |
| 判定 PASS/FAIL | 平均改进 > 0 且正向比例 > 50% |

### 快速查看 tree_pose_error 数值

```bash
# 在线模式 (ROS 运行中)
python3 analyze_tree_pose_error.py        # 统计 UAV1+UAV2，Ctrl+C 输出
python3 analyze_tree_pose_error.py 1      # 仅统计 UAV1

# 离线模式 (从日志)
python3 analyze_tree_pose_error.py --log <tree_loc_node日志文件>
```

### 调试话题

```bash
# 查看树检测结果
rostopic echo /uav1/tree_detection

# 查看 tree_pose_error
rostopic echo /uav1/tree_pose_error

# 查看 iSAM2 优化结果 (三机仿真)
rostopic echo /uav1/tree_relative_pose_optimized

# RViz 可视化
rviz -d $(rospack find drone_detect_lidar)/rviz/lidar_detect.rviz
```

---

## License

GPLv3
