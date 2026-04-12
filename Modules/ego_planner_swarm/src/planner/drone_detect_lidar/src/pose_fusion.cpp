#include "drone_detect_lidar/pose_fusion.h"
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <eigen_conversions/eigen_msg.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/filter.h>

namespace drone_detect_lidar {

// PCL 类型定义
typedef pcl::PointXYZINormal PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

PoseFusion::PoseFusion(int my_id, const ICPConfig& icp_config)
  : my_id_(my_id)
  , icp_config_(icp_config)
  , icp_wrapper_(icp_config)
  , feature_extractor_(FeatureExtractorConfig()) {
}

PoseFusion::~PoseFusion() {}

void PoseFusion::setICPConfig(const ICPConfig& config) {
  icp_config_ = config;
  icp_wrapper_.setConfig(config);
}

void PoseFusion::processNeighborFeature(const sensor_msgs::PointCloud2::ConstPtr& feature_cloud,
                                        const nav_msgs::Odometry::ConstPtr& neighbor_odom,
                                        int sender_id) {
  if (sender_id == my_id_) {
    ROS_WARN("[PoseFusion] Received own feature cloud, ignoring");
    return;
  }

  // 更新邻居状态
  NeighborState& state = neighbor_states_[sender_id];
  state.drone_id = sender_id;
  state.last_update_time = ros::Time::now();
  state.feature_timestamp = feature_cloud->header.stamp;

  // 解析里程计
  if (neighbor_odom) {
    tf::pointMsgToEigen(neighbor_odom->pose.pose.position, state.position);
    tf::quaternionMsgToEigen(neighbor_odom->pose.pose.orientation, state.orientation);
    tf::vectorMsgToEigen(neighbor_odom->twist.twist.linear, state.velocity);
  }

  // 解析特征云
  state.has_feature = fromROSMessage(*feature_cloud, state.feature_cloud);

  ROS_INFO("[PoseFusion] Received feature cloud from drone_%d: %zu points",
           sender_id, state.feature_cloud.size());
}

void PoseFusion::setLocalMap(const sensor_msgs::PointCloud2::ConstPtr& local_map) {
  my_local_map_ = local_map;
}

void PoseFusion::setMyOdometry(const nav_msgs::Odometry::ConstPtr& my_odom) {
  my_odom_ = my_odom;
}

PoseError PoseFusion::computePoseError(int neighbor_id) {
  PoseError error;
  error.valid = false;

  ROS_INFO("[PoseFusion] Computing pose error for neighbor %d", neighbor_id);

  auto it = neighbor_states_.find(neighbor_id);
  if (it == neighbor_states_.end()) {
    ROS_WARN("[PoseFusion] No state for neighbor %d", neighbor_id);
    return error;
  }

  const NeighborState& neighbor = it->second;

  // 检查数据有效性
  if (!neighbor.has_feature) {
    ROS_WARN("[PoseFusion] No feature cloud from neighbor %d", neighbor_id);
    return error;
  }

  if (!my_local_map_) {
    ROS_WARN("[PoseFusion] No local map available");
    return error;
  }

  if (!my_odom_) {
    ROS_WARN("[PoseFusion] No odometry available");
    return error;
  }

  ROS_INFO("[PoseFusion] Data OK, converting point clouds...");

  // 转换邻居特征云为 PCL (source: 邻居特征，在邻居局部系 LA)
  PointCloudT::Ptr neighbor_cloud(new PointCloudT());
  for (const auto& fp : neighbor.feature_cloud) {
    PointT pt;
    pt.x = fp.point.x();
    pt.y = fp.point.y();
    pt.z = fp.point.z();
    pt.normal_x = fp.normal.x();
    pt.normal_y = fp.normal.y();
    pt.normal_z = fp.normal.z();
    neighbor_cloud->push_back(pt);
  }

  // 转换自身局部地图为 PCL
  PointCloudT::Ptr my_map_raw(new PointCloudT());
  pcl::fromROSMsg(*my_local_map_, *my_map_raw);

  // 去除 NaN/Inf 点
  std::vector<int> nan_indices;
  pcl::removeNaNFromPointCloud(*my_map_raw, *my_map_raw, nan_indices);
  if (!nan_indices.empty()) {
    ROS_INFO("[PoseFusion] Removed %zu NaN points from local map", nan_indices.size());
  }

  // 体素降采样：将局部地图降至 ~0.3m 分辨率，减少 ICP 计算噪声
  PointCloudT::Ptr my_map_downsampled(new PointCloudT());
  pcl::VoxelGrid<PointT> voxel_grid;
  voxel_grid.setInputCloud(my_map_raw);
  voxel_grid.setLeafSize(0.3f, 0.3f, 0.3f);
  voxel_grid.filter(*my_map_downsampled);

  ROS_INFO("[PoseFusion] Neighbor cloud: %zu points, Local map: %zu -> %zu (downsampled)",
           neighbor_cloud->size(), my_map_raw->size(), my_map_downsampled->size());

  // 使用降采样后的地图
  PointCloudT::Ptr my_map_cloud = my_map_downsampled;

  if (neighbor_cloud->size() < 4 || my_map_cloud->size() < 4) {
    ROS_WARN("[PoseFusion] Point clouds too small for ICP");
    return error;
  }

  // 获取自身里程计位姿（用于误差比较）
  Eigen::Quaterniond my_orient;
  Eigen::Vector3d my_pos;
  tf::quaternionMsgToEigen(my_odom_->pose.pose.orientation, my_orient);
  tf::pointMsgToEigen(my_odom_->pose.pose.position, my_pos);

  // ============================================================
  // 两个点云都在 world 系。ICP 的目标：检验两套世界系点云是否
  // 自然对齐。如果 odometry 无漂移，ICP 应返回单位阵（误差≈0）。
  //
  // 方法：
  // 1. 裁剪邻居特征到重叠区域（减少 source 计算量）
  // 2. target 用完整自身局部地图（提供充足几何约束）
  // 3. 直接在世界系做 ICP（初值=单位阵）
  // 4. 多重质量检查过滤不可信结果
  // ============================================================

  // 步骤 1: 裁剪邻居特征到重叠区域

  // 以自身 position 为中心筛选邻居特征
  // 这样 overlap_source 是"在我附近的邻居特征点"
  double search_radius_sq = icp_config_.overlap_radius * icp_config_.overlap_radius;

  PointCloudT::Ptr overlap_source(new PointCloudT());
  for (const auto& pt : *neighbor_cloud) {
    double dx = pt.x - my_pos.x();
    double dy = pt.y - my_pos.y();
    double dz = pt.z - my_pos.z();
    if (dx*dx + dy*dy + dz*dz < search_radius_sq) {
      overlap_source->push_back(pt);
    }
  }

  // 步骤 2: target 用完整降采样地图，不裁剪（已在上面定义 my_map_cloud）
  // 提供充足的几何约束，避免 ICP 在稀疏点上匹配错误

  ROS_INFO("[PoseFusion] Overlap: source=%zu / %zu (centered on my pos), target=%zu (full map)",
           overlap_source->size(), neighbor_cloud->size(), my_map_cloud->size());

  if (overlap_source->size() < static_cast<size_t>(icp_config_.min_overlap_points)) {
    ROS_WARN("[PoseFusion] Not enough overlap source points: %zu (need %d), skip ICP",
             overlap_source->size(), icp_config_.min_overlap_points);
    return error;
  }

  // 步骤 3: 为目标点云计算法线（point-to-plane ICP 需要）
  if (icp_config_.use_point_to_plane) {
    pcl::NormalEstimation<PointT, PointT> normal_est;
    normal_est.setInputCloud(my_map_cloud);
    normal_est.setKSearch(10);
    normal_est.compute(*my_map_cloud);
  }

  // 步骤 4: ICP 对齐 overlap_source → my_map_cloud（都在 world 系，初值=单位阵）
  ICPConfig fine_config = icp_config_;
  float fine_corr_dist = std::max(0.5f, static_cast<float>(icp_config_.overlap_radius * 0.3f));
  fine_config.max_correspondence_dist = fine_corr_dist;
  ICPWrapper fine_icp(fine_config);

  ICPResult icp_result = fine_icp.align(
    overlap_source, my_map_cloud, Eigen::Matrix4f::Identity());

  if (!icp_result.converged || icp_result.fitness_score > icp_config_.fitness_score_thresh) {
    ROS_WARN("[PoseFusion] ICP failed for drone_%d (converged=%d, fitness=%.4f)",
             neighbor_id, icp_result.converged, icp_result.fitness_score);
    return error;
  }

  Eigen::Matrix4d T_icp = icp_result.transformation.cast<double>();
  Eigen::Vector3d icp_translation = T_icp.block<3, 1>(0, 3);
  Eigen::Matrix3d icp_rotation = T_icp.block<3, 3>(0, 0);

  // ============================================================
  // 步骤 5: 方案3 — 多重质量验证
  // ============================================================

  // 检查 1: 内点比例
  double inlier_ratio_thresh = 0.4;
  if (icp_result.inlier_ratio < inlier_ratio_thresh) {
    ROS_WARN("[PoseFusion] Low inlier ratio: %.2f (need >%.1f)",
             icp_result.inlier_ratio, inlier_ratio_thresh);
    return error;
  }

  // 检查 2: 平移量上限（odometry 无漂移时应接近 0）
  double max_translation = 2.0; // meters
  double trans_norm = icp_translation.norm();
  if (trans_norm > max_translation) {
    ROS_WARN("[PoseFusion] ICP translation too large: %.3f m (max %.1f)",
             trans_norm, max_translation);
    return error;
  }

  // 检查 3: 旋转量上限（yaw 变化 < 15°）
  Eigen::Vector3d icp_euler = quaternionToEuler(Eigen::Quaterniond(icp_rotation));
  double yaw_deg = std::abs(icp_euler(2)) * 180.0 / M_PI;
  double max_yaw_deg = 15.0;
  if (yaw_deg > max_yaw_deg) {
    ROS_WARN("[PoseFusion] ICP yaw too large: %.1f deg (max %.1f)",
             yaw_deg, max_yaw_deg);
    return error;
  }

  // 保存相对位姿
  RelativePose rel_pose;
  rel_pose.sender_id = neighbor_id;
  rel_pose.receiver_id = my_id_;
  rel_pose.timestamp = ros::Time::now();
  rel_pose.valid = true;

  Eigen::Vector3d corrected_neighbor_pos = neighbor.position + icp_translation;

  rel_pose.position = corrected_neighbor_pos;
  rel_pose.orientation = Eigen::Quaterniond(icp_rotation);
  rel_pose.confidence = computeConfidence(icp_result);

  last_relative_poses_[neighbor_id] = rel_pose;

  error.delta_position = icp_translation;
  error.delta_euler = icp_euler;
  error.corrected_position = corrected_neighbor_pos;
  error.fitness_score = icp_result.fitness_score;
  error.timestamp = ros::Time::now();
  error.frame_id = my_odom_->header.frame_id;
  error.valid = true;

  // 规范化欧拉角到 [-pi, pi]
  for (int i = 0; i < 3; ++i) {
    while (error.delta_euler(i) > M_PI) error.delta_euler(i) -= 2 * M_PI;
    while (error.delta_euler(i) < -M_PI) error.delta_euler(i) += 2 * M_PI;
  }

  last_pose_errors_[neighbor_id] = error;

  ROS_INFO("[PoseFusion] Computed pose error for drone_%d: pos_err=%.3f m, yaw_err=%.2f deg, inliers=%d(%.0f%%)",
           neighbor_id, trans_norm,
           icp_euler(2) * 180.0 / M_PI,
           icp_result.inlier_count,
           icp_result.inlier_ratio * 100.0);

  return error;
}

RelativePose PoseFusion::getRelativePose(int neighbor_id) const {
  auto it = last_relative_poses_.find(neighbor_id);
  if (it != last_relative_poses_.end()) {
    return it->second;
  }
  return RelativePose();
}

std::map<int, PoseError> PoseFusion::getAllPoseErrors() {
  std::map<int, PoseError> errors;

  for (const auto& pair : neighbor_states_) {
    if (pair.second.has_feature) {
      PoseError err = computePoseError(pair.first);
      if (err.valid) {
        errors[pair.first] = err;
      }
    }
  }

  return errors;
}

std::vector<int> PoseFusion::getNeighborIds() const {
  std::vector<int> ids;
  for (const auto& pair : neighbor_states_) {
    ids.push_back(pair.first);
  }
  return ids;
}

void PoseFusion::cleanupExpiredNeighbors(double timeout) {
  ros::Time now = ros::Time::now();
  auto it = neighbor_states_.begin();

  while (it != neighbor_states_.end()) {
    if ((now - it->second.last_update_time).toSec() > timeout) {
      ROS_INFO("[PoseFusion] Removing expired neighbor %d", it->first);
      it = neighbor_states_.erase(it);
    } else {
      ++it;
    }
  }
}

bool PoseFusion::fromROSMessage(const sensor_msgs::PointCloud2& msg,
                                 std::vector<FeaturePoint>& features) {
  PointCloudT cloud;
  pcl::fromROSMsg(msg, cloud);

  features.clear();
  features.reserve(cloud.size());

  for (const auto& pt : cloud) {
    FeaturePoint fp;
    fp.point = pt.getVector3fMap();
    fp.normal = pt.getNormalVector3fMap();
    fp.curvature = pt.curvature;

    // 从 intensity 或自定义字段解析特征类型
    int type_int = static_cast<int>(pt.intensity);
    if (type_int == 0) {
      fp.type = FeatureType::CORNER;
    } else if (type_int == 1) {
      fp.type = FeatureType::PLANAR;
    } else {
      fp.type = FeatureType::INVALID;
    }

    features.push_back(fp);
  }

  return !features.empty();
}

Eigen::Matrix4d PoseFusion::motionCompensation(const Eigen::Matrix4d& pose,
                                                const Eigen::Vector3d& velocity,
                                                double dt) {
  Eigen::Matrix4d motion_comp = Eigen::Matrix4d::Identity();
  motion_comp.block<3, 1>(0, 3) = velocity * dt;
  return pose * motion_comp;
}

Eigen::Matrix4d PoseFusion::computeWorldPose(
    const Eigen::Matrix4d& T_W_A,
    const Eigen::Matrix4d& T_A_LA,
    const Eigen::Matrix4d& T_LA_LB,
    const Eigen::Matrix4d& T_LB_B) {
  // T_W_B = T_W_A * T_A_LA * T_LA_LB * T_LB_B
  // 假设 T_A_LA 和 T_LB_B 都是单位矩阵
  return T_W_A * T_LA_LB;
}

double PoseFusion::computeConfidence(const ICPResult& icp_result) {
  double confidence = 1.0;

  // 基于收敛性
  if (!icp_result.converged) confidence *= 0.0;

  // 基于适配度分数 (越低越好)
  double fitness_factor = std::max(0.0, 1.0 - icp_result.fitness_score / icp_config_.fitness_score_thresh);
  confidence *= fitness_factor;

  // 基于内点比例 (越高越好)
  confidence *= icp_result.inlier_ratio;

  // 基于迭代次数 (越少越好，说明初始值好)
  double iter_factor = std::max(0.0, 1.0 - static_cast<double>(icp_result.iteration_num) /
                                         static_cast<double>(icp_config_.max_iterations));
  confidence *= (0.5 + 0.5 * iter_factor);

  return std::max(0.0, std::min(1.0, confidence));
}

Eigen::Vector3d PoseFusion::quaternionToEuler(const Eigen::Quaterniond& q) {
  Eigen::Vector3d euler;
  euler(0) = std::atan2(2 * (q.w() * q.x() + q.y() * q.z()),
                        1 - 2 * (q.x() * q.x() + q.y() * q.y()));  // roll
  euler(1) = std::asin(2 * (q.w() * q.y() - q.z() * q.x()));       // pitch
  euler(2) = std::atan2(2 * (q.w() * q.z() + q.x() * q.y()),
                        1 - 2 * (q.y() * q.y() + q.z() * q.z()));  // yaw
  return euler;
}

} // namespace drone_detect_lidar
