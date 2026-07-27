#include "drone_detect_lidar/tree_detector.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/conversions.h>
#include <pcl/PCLPointField.h>
#include <pcl/common/centroid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cmath>

namespace drone_detect_lidar {

TreeDetector::TreeDetector(const TreeDetectorConfig& config)
  : config_(config), last_processing_time_(0), last_tree_count_(0) {

  // Initialize PatchWork++ parameters
  patchwork_params_.sensor_height = config_.patchwork_sensor_height;
  patchwork_params_.max_range = config_.patchwork_max_range;
  patchwork_params_.min_range = config_.patchwork_min_range;
  patchwork_params_.verbose = false;

  patchworkpp_ = std::make_unique<patchwork::PatchWorkpp>(patchwork_params_);
}

TreeDetector::~TreeDetector() {}

void TreeDetector::setConfig(const TreeDetectorConfig& config) {
  config_ = config;
}

bool TreeDetector::detect(const PointCloudPtr& cloud_in, std::vector<TreeInfo>& trees_out) {
  ros::WallTime start = ros::WallTime::now();
  trees_out.clear();

  if (!cloud_in || cloud_in->empty()) {
    ROS_WARN("[TreeDetector] Empty input cloud");
    return false;
  }

  // 步骤1: 体素降采样
  PointCloudPtr downsampled(new PointCloudT());
  pcl::VoxelGrid<PointT> voxel;
  voxel.setInputCloud(cloud_in);
  voxel.setLeafSize(config_.tree_voxel_size, config_.tree_voxel_size, config_.tree_voxel_size);
  voxel.filter(*downsampled);

  // 步骤2: PatchWork++ 地面剔除
  PointCloudPtr no_ground = removeGroundPatchWork(downsampled);
  if (no_ground->size() < static_cast<size_t>(config_.tree_min_cluster_size)) {
    ROS_WARN_THROTTLE(2.0, "[TreeDetector] Too few points after ground removal: %zu", no_ground->size());
    last_processing_time_ = (ros::WallTime::now() - start).toSec();
    return false;
  }

  // 步骤4: 欧氏聚类
  std::vector<PointCloudPtr> clusters = clusterPoints(no_ground);

  // 步骤5-6: 连续性/几何约束筛选 + 树干参数估计
  uint32_t tree_id = 0;
  int rejected_low_linearity = 0;
  int rejected_short = 0;
  int rejected_flat = 0;

  for (const auto& cluster : clusters) {
    TreeInfo tree;
    tree.id = tree_id;
    if (computeTreeParameters(cluster, tree)) {
      trees_out.push_back(tree);
      tree_id++;
    } else {
      // 统计被拒原因
      if (cluster->size() >= 10) {
        // 参数已计算过，检查是哪个条件失败
        if (tree.height < config_.tree_min_height_spread) rejected_short++;
        else if (tree.planarity > config_.tree_planarity_threshold) rejected_flat++;
      } else {
        rejected_low_linearity++;
      }
    }
  }

  last_tree_count_ = trees_out.size();
  last_processing_time_ = (ros::WallTime::now() - start).toSec();
  ROS_INFO("[TreeDetector] Detected %zu trees in %.3f s (input=%zu, clusters=%zu"
           ", rejected: low_lin=%d, flat=%d, short=%d)",
           trees_out.size(), last_processing_time_,
           cloud_in->size(), clusters.size(),
           rejected_low_linearity, rejected_flat, rejected_short);

  return !trees_out.empty();
}

bool TreeDetector::detectFromROS(const sensor_msgs::PointCloud2::ConstPtr& msg,
                                  std::vector<TreeInfo>& trees_out) {
  PointCloudPtr cloud(new PointCloudT());
  pcl::fromROSMsg(*msg, *cloud);
  return detect(cloud, trees_out);
}

// PatchWork++ 地面剔除
TreeDetector::PointCloudPtr TreeDetector::removeGroundPatchWork(const PointCloudPtr& cloud) {
  patchworkpp_->estimateGround<PointT>(cloud);

  // Extract non-ground points
  Eigen::MatrixX3f nonground = patchworkpp_->getNonground();

  PointCloudPtr out(new PointCloudT());
  out->resize(nonground.rows());
  for (int i = 0; i < nonground.rows(); i++) {
    out->points[i].x = nonground(i, 0);
    out->points[i].y = nonground(i, 1);
    out->points[i].z = nonground(i, 2);
  }

  return out;
}

// 欧氏聚类
std::vector<TreeDetector::PointCloudPtr> TreeDetector::clusterPoints(const PointCloudPtr& cloud) {
  std::vector<PointCloudPtr> clusters;

  pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
  tree->setInputCloud(cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance(config_.tree_cluster_tolerance);
  ec.setMinClusterSize(config_.tree_min_cluster_size);
  ec.setMaxClusterSize(config_.tree_max_cluster_size);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(cluster_indices);

  for (size_t i = 0; i < cluster_indices.size(); i++) {
    PointCloudPtr cluster(new PointCloudT());
    for (size_t j = 0; j < cluster_indices[i].indices.size(); j++) {
      int idx = cluster_indices[i].indices[j];
      cluster->push_back((*cloud)[idx]);
    }
    clusters.push_back(cluster);
  }

  return clusters;
}

// 对单个聚类计算树干参数（多条件筛选）
bool TreeDetector::computeTreeParameters(const PointCloudPtr& cluster, TreeInfo& tree) {
  int n = cluster->size();
  if (n < 10) return false;

  // 计算协方差矩阵
  Eigen::Vector4d centroid;
  pcl::compute3DCentroid(*cluster, centroid);

  Eigen::Matrix3d cov_matrix = Eigen::Matrix3d::Zero();
  for (const auto& pt : *cluster) {
    Eigen::Vector3d diff(pt.x - centroid[0], pt.y - centroid[1], pt.z - centroid[2]);
    cov_matrix += diff * diff.transpose();
  }
  cov_matrix /= (n - 1);

  // 特征值分解
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov_matrix);
  Eigen::Vector3d eigenvalues = solver.eigenvalues();  // λ₀ ≤ λ₁ ≤ λ₂
  Eigen::Vector3d principal_dir = solver.eigenvectors().col(2);  // 主成分方向

  double lambda_max = eigenvalues[2];
  double lambda_mid = eigenvalues[1];
  double lambda_min = eigenvalues[0];
  if (lambda_max < 1e-6) return false;

  // 线性度: (λ₂ - λ₁) / λ₂ — 树干沿一个方向延伸
  tree.linearity = (lambda_max - lambda_mid) / lambda_max;
  if (tree.linearity < config_.tree_linearity_threshold) return false;

  // 平面度: (λ₁ - λ₀) / λ₂ — 排除扁平物体（地面残片、树枝平面）
  tree.planarity = (lambda_mid - lambda_min) / lambda_max;
  if (tree.planarity > config_.tree_planarity_threshold) return false;

  // 高度延伸: Z 方向 spread — 排除矮灌木
  double z_min = centroid[2], z_max = centroid[2];
  for (const auto& pt : *cluster) {
    if (pt.z < z_min) z_min = pt.z;
    if (pt.z > z_max) z_max = pt.z;
  }
  tree.height = z_max - z_min;
  if (tree.height < config_.tree_min_height_spread) return false;

  // 树干位置: 使用质心 XY + 最低 Z（PatchWork++ 不提供全局地面方程）
  tree.x = centroid[0];
  tree.y = centroid[1];
  tree.z_base = z_min;

  // 直径: 在垂直于主成分轴的平面上计算截面圆度
  // 投影到 XY 平面计算（近似，适用于近似垂直的树干）
  double avg_radius = 0, radius_var = 0;
  for (const auto& pt : *cluster) {
    double dx = pt.x - tree.x;
    double dy = pt.y - tree.y;
    double r = std::sqrt(dx * dx + dy * dy);
    avg_radius += r;
  }
  avg_radius /= n;

  // 计算半径方差用于圆度评估
  for (const auto& pt : *cluster) {
    double dx = pt.x - tree.x;
    double dy = pt.y - tree.y;
    double r = std::sqrt(dx * dx + dy * dy);
    radius_var += (r - avg_radius) * (r - avg_radius);
  }
  radius_var /= n;

  tree.diameter = avg_radius * 2.0;

  // 圆度: 1 / (1 + radius_var / avg_radius^2)
  // 圆度越高说明横截面越接近圆形（树干特征），越低说明形状不规则（灌木/树枝）
  if (avg_radius > 0.01) {
    double cv = std::sqrt(radius_var) / avg_radius;  // 变异系数
    tree.roundness = 1.0 / (1.0 + cv);
  } else {
    tree.roundness = 0;
  }

  // 圆度检查
  if (tree.roundness < config_.tree_min_roundness) return false;

  // 置信度: 综合线性度、圆度、点数
  double pts_factor = std::min(1.0, static_cast<double>(n) / 100.0);
  tree.confidence = tree.linearity * 0.5 + tree.roundness * 0.3 + pts_factor * 0.2;

  return true;
}

} // namespace drone_detect_lidar
