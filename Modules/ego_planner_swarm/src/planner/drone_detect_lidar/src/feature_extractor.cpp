#include "drone_detect_lidar/feature_extractor.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/io/pcd_io.h>
#include <chrono>

namespace drone_detect_lidar {

FeatureExtractor::FeatureExtractor(const FeatureExtractorConfig& config)
  : config_(config)
  , filtered_cloud_(new PointCloudT())
  , last_processing_time_(0.0)
  , last_feature_count_(0) {

  // 初始化搜索方法
  search_method_.reset(new pcl::search::KdTree<PointT>());

  // 配置体素滤波器
  voxel_grid_.setLeafSize(
    static_cast<float>(config_.voxel_size),
    static_cast<float>(config_.voxel_size),
    static_cast<float>(config_.voxel_size));

  // 配置法向量估计
  normal_estimator_.setSearchMethod(search_method_);
  normal_estimator_.setKSearch(config_.search_radius * 3);
}

FeatureExtractor::~FeatureExtractor() {}

void FeatureExtractor::setConfig(const FeatureExtractorConfig& config) {
  config_ = config;
  voxel_grid_.setLeafSize(
    static_cast<float>(config_.voxel_size),
    static_cast<float>(config_.voxel_size),
    static_cast<float>(config_.voxel_size));
  normal_estimator_.setKSearch(config_.search_radius * 3);
}

bool FeatureExtractor::extract(const PointCloudPtr& cloud_in,
                               std::vector<FeaturePoint>& features_out) {
  auto start_time = std::chrono::high_resolution_clock::now();

  features_out.clear();

  if (cloud_in->empty() || cloud_in->size() < static_cast<size_t>(config_.search_radius + 1)) {
    ROS_WARN("[FeatureExtractor] Input cloud too small: %zu points", cloud_in->size());
    return false;
  }

  // 1. 体素降采样
  PointCloudPtr downsampled_cloud = voxelFilter(cloud_in);

  if (downsampled_cloud->size() < static_cast<size_t>(config_.search_radius + 1)) {
    ROS_WARN("[FeatureExtractor] Downsampled cloud too small: %zu points", downsampled_cloud->size());
    return false;
  }

  // 2. 估计法向量
  estimateNormals(downsampled_cloud);

  // 3. 计算曲率
  std::vector<float> curvatures;
  computeCurvature(downsampled_cloud, curvatures);

  // 4. 提取角点
  extractCornerPoints(downsampled_cloud, curvatures, features_out);

  // 5. 提取平面点
  extractPlanarPoints(downsampled_cloud, features_out);

  // 6. 特征点筛选
  filterFeatures(features_out);

  // 7. 限制最大特征点数
  if (static_cast<int>(features_out.size()) > config_.max_feature_points) {
    // 按曲率排序，保留最显著的点
    std::sort(features_out.begin(), features_out.end(),
      [](const FeaturePoint& a, const FeaturePoint& b) {
        // 角点优先，然后按曲率排序
        if (a.type != b.type) {
          return a.type == FeatureType::CORNER;
        }
        return a.curvature > b.curvature;
      });
    features_out.resize(config_.max_feature_points);
  }

  auto end_time = std::chrono::high_resolution_clock::now();
  last_processing_time_ = std::chrono::duration<double, std::milli>(end_time - start_time).count();
  last_feature_count_ = static_cast<int>(features_out.size());

  ROS_DEBUG("[FeatureExtractor] Extracted %d features in %.2f ms",
            last_feature_count_, last_processing_time_);

  return !features_out.empty();
}

bool FeatureExtractor::extractFromROS(const sensor_msgs::PointCloud2::ConstPtr& msg,
                                       std::vector<FeaturePoint>& features_out) {
  PointCloudT cloud;
  pcl::fromROSMsg(*msg, cloud);
  return extract(cloud.makeShared(), features_out);
}

void FeatureExtractor::toROSMessage(const std::vector<FeaturePoint>& features,
                                    sensor_msgs::PointCloud2& cloud_out) {
  PointCloudT cloud;
  cloud.resize(features.size());
  cloud.is_dense = true;

  for (size_t i = 0; i < features.size(); ++i) {
    cloud[i].x = features[i].point.x();
    cloud[i].y = features[i].point.y();
    cloud[i].z = features[i].point.z();
    cloud[i].normal_x = features[i].normal.x();
    cloud[i].normal_y = features[i].normal.y();
    cloud[i].normal_z = features[i].normal.z();
    cloud[i].curvature = features[i].curvature;
    cloud[i].intensity = static_cast<float>(static_cast<int>(features[i].type));
  }

  pcl::toROSMsg(cloud, cloud_out);
}

FeatureExtractor::PointCloudPtr FeatureExtractor::voxelFilter(const PointCloudPtr& cloud) {
  PointCloudPtr filtered(new PointCloudT());
  voxel_grid_.setInputCloud(cloud);
  voxel_grid_.filter(*filtered);
  return filtered;
}

void FeatureExtractor::estimateNormals(const PointCloudPtr& cloud) {
  normal_estimator_.setInputCloud(cloud);
  normal_estimator_.setSearchSurface(cloud);
  normal_estimator_.compute(*cloud);

  // 统一法向方向 (朝外)
  for (size_t i = 0; i < cloud->size(); ++i) {
    if (cloud->at(i).z < 0) {
      cloud->at(i).normal_x *= -1.0f;
      cloud->at(i).normal_y *= -1.0f;
      cloud->at(i).normal_z *= -1.0f;
    }
  }
}

void FeatureExtractor::computeCurvature(const PointCloudPtr& cloud,
                                        std::vector<float>& curvatures) {
  curvatures.resize(cloud->size());

  pcl::KdTreeFLANN<PointT> kdtree;
  kdtree.setInputCloud(cloud);

  std::vector<int> k_indices;
  std::vector<float> k_sqr_distances;

  for (size_t i = 0; i < cloud->size(); ++i) {
    int k = kdtree.radiusSearch(i, config_.voxel_size * 2, k_indices, k_sqr_distances);

    if (k < 3) {
      curvatures[i] = 0.0f;
      continue;
    }

    // 计算邻域点的法向变化
    Eigen::Vector3f normal_ref = cloud->at(i).getNormalVector3fMap();
    float normal_variance = 0.0f;

    for (int j = 0; j < k; ++j) {
      Eigen::Vector3f normal_nb = cloud->at(k_indices[j]).getNormalVector3fMap();
      float dot = normal_ref.dot(normal_nb);
      normal_variance += (1.0f - std::abs(dot));
    }

    curvatures[i] = normal_variance / static_cast<float>(k);
  }
}

void FeatureExtractor::extractCornerPoints(const PointCloudPtr& cloud,
                                           const std::vector<float>& curvatures,
                                           std::vector<FeaturePoint>& features) {
  for (size_t i = 0; i < cloud->size(); ++i) {
    if (curvatures[i] > config_.corner_threshold) {
      FeaturePoint fp;
      fp.point = cloud->at(i).getVector3fMap();
      fp.normal = cloud->at(i).getNormalVector3fMap();
      fp.curvature = curvatures[i];
      fp.type = FeatureType::CORNER;
      fp.ring_id = getRingId(cloud->at(i), cloud);
      features.push_back(fp);
    }
  }
}

void FeatureExtractor::extractPlanarPoints(const PointCloudPtr& cloud,
                                           std::vector<FeaturePoint>& features) {
  for (size_t i = 0; i < cloud->size(); ++i) {
    // 平面点：曲率低且法向稳定
    if (cloud->at(i).curvature < config_.planar_threshold) {
      // 检查法向是否接近主轴 (墙面/地面特征)
      Eigen::Vector3f n = cloud->at(i).getNormalVector3fMap();
      float nz = std::abs(n.z());

      if (nz > 0.85 || nz < 0.15) {  // 接近垂直或水平
        FeaturePoint fp;
        fp.point = cloud->at(i).getVector3fMap();
        fp.normal = n;
        fp.curvature = cloud->at(i).curvature;
        fp.type = FeatureType::PLANAR;
        fp.ring_id = getRingId(cloud->at(i), cloud);
        features.push_back(fp);
      }
    }
  }
}

void FeatureExtractor::filterFeatures(std::vector<FeaturePoint>& features) {
  if (features.size() < 2) return;

  std::vector<FeaturePoint> filtered;
  filtered.reserve(features.size());

  // 简单距离过滤
  for (const auto& fp : features) {
    bool too_close = false;
    for (const auto& existing : filtered) {
      float dist = (fp.point - existing.point).norm();
      if (dist < config_.min_distance) {
        too_close = true;
        break;
      }
    }
    if (!too_close) {
      filtered.push_back(fp);
    }
  }

  features = filtered;
}

uint8_t FeatureExtractor::getRingId(const PointT& pt, const PointCloudPtr& original_cloud) {
  // 对于 LiDAR 点云，ring 信息通常在 intensity 或特定字段中
  // 这里简化处理，根据 z 坐标估算
  if (!config_.enable_ring_filter) return 0;

  // 简化：根据垂直角度估算 ring
  float vertical_angle = std::atan2(pt.z, std::sqrt(pt.x * pt.x + pt.y * pt.y)) * 180.0f / M_PI;
  int ring = static_cast<int>((vertical_angle + 15.0f) / 30.0f * config_.scan_ring_num);
  ring = std::max(0, std::min(ring, config_.scan_ring_num - 1));
  return static_cast<uint8_t>(ring);
}

} // namespace drone_detect_lidar
