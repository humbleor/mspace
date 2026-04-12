#include "drone_detect_lidar/icp_wrapper.h"
#include <pcl/common/centroid.h>
#include <chrono>

namespace drone_detect_lidar {

ICPWrapper::ICPWrapper(const ICPConfig& config)
  : config_(config)
  , kdtree_(new pcl::KdTreeFLANN<PointT>()) {

  // 配置 point-to-point ICP
  icp_point_to_point_.setMaxCorrespondenceDistance(config_.max_correspondence_dist);
  icp_point_to_point_.setMaximumIterations(config_.max_iterations);
  icp_point_to_point_.setTransformationEpsilon(config_.transformation_epsilon);
  icp_point_to_point_.setEuclideanFitnessEpsilon(config_.euclidean_fitness_epsilon);

  // 配置 point-to-plane ICP
  icp_point_to_plane_.setMaxCorrespondenceDistance(config_.max_correspondence_dist);
  icp_point_to_plane_.setMaximumIterations(config_.max_iterations);
  icp_point_to_plane_.setTransformationEpsilon(config_.transformation_epsilon);
  icp_point_to_plane_.setEuclideanFitnessEpsilon(config_.euclidean_fitness_epsilon);
}

ICPWrapper::~ICPWrapper() {}

void ICPWrapper::setConfig(const ICPConfig& config) {
  config_ = config;

  icp_point_to_point_.setMaxCorrespondenceDistance(config_.max_correspondence_dist);
  icp_point_to_point_.setMaximumIterations(config_.max_iterations);
  icp_point_to_point_.setTransformationEpsilon(config_.transformation_epsilon);

  icp_point_to_plane_.setMaxCorrespondenceDistance(config_.max_correspondence_dist);
  icp_point_to_plane_.setMaximumIterations(config_.max_iterations);
  icp_point_to_plane_.setTransformationEpsilon(config_.transformation_epsilon);
}

ICPWrapper::PointCloudPtr ICPWrapper::toPCLCloud(const std::vector<FeaturePoint>& features) {
  PointCloudPtr cloud(new PointCloudT());
  cloud->resize(features.size());

  for (size_t i = 0; i < features.size(); ++i) {
    cloud->at(i).x = features[i].point.x();
    cloud->at(i).y = features[i].point.y();
    cloud->at(i).z = features[i].point.z();
    cloud->at(i).normal_x = features[i].normal.x();
    cloud->at(i).normal_y = features[i].normal.y();
    cloud->at(i).normal_z = features[i].normal.z();
    cloud->at(i).curvature = features[i].curvature;
  }

  return cloud;
}

bool ICPWrapper::isValidResult() const {
  if (!last_result_.converged) return false;
  if (last_result_.fitness_score > config_.fitness_score_thresh) return false;
  if (last_result_.inlier_count < config_.min_inlier_count) return false;
  return true;
}

void ICPWrapper::setLastTransformation(const Eigen::Matrix4f& trans) {
  last_result_.transformation = trans;
}

ICPResult ICPWrapper::align(const PointCloudPtr& source_cloud,
                            const PointCloudPtr& target_cloud,
                            const Eigen::Matrix4f& initial_guess) {
  ICPResult result;

  ROS_INFO("[ICPWrapper] Starting ICP: source=%zu pts, target=%zu pts, use_plane=%d",
           source_cloud->size(), target_cloud->size(), config_.use_point_to_plane);

  if (source_cloud->empty() || target_cloud->empty()) {
    ROS_WARN("[ICPWrapper] Empty input clouds");
    return result;
  }

  if (source_cloud->size() < 4 || target_cloud->size() < 4) {
    ROS_WARN("[ICPWrapper] Input clouds too small for ICP");
    return result;
  }

  auto start_time = std::chrono::high_resolution_clock::now();

  // Point-to-Plane ICP 准备：确保目标点云有法线
  bool use_point_to_plane = config_.use_point_to_plane;

  if (use_point_to_plane) {
    // 检查目标点云是否有有效法线
    bool has_normals = true;
    for (size_t i = 0; i < std::min(target_cloud->size(), size_t(10)); ++i) {
      const auto& pt = target_cloud->at(i);
      if (std::isnan(pt.normal_x) || std::isnan(pt.normal_y) || std::isnan(pt.normal_z)) {
        has_normals = false;
        break;
      }
    }

    if (!has_normals) {
      ROS_WARN("[ICPWrapper] Target cloud missing normals, falling back to point-to-point");
      use_point_to_plane = false;
      icp_point_to_point_.setInputSource(source_cloud);
      icp_point_to_point_.setInputTarget(target_cloud);
    } else {
      ROS_INFO("[ICPWrapper] Target cloud has valid normals, using point-to-plane");
      icp_point_to_plane_.setInputSource(source_cloud);
      icp_point_to_plane_.setInputTarget(target_cloud);
    }
  } else {
    icp_point_to_point_.setInputSource(source_cloud);
    icp_point_to_point_.setInputTarget(target_cloud);
  }

  ROS_INFO("[ICPWrapper] Running ICP alignment...");

  // 执行配准
  PointCloudT output;

  if (config_.use_hierarchical && use_point_to_plane) {
    // 分层配准
    result = hierarchicalAlign(source_cloud, target_cloud, initial_guess);
  } else {
    if (use_point_to_plane) {
      // Point-to-plane ICP
      ROS_INFO("[ICPWrapper] Using point-to-plane ICP");
      icp_point_to_plane_.align(output, initial_guess);
      result.converged = icp_point_to_plane_.hasConverged();
      // 注意: PCL 不公开暴露实际迭代次数，此处返回配置的最大迭代次数
      result.iteration_num = icp_point_to_plane_.getMaximumIterations();
      result.transformation = icp_point_to_plane_.getFinalTransformation();
    } else {
      // Point-to-point ICP
      ROS_INFO("[ICPWrapper] Using point-to-point ICP");
      icp_point_to_point_.align(output, initial_guess);
      result.converged = icp_point_to_point_.hasConverged();
      // 注意: PCL 不公开暴露实际迭代次数，此处返回配置的最大迭代次数
      result.iteration_num = icp_point_to_point_.getMaximumIterations();
      result.transformation = icp_point_to_point_.getFinalTransformation();
    }
  }

  ROS_INFO("[ICPWrapper] ICP done: converged=%d, iterations=%d",
           result.converged, result.iteration_num);

  // 计算评估指标
  result.fitness_score = computeFitnessScore(source_cloud, target_cloud, result.transformation);
  result.rmse = computeRMSE(source_cloud, target_cloud, result.transformation);
  result.inlier_count = countInliers(source_cloud, target_cloud, result.transformation,
                                     config_.max_correspondence_dist * 0.5);
  result.inlier_ratio = static_cast<double>(result.inlier_count) /
                        static_cast<double>(source_cloud->size());

  // 检查配准质量
  if (!result.converged) {
    ROS_WARN("[ICPWrapper] ICP did not converge");
  }
  if (result.fitness_score > config_.fitness_score_thresh) {
    ROS_WARN("[ICPWrapper] Fitness score too high: %.4f", result.fitness_score);
  }
  if (result.inlier_ratio < (1.0 - config_.outlier_ratio_thresh)) {
    ROS_WARN("[ICPWrapper] Inlier ratio too low: %.2f", result.inlier_ratio);
  }

  auto end_time = std::chrono::high_resolution_clock::now();
  double elapsed_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();

  ROS_DEBUG("[ICPWrapper] ICP completed in %.2f ms: converged=%s, fitness=%.4f, inliers=%d",
            elapsed_ms, result.converged ? "yes" : "no", result.fitness_score, result.inlier_count);

  last_result_ = result;
  return result;
}

ICPResult ICPWrapper::hierarchicalAlign(const PointCloudPtr& source,
                                        const PointCloudPtr& target,
                                        const Eigen::Matrix4f& initial_guess) {
  ICPResult result;

  // 粗配准阶段
  int coarse_iter = config_.coarse_iterations;
  double coarse_corr_dist = config_.max_correspondence_dist * 2.0;

  if (config_.use_point_to_plane) {
    icp_point_to_plane_.setMaximumIterations(coarse_iter);
    icp_point_to_plane_.setMaxCorrespondenceDistance(coarse_corr_dist);
  } else {
    icp_point_to_point_.setMaximumIterations(coarse_iter);
    icp_point_to_point_.setMaxCorrespondenceDistance(coarse_corr_dist);
  }

  PointCloudT temp_output;
  Eigen::Matrix4f coarse_transform = initial_guess;

  if (config_.use_point_to_plane) {
    icp_point_to_plane_.align(temp_output, coarse_transform);
    coarse_transform = icp_point_to_plane_.getFinalTransformation();
  } else {
    icp_point_to_point_.align(temp_output, coarse_transform);
    coarse_transform = icp_point_to_point_.getFinalTransformation();
  }

  // 精配准阶段
  if (config_.use_point_to_plane) {
    icp_point_to_plane_.setMaximumIterations(config_.max_iterations);
    icp_point_to_plane_.setMaxCorrespondenceDistance(config_.max_correspondence_dist);
    icp_point_to_plane_.setTransformationEpsilon(config_.fine_trans_epsilon);
    icp_point_to_plane_.align(temp_output, coarse_transform);

    result.converged = icp_point_to_plane_.hasConverged();
    result.iteration_num = icp_point_to_plane_.getMaximumIterations();
    result.transformation = icp_point_to_plane_.getFinalTransformation();
  } else {
    icp_point_to_point_.setMaximumIterations(config_.max_iterations);
    icp_point_to_point_.setMaxCorrespondenceDistance(config_.max_correspondence_dist);
    icp_point_to_point_.setTransformationEpsilon(config_.fine_trans_epsilon);
    icp_point_to_point_.align(temp_output, coarse_transform);

    result.converged = icp_point_to_point_.hasConverged();
    result.iteration_num = icp_point_to_point_.getMaximumIterations();
    result.transformation = icp_point_to_point_.getFinalTransformation();
  }

  // 恢复默认参数
  if (config_.use_point_to_plane) {
    icp_point_to_plane_.setTransformationEpsilon(config_.transformation_epsilon);
  } else {
    icp_point_to_point_.setTransformationEpsilon(config_.transformation_epsilon);
  }

  return result;
}

double ICPWrapper::computeFitnessScore(const PointCloudPtr& source,
                                       const PointCloudPtr& target,
                                       const Eigen::Matrix4f& transformation) {
  // 构建 target 的 kdtree
  pcl::KdTreeFLANN<PointT> tree;
  tree.setInputCloud(target);

  PointCloudT transformed_source;
  pcl::transformPointCloud(*source, transformed_source, transformation);

  double score = 0.0;
  int count = 0;

  std::vector<int> indices(1);
  std::vector<float> dists(1);

  for (const auto& pt : transformed_source) {
    if (tree.nearestKSearch(pt, 1, indices, dists) > 0) {
      score += std::sqrt(dists[0]);
      count++;
    }
  }

  return count > 0 ? score / count : 1e6;
}

double ICPWrapper::computeRMSE(const PointCloudPtr& source,
                               const PointCloudPtr& target,
                               const Eigen::Matrix4f& transformation) {
  pcl::KdTreeFLANN<PointT> tree;
  tree.setInputCloud(target);

  PointCloudT transformed_source;
  pcl::transformPointCloud(*source, transformed_source, transformation);

  double mse = 0.0;
  int count = 0;

  std::vector<int> indices(1);
  std::vector<float> dists(1);

  for (const auto& pt : transformed_source) {
    if (tree.nearestKSearch(pt, 1, indices, dists) > 0) {
      mse += dists[0];
      count++;
    }
  }

  return count > 0 ? std::sqrt(mse / count) : 1e6;
}

int ICPWrapper::countInliers(const PointCloudPtr& source,
                             const PointCloudPtr& target,
                             const Eigen::Matrix4f& transformation,
                             double threshold) {
  pcl::KdTreeFLANN<PointT> tree;
  tree.setInputCloud(target);

  PointCloudT transformed_source;
  pcl::transformPointCloud(*source, transformed_source, transformation);

  int inliers = 0;
  std::vector<int> indices(1);
  std::vector<float> dists(1);

  for (const auto& pt : transformed_source) {
    if (tree.nearestKSearch(pt, 1, indices, dists) > 0) {
      if (dists[0] < threshold * threshold) {
        inliers++;
      }
    }
  }

  return inliers;
}

ICPWrapper::PointCloudPtr ICPWrapper::removeOutliers(const PointCloudPtr& cloud,
                                                      double radius,
                                                      int min_neighbors) {
  PointCloudPtr filtered(new PointCloudT());

  pcl::KdTreeFLANN<PointT> tree;
  tree.setInputCloud(cloud);

  std::vector<int> indices;
  std::vector<float> dists;

  for (const auto& pt : *cloud) {
    int count = tree.radiusSearch(pt, radius, indices, dists);
    if (count >= min_neighbors) {
      filtered->push_back(pt);
    }
  }

  return filtered;
}

} // namespace drone_detect_lidar
