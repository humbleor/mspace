#ifndef _DRONE_DETECT_LIDAR_TREE_DETECTOR_H_
#define _DRONE_DETECT_LIDAR_TREE_DETECTOR_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Eigen>
#include <memory>
#include "drone_detect_lidar/patchworkpp.h"

namespace drone_detect_lidar {

struct TreeInfo {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  uint32_t id;
  double x, y;           // 树干底部位置 (world frame XY)
  double z_base;         // 地面高度 (Z)
  double height;         // 树高 (m)
  double diameter;       // 树干直径 (m)
  double linearity;      // PCA 线性度 [0,1]
  double planarity;      // PCA 平面度 [0,1]
  double roundness;      // 横截面圆度 [0,1]
  double confidence;     // 检测置信度 [0,1]

  TreeInfo()
    : id(0), x(0), y(0), z_base(0), height(0), diameter(0),
      linearity(0), planarity(0), roundness(0), confidence(0) {}
};

struct TreeDetectorConfig {
  double tree_voxel_size;       // 降采样分辨率 (m)
  double tree_cluster_tolerance; // 聚类最大间距 (m)
  int tree_min_cluster_size;    // 最小聚类点数
  int tree_max_cluster_size;    // 最大聚类点数
  double tree_linearity_threshold; // PCA 线性度阈值
  double tree_planarity_threshold; // PCA 平面度阈值（排除扁平物体）
  double tree_min_height_spread;   // 最小高度延伸 (m)，排除矮灌木
  double tree_min_roundness;       // 横截面圆度阈值

  // PatchWork++ 地面剔除参数
  double patchwork_sensor_height; // 传感器离地高度 (m)
  double patchwork_max_range;     // PatchWork++ 最大探测距离 (m)
  double patchwork_min_range;     // PatchWork++ 最小探测距离 (m)

  TreeDetectorConfig() :
      tree_voxel_size(0.15), tree_cluster_tolerance(0.3),
      tree_min_cluster_size(20), tree_max_cluster_size(5000),
      tree_linearity_threshold(0.6), tree_planarity_threshold(0.3),
      tree_min_height_spread(0.5), tree_min_roundness(0.4),
      patchwork_sensor_height(1.0), patchwork_max_range(80.0),
      patchwork_min_range(0.5) {}
};

/**
 * @brief 森林场景树干检测器
 *
 * 流程:
 * 1. 体素降采样
 * 2. PatchWork++ 地面剔除
 * 3. 欧氏聚类
 * 4. PCA筛选：线性度 + 平面度 + 高度延伸 + 横截面圆度
 * 5. 树干参数估计：底部位置 + 高度 + 直径
 */
class TreeDetector {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using PointT = pcl::PointXYZ;
  using PointCloudT = pcl::PointCloud<PointT>;
  using PointCloudPtr = PointCloudT::Ptr;
  using PointCloudConstPtr = PointCloudT::ConstPtr;

  explicit TreeDetector(const TreeDetectorConfig& config = TreeDetectorConfig());
  ~TreeDetector();

  bool detect(const PointCloudPtr& cloud_in, std::vector<TreeInfo>& trees_out);

  bool detectFromROS(const sensor_msgs::PointCloud2::ConstPtr& msg,
                     std::vector<TreeInfo>& trees_out);

  void setConfig(const TreeDetectorConfig& config);
  const TreeDetectorConfig& getConfig() const { return config_; }
  double getLastProcessingTime() const { return last_processing_time_; }
  int getLastTreeCount() const { return last_tree_count_; }

private:
  TreeDetectorConfig config_;
  double last_processing_time_;
  int last_tree_count_;

  // PatchWork++ ground segmentation
  std::unique_ptr<patchwork::PatchWorkpp> patchworkpp_;
  patchwork::Params patchwork_params_;

  PointCloudPtr removeGroundPatchWork(const PointCloudPtr& cloud);
  std::vector<PointCloudPtr> clusterPoints(const PointCloudPtr& cloud);
  bool computeTreeParameters(const PointCloudPtr& cluster, TreeInfo& tree);
};

} // namespace drone_detect_lidar

#endif // _DRONE_DETECT_LIDAR_TREE_DETECTOR_H_
