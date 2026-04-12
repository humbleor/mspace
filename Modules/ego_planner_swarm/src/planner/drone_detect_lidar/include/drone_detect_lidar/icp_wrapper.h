#ifndef _DRONE_DETECT_LIDAR_ICP_WRAPPER_H_
#define _DRONE_DETECT_LIDAR_ICP_WRAPPER_H_

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "drone_detect_lidar/feature_extractor.h"

namespace drone_detect_lidar {

/**
 * @brief ICP 配准结果结构体
 */
struct ICPResult {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Matrix4f transformation;   // 变换矩阵 T_source_to_target
  bool converged;                   // 是否收敛
  double fitness_score;             // 适配度分数 (越小越好)
  int iteration_num;                // 实际迭代次数
  double rmse;                      // 均方根误差
  int inlier_count;                 // 内点数量
  double inlier_ratio;              // 内点比例

  ICPResult()
    : transformation(Eigen::Matrix4f::Identity())
    , converged(false)
    , fitness_score(0.0)
    , iteration_num(0)
    , rmse(0.0)
    , inlier_count(0)
    , inlier_ratio(0.0) {}
};

/**
 * @brief ICP 配置参数
 */
struct ICPConfig {
  int max_iterations;               // 最大迭代次数
  double transformation_epsilon;    // 变换收敛阈值
  double euclidean_fitness_epsilon; // Euclidean 适配度收敛阈值
  double fitness_score_thresh;      // 适配度阈值 (超过认为失败)
  double max_correspondence_dist;   // 对应点最大距离 (m)
  bool use_point_to_plane;          // 是否使用 point-to-plane ICP
  double outlier_ratio_thresh;      // 外点比例阈值
  int min_inlier_count;             // 最小内点数量

  // 重叠区域配准参数
  double max_uav_distance;          // 最大无人机距离 (超过不配准)
  double overlap_radius;            // 重叠搜索半径 (m)
  int min_overlap_points;           // 最小重叠点数

  // 分层配准参数
  bool use_hierarchical;            // 是否使用分层配准
  int coarse_iterations;            // 粗配准迭代次数
  double fine_trans_epsilon;        // 精配准平移阈值

  ICPConfig()
    : max_iterations(50)
    , transformation_epsilon(1e-4)
    , euclidean_fitness_epsilon(1e-4)
    , fitness_score_thresh(1.0)
    , max_correspondence_dist(1.5)
    , use_point_to_plane(true)
    , outlier_ratio_thresh(0.5)
    , min_inlier_count(5)
    , max_uav_distance(10.0)
    , overlap_radius(3.0)
    , min_overlap_points(10)
    , use_hierarchical(false)
    , coarse_iterations(10)
    , fine_trans_epsilon(1e-6) {}
};

/**
 * @brief ICP 配准封装类
 *
 * 功能:
 * 1. 支持 point-to-point 和 point-to-plane ICP
 * 2. 支持初始值输入
 * 3. 配准质量评估
 * 4. 外点剔除
 */
class ICPWrapper {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using PointT = pcl::PointXYZINormal;
  using PointCloudT = pcl::PointCloud<PointT>;
  using PointCloudPtr = PointCloudT::Ptr;
  using PointCloudConstPtr = PointCloudT::ConstPtr;

  explicit ICPWrapper(const ICPConfig& config = ICPConfig());
  ~ICPWrapper();

  /**
   * @brief 执行 ICP 配准
   * @param source_cloud 源点云 (A 机特征点云，在 A 的局部地图系下)
   * @param target_cloud 目标点云 (B 机局部地图/特征点云)
   * @param initial_guess 初始变换矩阵 (从里程计/上一次配准获得)
   * @return ICPResult 配准结果
   */
  ICPResult align(const PointCloudPtr& source_cloud,
                  const PointCloudPtr& target_cloud,
                  const Eigen::Matrix4f& initial_guess = Eigen::Matrix4f::Identity());

  /**
   * @brief 从 FeaturePoint 列表创建 PCL 点云
   */
  PointCloudPtr toPCLCloud(const std::vector<FeaturePoint>& features);

  /**
   * @brief 设置配置参数
   */
  void setConfig(const ICPConfig& config);

  /**
   * @brief 获取配置参数
   */
  const ICPConfig& getConfig() const { return config_; }

  /**
   * @brief 获取最后配准结果
   */
  const ICPResult& getLastResult() const { return last_result_; }

  /**
   * @brief 检查配准是否有效
   */
  bool isValidResult() const;

  /**
   * @brief 设置变换矩阵 (用于外部更新)
   */
  void setLastTransformation(const Eigen::Matrix4f& trans);

private:
  ICPConfig config_;
  ICPResult last_result_;

  // PCL ICP 对象
  pcl::IterativeClosestPoint<PointT, PointT> icp_point_to_point_;
  pcl::IterativeClosestPointWithNormals<PointT, PointT> icp_point_to_plane_;

  // Kd-tree 用于搜索
  pcl::KdTreeFLANN<PointT>::Ptr kdtree_;

  /**
   * @brief 执行分层 ICP (粗配准 + 精配准)
   */
  ICPResult hierarchicalAlign(const PointCloudPtr& source,
                              const PointCloudPtr& target,
                              const Eigen::Matrix4f& initial_guess);

  /**
   * @brief 计算适配度分数
   */
  double computeFitnessScore(const PointCloudPtr& source,
                            const PointCloudPtr& target,
                            const Eigen::Matrix4f& transformation);

  /**
   * @brief 计算 RMSE
   */
  double computeRMSE(const PointCloudPtr& source,
                    const PointCloudPtr& target,
                    const Eigen::Matrix4f& transformation);

  /**
   * @brief 统计内点数量
   */
  int countInliers(const PointCloudPtr& source,
                  const PointCloudPtr& target,
                  const Eigen::Matrix4f& transformation,
                  double threshold);

  /**
   * @brief 外点剔除
   */
  PointCloudPtr removeOutliers(const PointCloudPtr& cloud, double radius, int min_neighbors);
};

} // namespace drone_detect_lidar

#endif // _DRONE_DETECT_LIDAR_ICP_WRAPPER_H_
