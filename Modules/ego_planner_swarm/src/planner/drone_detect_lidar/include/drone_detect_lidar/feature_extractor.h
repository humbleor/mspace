#ifndef _DRONE_DETECT_LIDAR_FEATURE_EXTRACTOR_H_
#define _DRONE_DETECT_LIDAR_FEATURE_EXTRACTOR_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <Eigen/Eigen>

namespace drone_detect_lidar {

/**
 * @brief 特征点类型枚举
 */
enum class FeatureType : int8_t {
  CORNER = 0,    // 角点（高曲率）
  PLANAR = 1,    // 平面点（法向稳定）
  INVALID = 2    // 无效点
};

/**
 * @brief 特征点数据结构
 */
struct FeaturePoint {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector3f point;        // 3D 坐标
  Eigen::Vector3f normal;       // 法向量
  float curvature;              // 曲率
  FeatureType type;             // 特征类型
  uint8_t ring_id;              // 激光线 ID (对于多线 LiDAR)
  double timestamp;             // 时间戳

  FeaturePoint()
    : curvature(0.0f), type(FeatureType::INVALID), ring_id(0), timestamp(0.0) {
    point.setZero();
    normal.setZero();
    normal(2) = 1.0f;  // 默认法向向上
  }
};

/**
 * @brief 特征提取器配置参数
 */
struct FeatureExtractorConfig {
  double voxel_size;              // 体素降采样尺寸 (m)
  int max_feature_points;         // 最大特征点数
  double corner_threshold;        // 角点曲率阈值
  double planar_threshold;        // 平面点曲率阈值（越低越平坦）
  int search_radius;              // 邻域搜索半径 (点数)
  double min_distance;            // 最小点间距 (m)
  int scan_ring_num;              // LiDAR 线数 (16/32/128)
  bool enable_ring_filter;        // 是否启用线滤波

  FeatureExtractorConfig()
    : voxel_size(0.2)
    , max_feature_points(50)
    , corner_threshold(0.1)
    , planar_threshold(0.05)
    , search_radius(5)
    , min_distance(0.3)
    , scan_ring_num(32)
    , enable_ring_filter(true) {}
};

/**
 * @brief 特征提取器主类
 *
 * 功能:
 * 1. 体素降采样
 * 2. 法向量估计
 * 3. 曲率计算
 * 4. 角点/平面点提取
 * 5. 特征点筛选
 */
class FeatureExtractor {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using PointT = pcl::PointXYZINormal;
  using PointCloudT = pcl::PointCloud<PointT>;
  using PointCloudPtr = PointCloudT::Ptr;
  using PointCloudConstPtr = PointCloudT::ConstPtr;

  explicit FeatureExtractor(const FeatureExtractorConfig& config = FeatureExtractorConfig());
  ~FeatureExtractor();

  /**
   * @brief 从原始点云提取特征
   * @param cloud_in 输入点云 (世界系或局部地图系)
   * @param features_out 输出特征点列表
   * @return true 如果提取成功
   */
  bool extract(const PointCloudPtr& cloud_in, std::vector<FeaturePoint>& features_out);

  /**
   * @brief ROS PointCloud2 消息转换接口
   * @param msg ROS 点云消息
   * @param features_out 输出特征点列表
   * @return true 如果提取成功
   */
  bool extractFromROS(const sensor_msgs::PointCloud2::ConstPtr& msg,
                      std::vector<FeaturePoint>& features_out);

  /**
   * @brief 将特征点转换为 ROS 点云消息 (用于可视化/传输)
   * @param features 特征点列表
   * @param cloud_out 输出点云消息
   */
  void toROSMessage(const std::vector<FeaturePoint>& features,
                    sensor_msgs::PointCloud2& cloud_out);

  /**
   * @brief 设置配置参数
   */
  void setConfig(const FeatureExtractorConfig& config);

  /**
   * @brief 获取配置参数
   */
  const FeatureExtractorConfig& getConfig() const { return config_; }

  /**
   * @brief 获取最后处理时间
   */
  double getLastProcessingTime() const { return last_processing_time_; }

  /**
   * @brief 获取最后提取的特征点数
   */
  int getLastFeatureCount() const { return last_feature_count_; }

private:
  FeatureExtractorConfig config_;

  // PCL 滤波器
  pcl::VoxelGrid<PointT> voxel_grid_;
  pcl::search::KdTree<PointT>::Ptr search_method_;
  pcl::NormalEstimation<PointT, PointT> normal_estimator_;

  // 中间数据
  PointCloudPtr filtered_cloud_;

  // 性能统计
  double last_processing_time_;
  int last_feature_count_;

  /**
   * @brief 体素降采样
   */
  PointCloudPtr voxelFilter(const PointCloudPtr& cloud);

  /**
   * @brief 估计法向量
   */
  void estimateNormals(const PointCloudPtr& cloud);

  /**
   * @brief 计算曲率
   * @param cloud 输入点云 (带法向量)
   * @param curvatures 输出曲率数组
   */
  void computeCurvature(const PointCloudPtr& cloud, std::vector<float>& curvatures);

  /**
   * @brief 提取角点
   */
  void extractCornerPoints(const PointCloudPtr& cloud,
                          const std::vector<float>& curvatures,
                          std::vector<FeaturePoint>& features);

  /**
   * @brief 提取平面点
   */
  void extractPlanarPoints(const PointCloudPtr& cloud,
                          std::vector<FeaturePoint>& features);

  /**
   * @brief 特征点筛选 (非极大值抑制、距离过滤)
   */
  void filterFeatures(std::vector<FeaturePoint>& features);

  /**
   * @brief 从点云获取 ring ID
   */
  uint8_t getRingId(const PointT& pt, const PointCloudPtr& original_cloud);
};

} // namespace drone_detect_lidar

#endif // _DRONE_DETECT_LIDAR_FEATURE_EXTRACTOR_H_
