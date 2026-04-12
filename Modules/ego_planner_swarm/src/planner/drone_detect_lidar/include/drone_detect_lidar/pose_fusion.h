#ifndef _DRONE_DETECT_LIDAR_POSE_FUSION_H_
#define _DRONE_DETECT_LIDAR_POSE_FUSION_H_

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "drone_detect_lidar/feature_extractor.h"
#include "drone_detect_lidar/icp_wrapper.h"

namespace drone_detect_lidar {

/**
 * @brief 相对位姿信息
 */
struct RelativePose {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector3d position;       // 相对位置
  Eigen::Quaterniond orientation; // 相对姿态
  ros::Time timestamp;            // 时间戳
  int sender_id;                  // 发送者 ID
  int receiver_id;                // 接收者 ID
  double confidence;              // 置信度 [0-1]
  bool valid;                     // 是否有效

  RelativePose()
    : confidence(0.0), valid(false) {
    position.setZero();
    orientation.setIdentity();
  }
};

/**
 * @brief 位姿误差输出
 */
struct PoseError {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector3d delta_position;   // 位置误差
  Eigen::Vector3d delta_euler;      // 姿态误差 (roll/pitch/yaw)
  Eigen::Vector3d corrected_position; // 修正后的位置
  double fitness_score;             // ICP 适配度分数
  ros::Time timestamp;
  std::string frame_id;
  bool valid;

  PoseError()
    : delta_position(Eigen::Vector3d::Zero())
    , delta_euler(Eigen::Vector3d::Zero())
    , corrected_position(Eigen::Vector3d::Zero())
    , fitness_score(0.0)
    , valid(false) {}
};

/**
 * @brief 邻居无人机状态缓存
 */
struct NeighborState {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  int drone_id;
  Eigen::Vector3d position;         // 世界系位置
  Eigen::Quaterniond orientation;   // 世界系姿态
  Eigen::Vector3d velocity;         // 世界系速度
  ros::Time last_update_time;       // 最后更新时间
  ros::Time feature_timestamp;      // 特征云时间戳
  bool has_feature;                 // 是否有特征云
  std::vector<FeaturePoint> feature_cloud; // 特征点云

  NeighborState()
    : drone_id(-1)
    , has_feature(false) {
    position.setZero();
    orientation.setIdentity();
    velocity.setZero();
  }
};

/**
 * @brief 位姿融合与误差计算类
 *
 * 功能:
 * 1. 接收邻居无人机特征云和里程计
 * 2. 与自身局部地图进行 ICP 配准
 * 3. 计算相对位姿误差
 * 4. 输出修正建议
 */
class PoseFusion {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit PoseFusion(int my_id, const ICPConfig& icp_config = ICPConfig());
  ~PoseFusion();

  /**
   * @brief 处理接收到的邻居特征云消息
   * @param feature_cloud 邻居特征点云 (sensor_msgs::PointCloud2)
   * @param neighbor_odom 邻居里程计
   * @param sender_id 发送者 ID
   */
  void processNeighborFeature(const sensor_msgs::PointCloud2::ConstPtr& feature_cloud,
                              const nav_msgs::Odometry::ConstPtr& neighbor_odom,
                              int sender_id);

  /**
   * @brief 处理自身局部地图
   * @param local_map 自身局部地图点云
   */
  void setLocalMap(const sensor_msgs::PointCloud2::ConstPtr& local_map);

  /**
   * @brief 处理自身里程计
   */
  void setMyOdometry(const nav_msgs::Odometry::ConstPtr& my_odom);

  /**
   * @brief 执行 ICP 配准并计算位姿误差
   * @param neighbor_id 邻居无人机 ID
   * @return PoseError 位姿误差 (如果计算成功)
   */
  PoseError computePoseError(int neighbor_id);

  /**
   * @brief 获取相对位姿
   */
  RelativePose getRelativePose(int neighbor_id) const;

  /**
   * @brief 获取所有邻居的位姿误差
   */
  std::map<int, PoseError> getAllPoseErrors();

  /**
   * @brief 设置配置参数
   */
  void setICPConfig(const ICPConfig& config);

  /**
   * @brief 获取邻居 ID 列表
   */
  std::vector<int> getNeighborIds() const;

  /**
   * @brief 清除过期邻居状态
   * @param timeout 超时时间 (秒)
   */
  void cleanupExpiredNeighbors(double timeout = 5.0);

  /**
   * @brief 获取 ICP 配准器 (用于访问配准结果)
   */
  ICPWrapper& icpWrapper() { return icp_wrapper_; }
  const ICPWrapper& icpWrapper() const { return icp_wrapper_; }

private:
  int my_id_;
  ICPConfig icp_config_;

  // 自身状态
  nav_msgs::Odometry::ConstPtr my_odom_;
  sensor_msgs::PointCloud2::ConstPtr my_local_map_;

  // 邻居状态缓存
  std::map<int, NeighborState> neighbor_states_;

  // ICP 配准器
  ICPWrapper icp_wrapper_;

  // 特征提取器 (用于转换消息格式)
  FeatureExtractor feature_extractor_;

  // 最后计算的位姿误差
  std::map<int, PoseError> last_pose_errors_;
  std::map<int, RelativePose> last_relative_poses_;

  /**
   * @brief 从 ROS 消息转换到 FeaturePoint 列表
   */
  bool fromROSMessage(const sensor_msgs::PointCloud2& msg,
                      std::vector<FeaturePoint>& features);

  /**
   * @brief 运动补偿 (处理时间同步问题)
   * @param pose 原始位姿
   * @param velocity 速度
   * @param dt 时间差
   * @return 补偿后的位姿
   */
  Eigen::Matrix4d motionCompensation(const Eigen::Matrix4d& pose,
                                     const Eigen::Vector3d& velocity,
                                     double dt);

  /**
   * @brief 坐标系变换链计算
   * T_W_B = T_W_A * T_A_LA * T_LA_LB * T_LB_B
   */
  Eigen::Matrix4d computeWorldPose(
      const Eigen::Matrix4d& T_W_A,   // A 机世界位姿
      const Eigen::Matrix4d& T_A_LA,  // A 机体系→局部地图系 (假设为单位矩阵)
      const Eigen::Matrix4d& T_LA_LB, // ICP 结果：A 局部系→B 局部系
      const Eigen::Matrix4d& T_LB_B); // B 机局部地图系→机体系 (假设为单位矩阵)

  /**
   * @brief 计算置信度
   */
  double computeConfidence(const ICPResult& icp_result);

  /**
   * @brief 四元数转欧拉角
   */
  Eigen::Vector3d quaternionToEuler(const Eigen::Quaterniond& q);
};

} // namespace drone_detect_lidar

#endif // _DRONE_DETECT_LIDAR_POSE_FUSION_H_
