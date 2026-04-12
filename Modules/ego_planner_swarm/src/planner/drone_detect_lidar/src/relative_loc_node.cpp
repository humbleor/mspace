#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32.h>
#include <pcl_conversions/pcl_conversions.h>
#include "drone_detect_lidar/pose_fusion.h"
#include "drone_detect_lidar/icp_wrapper.h"

using namespace drone_detect_lidar;

/**
 * @brief LiDAR 相对定位节点
 *
 * 功能:
 * 1. 接收邻居无人机的特征点云和里程计
 * 2. 与自身局部地图进行 ICP 配准
 * 3. 计算并发布位姿误差
 */
class RelativeLocNode {
public:
  RelativeLocNode(ros::NodeHandle& nh)
    : nh_(nh)
    , pose_fusion_(nullptr)
    , has_local_map_(false)
    , has_odom_(false)
    , last_fitness_(1e6) {

    // 加载参数
    loadParameters();

    // 初始化 ICP 配置
    ICPConfig icp_config;
    icp_config.max_iterations = icp_max_iterations_;
    icp_config.transformation_epsilon = icp_trans_epsilon_;
    icp_config.fitness_score_thresh = icp_fitness_thresh_;
    icp_config.max_correspondence_dist = icp_corr_dist_;
    icp_config.use_point_to_plane = icp_use_point_to_plane_;
    icp_config.min_inlier_count = icp_min_inliers_;
    icp_config.overlap_radius = icp_overlap_radius_;
    icp_config.min_overlap_points = icp_min_overlap_points_;

    // 初始化位姿融合器
    pose_fusion_.reset(new PoseFusion(drone_id_, icp_config));

    // 订阅者
    feature_sub_ = nh_.subscribe("received_feature_cloud", 10,
                                 &RelativeLocNode::featureCallback, this);

    neighbor_odom_sub_ = nh_.subscribe("neighbor_odom", 100,
                                       &RelativeLocNode::neighborOdomCallback, this);

    local_map_sub_ = nh_.subscribe("local_map", 10,
                                   &RelativeLocNode::localMapCallback, this);

    my_odom_sub_ = nh_.subscribe(odom_topic_, 100,
                                 &RelativeLocNode::myOdomCallback, this);

    // 发布者
    relative_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("relative_pose", 10);
    pose_error_pub_ = nh_.advertise<geometry_msgs::Vector3>("pose_error", 10);
    icp_result_pub_ = nh_.advertise<std_msgs::Float32>("icp_fitness", 10);

    // 定时器
    compute_timer_ = nh_.createTimer(ros::Duration(1.0 / compute_frequency_),
                                     &RelativeLocNode::computeTimerCallback, this);

    ROS_INFO("[RelativeLocNode] Initialized for drone_%d", drone_id_);
  }

  void run() {
    ros::Rate rate(10.0);
    while (ros::ok()) {
      ros::spinOnce();
      rate.sleep();
    }
  }

private:
  ros::NodeHandle& nh_;
  std::unique_ptr<PoseFusion> pose_fusion_;

  // 参数
  int drone_id_;
  std::string odom_topic_;
  int icp_max_iterations_;
  double icp_trans_epsilon_;
  double icp_fitness_thresh_;
  double icp_corr_dist_;
  bool icp_use_point_to_plane_;
  int icp_min_inliers_;
  double icp_overlap_radius_;
  int icp_min_overlap_points_;
  double compute_frequency_;
  double neighbor_timeout_;

  // ROS 接口
  ros::Subscriber feature_sub_;
  ros::Subscriber neighbor_odom_sub_;
  ros::Subscriber local_map_sub_;
  ros::Subscriber my_odom_sub_;

  ros::Publisher relative_pose_pub_;
  ros::Publisher pose_error_pub_;
  ros::Publisher icp_result_pub_;

  ros::Timer compute_timer_;

  // 状态
  bool has_local_map_;
  bool has_odom_;
  bool has_neighbor_feature_;
  sensor_msgs::PointCloud2::ConstPtr last_local_map_;
  nav_msgs::Odometry::ConstPtr last_my_odom_;
  sensor_msgs::PointCloud2::ConstPtr last_neighbor_feature_;
  nav_msgs::Odometry::ConstPtr last_neighbor_odom_;
  int last_neighbor_id_;
  double last_fitness_;

  void loadParameters() {
    nh_.param("drone_id", drone_id_, 0);
    nh_.param("odom_topic", odom_topic_, std::string("/odometry"));
    nh_.param("icp_max_iterations", icp_max_iterations_, 15);
    nh_.param("icp_trans_epsilon", icp_trans_epsilon_, 1e-4);
    nh_.param("icp_fitness_thresh", icp_fitness_thresh_, 1.0);
    nh_.param("icp_corr_dist", icp_corr_dist_, 0.5);
    nh_.param("icp_use_point_to_plane", icp_use_point_to_plane_, true);
    nh_.param("icp_min_inliers", icp_min_inliers_, 10);
    nh_.param("icp_overlap_radius", icp_overlap_radius_, 2.0);
    nh_.param("icp_min_overlap_points", icp_min_overlap_points_, 15);
    nh_.param("compute_frequency", compute_frequency_, 5.0);
    nh_.param("neighbor_timeout", neighbor_timeout_, 5.0);
  }

  void featureCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    last_neighbor_feature_ = msg;
    has_neighbor_feature_ = true;
    ROS_INFO("[RelativeLocNode] Received feature cloud: %d points, stamp=%d",
             msg->width * msg->height, msg->header.stamp.toSec());
  }

  void neighborOdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    last_neighbor_odom_ = msg;

    // 根据当前无人机 ID 确定邻居 ID
    // UAV1 的邻居是 UAV2，UAV2 的邻居是 UAV1
    int sender_id = (drone_id_ == 1) ? 2 : 1;
    last_neighbor_id_ = sender_id;

    // ROS_INFO("[RelativeLocNode] Received neighbor odom from drone_%d", sender_id);

    // 如果有特征云，立即处理
    if (has_neighbor_feature_ && last_neighbor_feature_) {
      ROS_INFO("[RelativeLocNode] Processing feature+odom from drone_%d", sender_id);
      pose_fusion_->processNeighborFeature(last_neighbor_feature_, msg, sender_id);
    }
  }

  void localMapCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    last_local_map_ = msg;
    has_local_map_ = true;
    pose_fusion_->setLocalMap(msg);
    ROS_DEBUG("[RelativeLocNode] Updated local map: %d points", msg->width * msg->height);
  }

  void myOdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    last_my_odom_ = msg;
    has_odom_ = true;
    pose_fusion_->setMyOdometry(msg);
  }

  void computeTimerCallback(const ros::TimerEvent& event) {
    // 检查数据完整性
    if (!has_local_map_ || !has_odom_ || !has_neighbor_feature_) {
      ROS_DEBUG_THROTTLE(1.0, "[RelativeLocNode] Waiting for data: map=%d, odom=%d, feature=%d",
                         has_local_map_, has_odom_, has_neighbor_feature_);
      return;
    }

    // 清理过期邻居
    pose_fusion_->cleanupExpiredNeighbors(neighbor_timeout_);

    // 获取邻居列表并计算位姿误差
    std::vector<int> neighbors = pose_fusion_->getNeighborIds();
    ROS_INFO_THROTTLE(1.0, "[RelativeLocNode] Neighbor count: %zu", neighbors.size());

    for (int neighbor_id : neighbors) {
      if (neighbor_id == drone_id_) continue;

      ROS_INFO_THROTTLE(1.0, "[RelativeLocNode] Computing pose error for neighbor %d", neighbor_id);
      PoseError error = pose_fusion_->computePoseError(neighbor_id);

      if (error.valid) {
        // 发布相对位姿
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.stamp = error.timestamp;
        pose_msg.header.frame_id = error.frame_id;
        pose_msg.pose.position.x = error.corrected_position.x();
        pose_msg.pose.position.y = error.corrected_position.y();
        pose_msg.pose.position.z = error.corrected_position.z();
        pose_msg.pose.orientation.w = 1.0;
        pose_msg.pose.orientation.x = 0.0;
        pose_msg.pose.orientation.y = 0.0;
        pose_msg.pose.orientation.z = 0.0;
        relative_pose_pub_.publish(pose_msg);

        // 发布位置误差
        geometry_msgs::Vector3 error_msg;
        error_msg.x = error.delta_position.x();
        error_msg.y = error.delta_position.y();
        error_msg.z = error.delta_position.z();
        pose_error_pub_.publish(error_msg);

        // 更新 ICP fitness score
        last_fitness_ = error.fitness_score;

        ROS_INFO("[RelativeLocNode] drone_%d -> pos_err: %.3f, %.3f, %.3f (norm=%.3fm, fitness=%.4f)",
                 neighbor_id, error_msg.x, error_msg.y, error_msg.z,
                 error.delta_position.norm(), last_fitness_);
      } else {
        ROS_WARN_THROTTLE(1.0, "[RelativeLocNode] Pose error computation failed for neighbor %d", neighbor_id);
      }
    }

    // 发布 ICP 质量指标
    std_msgs::Float32 fitness_msg;
    fitness_msg.data = static_cast<float>(last_fitness_);
    icp_result_pub_.publish(fitness_msg);
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "relative_loc_node");
  ros::NodeHandle nh("~");

  RelativeLocNode node(nh);
  node.run();

  return 0;
}
