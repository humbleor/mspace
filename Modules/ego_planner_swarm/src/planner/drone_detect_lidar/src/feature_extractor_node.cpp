#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include "drone_detect_lidar/feature_extractor.h"

using namespace drone_detect_lidar;

class FeatureExtractorNode {
public:
  FeatureExtractorNode(ros::NodeHandle& nh)
    : nh_(nh)
    , extractor_()
    , has_odom_(false) {

    // 加载参数
    loadParameters();

    // 初始化提取器
    FeatureExtractorConfig config;
    config.voxel_size = voxel_size_;
    config.max_feature_points = max_features_;
    config.corner_threshold = corner_threshold_;
    config.planar_threshold = planar_threshold_;
    config.search_radius = search_radius_;
    config.min_distance = min_distance_;
    config.scan_ring_num = scan_ring_num_;

    extractor_.setConfig(config);

    // 订阅者
    cloud_sub_ = nh_.subscribe(lidar_topic_, 10,
                               &FeatureExtractorNode::cloudCallback, this,
                               ros::TransportHints().tcpNoDelay());

    odom_sub_ = nh_.subscribe(odom_topic_, 100,
                              &FeatureExtractorNode::odomCallback, this,
                              ros::TransportHints().tcpNoDelay());

    // 发布者
    feature_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("feature_cloud", 10);
    debug_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("feature_cloud_debug", 10);

    ROS_INFO("[FeatureExtractorNode] Initialized for drone_%d", drone_id_);
  }

  void run() {
    ros::Rate rate(publish_frequency_);
    while (ros::ok()) {
      ros::spinOnce();
      rate.sleep();
    }
  }

private:
  ros::NodeHandle& nh_;
  FeatureExtractor extractor_;

  // 参数
  int drone_id_;
  std::string lidar_topic_;
  std::string odom_topic_;
  double voxel_size_;
  int max_features_;
  double corner_threshold_;
  double planar_threshold_;
  int search_radius_;
  double min_distance_;
  int scan_ring_num_;
  double publish_frequency_;

  // ROS 接口
  ros::Subscriber cloud_sub_;
  ros::Subscriber odom_sub_;
  ros::Publisher feature_pub_;
  ros::Publisher debug_pub_;

  // 状态
  bool has_odom_;
  nav_msgs::Odometry::ConstPtr last_odom_;
  sensor_msgs::PointCloud2::ConstPtr last_cloud_;

  void loadParameters() {
    nh_.param("drone_id", drone_id_, 0);
    nh_.param("lidar_topic", lidar_topic_, std::string("/lidar_cloud"));
    nh_.param("odom_topic", odom_topic_, std::string("/odometry"));
    nh_.param("voxel_size", voxel_size_, 0.2);
    nh_.param("max_features", max_features_, 50);
    nh_.param("corner_threshold", corner_threshold_, 0.1);
    nh_.param("planar_threshold", planar_threshold_, 0.05);
    nh_.param("search_radius", search_radius_, 5);
    nh_.param("min_distance", min_distance_, 0.3);
    nh_.param("scan_ring_num", scan_ring_num_, 32);
    nh_.param("publish_frequency", publish_frequency_, 5.0);
  }

  void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    last_cloud_ = msg;

    if (!has_odom_) {
      ROS_WARN_THROTTLE(1.0, "[FeatureExtractorNode] Waiting for odometry...");
      return;
    }

    // 提取特征
    std::vector<FeaturePoint> features;
    if (!extractor_.extractFromROS(msg, features)) {
      ROS_WARN("[FeatureExtractorNode] Feature extraction failed");
      return;
    }

    ROS_INFO("[FeatureExtractorNode] Extracted %d features from drone_%d",
             static_cast<int>(features.size()), drone_id_);

    // 转换为 ROS 消息
    sensor_msgs::PointCloud2 feature_msg;
    extractor_.toROSMessage(features, feature_msg);

    // 设置消息头
    feature_msg.header = msg->header;
    feature_msg.header.frame_id = msg->header.frame_id;

    // 添加无人机 ID 到 header
    feature_msg.header.stamp = ros::Time::now();

    // 发布
    feature_pub_.publish(feature_msg);

    // 发布调试信息
    debug_pub_.publish(feature_msg);
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    last_odom_ = msg;
    has_odom_ = true;
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "feature_extractor_node");
  ros::NodeHandle nh("~");

  FeatureExtractorNode node(nh);
  node.run();

  return 0;
}
