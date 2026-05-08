#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include "drone_detect_lidar/tree_detector.h"
#include "drone_detect_lidar/Tree.h"
#include "drone_detect_lidar/TreeDetection.h"

using namespace drone_detect_lidar;

/**
 * @brief 树检测 ROS 节点
 *
 * 订阅: LiDAR 点云 + 里程计
 * 发布: TreeDetection (检测到的树干列表) + tree_cloud (RViz 可视化)
 */
class TreeDetectorNode {
public:
  TreeDetectorNode(ros::NodeHandle& nh)
    : nh_(nh), has_odom_(false) {

    loadParameters();

    detector_.reset(new TreeDetector(detector_config_));

    cloud_sub_ = nh_.subscribe("lidar_cloud", 10,
                               &TreeDetectorNode::cloudCallback, this);
    odom_sub_ = nh_.subscribe("odom", 100,
                              &TreeDetectorNode::odomCallback, this);

    tree_detection_pub_ = nh_.advertise<drone_detect_lidar::TreeDetection>("tree_detection", 10);
    tree_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud>("tree_cloud", 10);

    ROS_INFO("[TreeDetectorNode] Initialized for drone_%d", drone_id_);
  }

private:
  ros::NodeHandle& nh_;
  std::unique_ptr<TreeDetector> detector_;
  ros::Subscriber cloud_sub_;
  ros::Subscriber odom_sub_;
  ros::Publisher tree_detection_pub_;
  ros::Publisher tree_cloud_pub_;

  int drone_id_;
  std::string frame_id_;
  bool has_odom_;
  nav_msgs::Odometry::ConstPtr last_odom_;
  TreeDetectorConfig detector_config_;

  void loadParameters() {
    nh_.param("drone_id", drone_id_, 0);
    nh_.param<std::string>("frame_id", frame_id_, "world");
    nh_.param("tree_height_min", detector_config_.tree_height_min, 0.3);
    nh_.param("tree_height_max", detector_config_.tree_height_max, 3.0);
    nh_.param("tree_voxel_size", detector_config_.tree_voxel_size, 0.15);
    nh_.param("tree_cluster_tolerance", detector_config_.tree_cluster_tolerance, 0.3);
    nh_.param("tree_min_cluster_size", detector_config_.tree_min_cluster_size, 20);
    nh_.param("tree_max_cluster_size", detector_config_.tree_max_cluster_size, 5000);
    nh_.param("tree_linearity_threshold", detector_config_.tree_linearity_threshold, 0.6);
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    last_odom_ = msg;
    has_odom_ = true;
  }

  void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    std::vector<TreeInfo> trees;
    bool ok = detector_->detectFromROS(msg, trees);

    if (!ok || trees.empty()) {
      ROS_DEBUG_THROTTLE(2.0, "[TreeDetectorNode] No trees detected");
      return;
    }

    // 发布 TreeDetection 消息
    drone_detect_lidar::TreeDetection det_msg;
    det_msg.header.stamp = msg->header.stamp;
    det_msg.header.frame_id = msg->header.frame_id;
    det_msg.drone_id = drone_id_;
    det_msg.trees.resize(trees.size());

    for (size_t i = 0; i < trees.size(); i++) {
      det_msg.trees[i].id = trees[i].id;
      det_msg.trees[i].x = trees[i].x;
      det_msg.trees[i].y = trees[i].y;
      det_msg.trees[i].z_base = trees[i].z_base;
      det_msg.trees[i].height = trees[i].height;
      det_msg.trees[i].diameter = trees[i].diameter;
      det_msg.trees[i].linearity = trees[i].linearity;
      det_msg.trees[i].confidence = trees[i].confidence;
    }

    if (has_odom_ && last_odom_) {
      det_msg.odometry = *last_odom_;
    }

    tree_detection_pub_.publish(det_msg);

    // 发布可视化点云（每个树干中心一个点）
    sensor_msgs::PointCloud cloud_vis;
    cloud_vis.header.stamp = msg->header.stamp;
    cloud_vis.header.frame_id = msg->header.frame_id;
    cloud_vis.points.resize(trees.size());
    for (size_t i = 0; i < trees.size(); i++) {
      cloud_vis.points[i].x = trees[i].x;
      cloud_vis.points[i].y = trees[i].y;
      cloud_vis.points[i].z = trees[i].z_base + trees[i].height / 2.0;
    }
    tree_cloud_pub_.publish(cloud_vis);

    ROS_INFO("[TreeDetectorNode] Published %zu tree detections", trees.size());
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "tree_detector_node");
  ros::NodeHandle nh("~");
  TreeDetectorNode node(nh);
  ros::spin();
  return 0;
}
