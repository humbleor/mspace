#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <mutex>
#include "drone_detect_lidar/tree_detector.h"
#include "drone_detect_lidar/Tree.h"
#include "drone_detect_lidar/TreeDetection.h"

using namespace drone_detect_lidar;

/**
 * @brief 树检测 ROS 节点
 *
 * 订阅: LiDAR 点云 + 里程计
 * 发布: TreeDetection (检测到的树干列表) + tree_cloud (RViz 可视化)
 *
 * 点云积累: 积累 N 帧后再检测（默认 3 帧），提高森林场景树干检测鲁棒性。
 */
class TreeDetectorNode {
public:
  TreeDetectorNode(ros::NodeHandle& nh)
    : nh_(nh), has_odom_(false), accumulation_frame_count_(0), frame_counter_(0) {

    loadParameters();

    detector_.reset(new TreeDetector(detector_config_));

    cloud_sub_ = nh_.subscribe("lidar_cloud", 10,
                               &TreeDetectorNode::cloudCallback, this);
    odom_sub_ = nh_.subscribe("odom", 100,
                              &TreeDetectorNode::odomCallback, this);

    tree_detection_pub_ = nh_.advertise<drone_detect_lidar::TreeDetection>("tree_detection", 10);
    tree_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("tree_cloud", 10);

    ROS_INFO("[TreeDetectorNode] Initialized for drone_%d (accumulate %d frames)",
             drone_id_, accumulation_frame_count_);
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

  std::mutex odom_mutex_;
  bool has_odom_;
  nav_msgs::Odometry::ConstPtr last_odom_;
  TreeDetectorConfig detector_config_;

  // 点云积累
  int accumulation_frame_count_;    // 0 = 逐帧检测, >0 = 积累 N 帧
  int frame_counter_;
  std::vector<sensor_msgs::PointCloud2::ConstPtr> cloud_buffer_;

  void loadParameters() {
    nh_.param("drone_id", drone_id_, 0);
    nh_.param<std::string>("frame_id", frame_id_, "world");
    nh_.param("tree_voxel_size", detector_config_.tree_voxel_size, 0.15);
    nh_.param("tree_cluster_tolerance", detector_config_.tree_cluster_tolerance, 0.3);
    nh_.param("tree_min_cluster_size", detector_config_.tree_min_cluster_size, 20);
    nh_.param("tree_max_cluster_size", detector_config_.tree_max_cluster_size, 5000);
    nh_.param("tree_linearity_threshold", detector_config_.tree_linearity_threshold, 0.6);
    nh_.param("tree_planarity_threshold", detector_config_.tree_planarity_threshold, 0.3);
    nh_.param("tree_min_height_spread", detector_config_.tree_min_height_spread, 0.5);
    nh_.param("tree_min_roundness", detector_config_.tree_min_roundness, 0.4);
    nh_.param("patchwork_sensor_height", detector_config_.patchwork_sensor_height, 1.0);
    nh_.param("patchwork_max_range", detector_config_.patchwork_max_range, 80.0);
    nh_.param("patchwork_min_range", detector_config_.patchwork_min_range, 0.5);
    nh_.param("accumulation_frame_count", accumulation_frame_count_, 3);
    if (accumulation_frame_count_ < 1) accumulation_frame_count_ = 0;  // 0 means no accumulation
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    last_odom_ = msg;
    has_odom_ = true;
  }

  void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    try {
      // accumulation_frame_count_ == 0: 逐帧检测
      if (accumulation_frame_count_ == 0) {
        processCloudFrame(msg);
        return;
      }

      // accumulation_frame_count_ > 0: 积累 N 帧
      cloud_buffer_.push_back(msg);
      frame_counter_++;

      if (frame_counter_ >= accumulation_frame_count_) {
        processAccumulatedClouds();
        cloud_buffer_.clear();
        frame_counter_ = 0;
      }
    } catch (const std::exception& e) {
      ROS_ERROR_THROTTLE(2.0, "[TreeDetectorNode] Exception in cloudCallback: %s", e.what());
      cloud_buffer_.clear();
      frame_counter_ = 0;
    } catch (...) {
      ROS_ERROR_THROTTLE(2.0, "[TreeDetectorNode] Unknown exception in cloudCallback");
      cloud_buffer_.clear();
      frame_counter_ = 0;
    }
  }

  // 单帧检测（逐帧模式）
  void processCloudFrame(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    try {
      std::vector<TreeInfo> trees;
      bool ok = detector_->detectFromROS(msg, trees);

      if (!ok || trees.empty()) {
        ROS_WARN_THROTTLE(5.0, "[TreeDetectorNode] No trees detected in point cloud");
        return;
      }

      publishTrees(trees, msg->header.stamp, msg->header.frame_id);
    } catch (const std::exception& e) {
      ROS_ERROR_THROTTLE(2.0, "[TreeDetectorNode] Exception in processCloudFrame: %s", e.what());
    } catch (...) {
      ROS_ERROR_THROTTLE(2.0, "[TreeDetectorNode] Unknown exception in processCloudFrame");
    }
  }

  // 积累多帧后检测
  void processAccumulatedClouds() {
    if (cloud_buffer_.empty()) return;

    try {
      // 拼接所有帧，直接传入 TreeDetector（内部会做体素降采样）
      TreeDetector::PointCloudPtr combined(new TreeDetector::PointCloudT());
      for (const auto& msg : cloud_buffer_) {
        pcl::PointCloud<pcl::PointXYZ> frame;
        pcl::fromROSMsg(*msg, frame);
        *combined += frame;
      }

      std::vector<TreeInfo> trees;
      bool ok = detector_->detect(combined, trees);

      if (!ok || trees.empty()) {
        ROS_WARN_THROTTLE(5.0, "[TreeDetectorNode] No trees detected (accumulated %zu frames, %zu points)",
                         cloud_buffer_.size(), combined->size());
        return;
      }

      // 用最新一帧的 header
      ros::Time stamp = cloud_buffer_.back()->header.stamp;
      std::string frame_id = cloud_buffer_.back()->header.frame_id;
      publishTrees(trees, stamp, frame_id);

      ROS_DEBUG("[TreeDetectorNode] Accumulated %zu frames (%zu points) -> %zu trees",
                cloud_buffer_.size(), combined->size(), trees.size());
    } catch (const std::exception& e) {
      ROS_ERROR_THROTTLE(2.0, "[TreeDetectorNode] Exception in processAccumulatedClouds: %s", e.what());
    } catch (...) {
      ROS_ERROR_THROTTLE(2.0, "[TreeDetectorNode] Unknown exception in processAccumulatedClouds");
    }
  }

  // 发布检测结果
  void publishTrees(const std::vector<TreeInfo>& trees,
                    const ros::Time& stamp, const std::string& frame_id) {
    // 点云已经是 world frame (pcl_render_node 输出的 cloud topic 是 local_map_pcd，frame_id=world)
    // 树干位置直接就是世界坐标，无需额外变换

    // 发布 TreeDetection 消息
    drone_detect_lidar::TreeDetection det_msg;
    det_msg.header.stamp = stamp;
    det_msg.header.frame_id = frame_id;
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
      det_msg.trees[i].planarity = trees[i].planarity;
      det_msg.trees[i].roundness = trees[i].roundness;
      det_msg.trees[i].confidence = trees[i].confidence;
    }

    tree_detection_pub_.publish(det_msg);

    // 发布可视化点云（每个树干中心一个点，world frame）
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl_cloud.reserve(trees.size());
    for (size_t i = 0; i < trees.size(); i++) {
      double tx = det_msg.trees[i].x;
      double ty = det_msg.trees[i].y;
      pcl_cloud.push_back(pcl::PointXYZ(tx, ty, trees[i].z_base + trees[i].height / 2.0));
    }
    sensor_msgs::PointCloud2 cloud_vis;
    pcl::toROSMsg(pcl_cloud, cloud_vis);
    cloud_vis.header.stamp = stamp;
    cloud_vis.header.frame_id = frame_id;
    tree_cloud_pub_.publish(cloud_vis);

    ROS_INFO("[TreeDetectorNode] Published %zu tree detections", trees.size());
  }
};

int main(int argc, char** argv) {
  try {
    ros::init(argc, argv, "tree_detector_node");
    ros::NodeHandle nh("~");
    TreeDetectorNode node(nh);
    ros::spin();
    return 0;
  } catch (const std::exception& e) {
    ROS_FATAL("tree_detector_node crashed: %s", e.what());
    return 1;
  } catch (...) {
    ROS_FATAL("tree_detector_node crashed with unknown exception");
    return 1;
  }
}
