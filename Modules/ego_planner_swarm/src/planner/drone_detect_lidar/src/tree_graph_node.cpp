#include <ros/ros.h>
#include "drone_detect_lidar/TreeRelativePose.h"
#include "drone_detect_lidar/TreeRelativePoseOptimized.h"

#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/Values.h>

#include <map>
#include <set>
#include <vector>
#include <deque>
#include <cmath>

/**
 * @brief 基于因子图的多机相对位姿优化节点
 *
 * 订阅所有 UAV 的 TreeRelativePose 消息，构建因子图：
 * - 先验因子：固定 anchor UAV 为参考系 (0, 0, 0)
 * - 相对位姿因子：每对有足够匹配树的 UAV 对
 *
 * 求解后发布优化后的相对位姿
 */
class TreeGraphNode {
public:
  TreeGraphNode(ros::NodeHandle& nh)
    : nh_(nh), anchor_drone_id_(1) {

    loadParameters();

    // 订阅各 UAV 的 tree_relative_pose_msg
    // 使用 nh_.resolveName 配合 remap 实现灵活的话题绑定
    addSubscriber("self_relative_pose_msg");
    addSubscriber("uav2_relative_pose_msg");
    addSubscriber("uav3_relative_pose_msg");
    addSubscriber("uav4_relative_pose_msg");
    addSubscriber("uav5_relative_pose_msg");

    // 发布优化后的结果
    opt_pub_ = nh_.advertise<drone_detect_lidar::TreeRelativePoseOptimized>("optimized_pose", 10);

    // 定时器
    optimize_timer_ = nh_.createTimer(ros::Duration(1.0 / optimize_freq_),
                                       &TreeGraphNode::optimizeTimerCallback, this);

    ROS_INFO("[TreeGraphNode] Initialized, anchor=uav%d, freq=%.1f Hz, subs=%zu",
             anchor_drone_id_, optimize_freq_, subs_.size());
  }

private:
  void addSubscriber(const std::string& topic_param) {
    std::string topic_name;
    if (!nh_.getParam(topic_param, topic_name)) {
      // 未配置该话题，跳过
      return;
    }
    ros::Subscriber sub = nh_.subscribe<drone_detect_lidar::TreeRelativePose>(
      topic_name, 10,
      boost::bind(&TreeGraphNode::relativePoseCallback, this, _1));
    subs_.push_back(sub);
  }

  void relativePoseCallback(const drone_detect_lidar::TreeRelativePose::ConstPtr& msg) {
    boost::mutex::scoped_lock lock(mutex_);

    uint32_t src = msg->src_drone_id;
    uint32_t dst = msg->dst_drone_id;
    if (src == 0 || dst == 0) return;

    uint32_t pair_id = std::min(src, dst) * 1000 + std::max(src, dst);

    recent_measurements_[pair_id].push_back(msg);
    if (recent_measurements_[pair_id].size() > static_cast<size_t>(window_size_)) {
      recent_measurements_[pair_id].pop_front();
    }

    known_drone_ids_.insert(src);
    known_drone_ids_.insert(dst);
  }

  void optimizeTimerCallback(const ros::TimerEvent&) {
    boost::mutex::scoped_lock lock(mutex_);

    if (recent_measurements_.empty()) {
      ROS_DEBUG_THROTTLE(2.0, "[TreeGraphNode] No measurements yet");
      return;
    }

    // 对每对测量取平均（滑窗内）
    struct AvgMeasurement {
      uint32_t src, dst;
      double dx, dy, yaw;
      double avg_cov;
      int count;
    };

    std::map<uint32_t, AvgMeasurement> averaged;
    for (std::map<uint32_t, std::deque<drone_detect_lidar::TreeRelativePose::ConstPtr> >::iterator
         it = recent_measurements_.begin(); it != recent_measurements_.end(); ++it) {
      uint32_t pair_id = it->first;
      const std::deque<drone_detect_lidar::TreeRelativePose::ConstPtr>& deq = it->second;
      if (deq.empty()) continue;

      double sum_dx = 0, sum_dy = 0, sum_yaw_sin = 0, sum_yaw_cos = 0;
      double sum_cov = 0;
      int count = 0;
      uint32_t src = 0, dst = 0;

      for (size_t i = 0; i < deq.size(); ++i) {
        const drone_detect_lidar::TreeRelativePose::ConstPtr& m = deq[i];
        if (m->num_matched_trees < min_common_trees_) continue;
        src = m->src_drone_id;
        dst = m->dst_drone_id;
        sum_dx += m->dx;
        sum_dy += m->dy;
        sum_yaw_sin += std::sin(m->yaw);
        sum_yaw_cos += std::cos(m->yaw);
        double avg_cov = (m->covariance[0] + m->covariance[4] + m->covariance[8]) / 3.0;
        sum_cov += avg_cov;
        count++;
      }

      if (count < 1) continue;

      AvgMeasurement avg;
      avg.src = src;
      avg.dst = dst;
      avg.dx = sum_dx / count;
      avg.dy = sum_dy / count;
      avg.yaw = std::atan2(sum_yaw_sin / count, sum_yaw_cos / count);
      avg.avg_cov = sum_cov / count;
      avg.count = count;
      averaged[pair_id] = avg;
    }

    if (averaged.empty()) return;

    // 确保锚点存在
    known_drone_ids_.insert(anchor_drone_id_);

    // 构建因子图
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial;

    // 锚点先验
    gtsam::Pose2 anchor_pose(0.0, 0.0, 0.0);
    gtsam::SharedNoiseModel anchor_noise =
      gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(3) << 1e-6, 1e-6, 1e-6).finished());
    graph.add(gtsam::PriorFactor<gtsam::Pose2>(anchor_drone_id_, anchor_pose, anchor_noise));
    initial.insert(anchor_drone_id_, anchor_pose);

    // 初始化其他无人机
    for (std::set<uint32_t>::iterator it = known_drone_ids_.begin();
         it != known_drone_ids_.end(); ++it) {
      if (*it == static_cast<uint32_t>(anchor_drone_id_)) continue;
      initial.insert(*it, gtsam::Pose2(0.0, 0.0, 0.0));
    }

    // 添加相对位姿因子
    for (std::map<uint32_t, AvgMeasurement>::iterator it = averaged.begin();
         it != averaged.end(); ++it) {
      const AvgMeasurement& m = it->second;
      double noise_scale = std::sqrt(std::max(m.avg_cov, 1e-4)) * 10.0;
      gtsam::SharedNoiseModel between_noise =
        gtsam::noiseModel::Diagonal::Sigmas(
          (gtsam::Vector(3) << noise_scale, noise_scale, noise_scale * 5.0).finished());

      gtsam::Pose2 rel_pose(m.dx, m.dy, m.yaw);

      if (!initial.exists(m.src)) {
        initial.insert(m.src, gtsam::Pose2(0.0, 0.0, 0.0));
      }
      if (!initial.exists(m.dst)) {
        initial.insert(m.dst, gtsam::Pose2(0.0, 0.0, 0.0));
      }

      graph.add(gtsam::BetweenFactor<gtsam::Pose2>(m.src, m.dst, rel_pose, between_noise));
    }

    // 优化
    gtsam::LevenbergMarquardtParams params;
    params.setMaxIterations(50);
    params.setRelativeErrorTol(1e-5);

    gtsam::Values result;
    try {
      gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial, params);
      result = optimizer.optimize();
    } catch (const std::exception& e) {
      ROS_WARN("[TreeGraphNode] Optimization failed: %s", e.what());
      return;
    }

    // 发布优化后的两两位姿
    for (std::map<uint32_t, AvgMeasurement>::iterator it = averaged.begin();
         it != averaged.end(); ++it) {
      const AvgMeasurement& m = it->second;
      if (!result.exists(m.src) || !result.exists(m.dst)) continue;

      gtsam::Pose2 pose_src = result.at<gtsam::Pose2>(m.src);
      gtsam::Pose2 pose_dst = result.at<gtsam::Pose2>(m.dst);
      gtsam::Pose2 rel_opt = pose_src.between(pose_dst);

      drone_detect_lidar::TreeRelativePoseOptimized opt_msg;
      opt_msg.header.stamp = ros::Time::now();
      opt_msg.src_drone_id = m.src;
      opt_msg.dst_drone_id = m.dst;
      opt_msg.dx = rel_opt.x();
      opt_msg.dy = rel_opt.y();
      opt_msg.yaw = rel_opt.theta();
      opt_msg.rms_residual = m.avg_cov;
      opt_msg.num_matched_trees = m.count;

      opt_pub_.publish(opt_msg);
    }

    ROS_INFO("[TreeGraphNode] Optimized %zu pairs, %zu drones",
             averaged.size(), known_drone_ids_.size());
  }

  void loadParameters() {
    nh_.param("anchor_drone_id", anchor_drone_id_, 1);
    nh_.param("optimize_freq", optimize_freq_, 5.0);
    nh_.param("min_common_trees", min_common_trees_, 3);
    nh_.param("window_size", window_size_, 10);
  }

  ros::NodeHandle nh_;
  std::vector<ros::Subscriber> subs_;
  ros::Publisher opt_pub_;
  ros::Timer optimize_timer_;
  boost::mutex mutex_;

  int anchor_drone_id_;
  double optimize_freq_;
  int min_common_trees_;
  int window_size_;

  std::map<uint32_t, std::deque<drone_detect_lidar::TreeRelativePose::ConstPtr> > recent_measurements_;
  std::set<uint32_t> known_drone_ids_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "tree_graph_node");
  ros::NodeHandle nh("~");
  TreeGraphNode node(nh);
  ros::spin();
  return 0;
}
