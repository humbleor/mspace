#include <ros/ros.h>
#include "drone_detect_lidar/TreeRelativePose.h"
#include "drone_detect_lidar/TreeRelativePoseOptimized.h"

#include <gtsam/nonlinear/ISAM2.h>
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
 * @brief 基于 iSAM2 增量因子图的多机相对位姿优化节点
 *
 * 对比批量 Levenberg-Marquardt:
 *   - LM: 每次重建整个因子图，全部变量重新线性化，O(N³) 每轮
 *   - iSAM2: 持久化 Bayes tree，只对受影响变量增量更新，O(1) 每因子
 *
 * 流程:
 *   1. 滑窗平均各 UAV 对的测量值（降噪）
 *   2. 新测量作为 BetweenFactor 增量添加到 iSAM2
 *   3. iSAM2::update() — 仅重新线性化受影响 clique
 *   4. calculateEstimate() 获取当前最优估计
 *   5. 发布优化后的成对相对位姿
 */
class TreeGraphNode {
public:
  TreeGraphNode(ros::NodeHandle& nh)
    : nh_(nh), anchor_drone_id_(1), graph_initialized_(false), total_factors_added_(0) {

    loadParameters();

    // iSAM2 参数配置
    gtsam::ISAM2Params isam2_params;
    // 当线性化点偏移超过此阈值时触发重新线性化
    isam2_params.relinearizeThreshold = 0.01;
    // 每个 update 都检查是否需要重新线性化
    isam2_params.relinearizeSkip = 1;
    // 使用 QR 分解（CHOLESKY 更快但要求正定；QR 更稳健适合小图）
    isam2_params.factorization = gtsam::ISAM2Params::QR;
    isam2_ = gtsam::ISAM2(isam2_params);

    // 订阅各 UAV 的 tree_relative_pose_msg
    addSubscriber("self_relative_pose_msg");
    addSubscriber("uav2_relative_pose_msg");
    addSubscriber("uav3_relative_pose_msg");

    opt_pub_ = nh_.advertise<drone_detect_lidar::TreeRelativePoseOptimized>("optimized_pose", 10);

    optimize_timer_ = nh_.createTimer(ros::Duration(1.0 / optimize_freq_),
                                       &TreeGraphNode::optimizeTimerCallback, this);

    ROS_INFO("[TreeGraphNode] iSAM2 initialized, anchor=uav%d, freq=%.1f Hz, subs=%zu",
             anchor_drone_id_, optimize_freq_, subs_.size());
  }

private:
  void addSubscriber(const std::string& topic_param) {
    std::string topic_name;
    if (!nh_.getParam(topic_param, topic_name)) {
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

    // ---- 滑窗平均 ----
    struct AvgMeasurement {
      uint32_t src, dst;
      double dx, dy, yaw;
      double avg_cov;
      int count;
    };

    std::map<uint32_t, AvgMeasurement> averaged;
    for (auto it = recent_measurements_.begin(); it != recent_measurements_.end(); ++it) {
      uint32_t pair_id = it->first;
      const auto& deq = it->second;
      if (deq.empty()) continue;

      double sum_dx = 0, sum_dy = 0, sum_yaw_sin = 0, sum_yaw_cos = 0;
      double sum_cov = 0;
      int count = 0;
      uint32_t src = 0, dst = 0;

      for (size_t i = 0; i < deq.size(); ++i) {
        const auto& m = deq[i];
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

    known_drone_ids_.insert(anchor_drone_id_);

    // ---- 构建增量因子图 ----
    gtsam::NonlinearFactorGraph new_factors;
    gtsam::Values new_initial;

    // 首次：添加锚点先验因子
    if (!graph_initialized_) {
      gtsam::Pose2 anchor_pose(0.0, 0.0, 0.0);
      gtsam::SharedNoiseModel anchor_noise =
        gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(3) << 1e-6, 1e-6, 1e-6).finished());
      new_factors.add(gtsam::PriorFactor<gtsam::Pose2>(anchor_drone_id_, anchor_pose, anchor_noise));
      new_initial.insert(anchor_drone_id_, anchor_pose);
      initialized_drone_ids_.insert(anchor_drone_id_);
      graph_initialized_ = true;
    }

    // 为新出现的无人机添加初始估计（第一次出现时需要的初值）
    for (auto id : known_drone_ids_) {
      if (initialized_drone_ids_.find(id) == initialized_drone_ids_.end()) {
        // 新无人机：从已有估计中查找，或初始化为 (0,0,0)
        try {
          gtsam::Values current = isam2_.calculateEstimate();
          if (current.exists(id)) {
            new_initial.insert(id, current.at<gtsam::Pose2>(id));
          } else {
            new_initial.insert(id, gtsam::Pose2(0.0, 0.0, 0.0));
          }
        } catch (...) {
          new_initial.insert(id, gtsam::Pose2(0.0, 0.0, 0.0));
        }
        initialized_drone_ids_.insert(id);
      }
    }

    // 为当前滑窗平均后的每对测量添加 BetweenFactor
    // iSAM2 会累积历史测量（不删除旧的），形成信息丰富的因子图
    for (auto it = averaged.begin(); it != averaged.end(); ++it) {
      const AvgMeasurement& m = it->second;

      // 确保两个端点都已初始化
      if (initialized_drone_ids_.find(m.src) == initialized_drone_ids_.end()) {
        new_initial.insert(m.src, gtsam::Pose2(0.0, 0.0, 0.0));
        initialized_drone_ids_.insert(m.src);
      }
      if (initialized_drone_ids_.find(m.dst) == initialized_drone_ids_.end()) {
        new_initial.insert(m.dst, gtsam::Pose2(0.0, 0.0, 0.0));
        initialized_drone_ids_.insert(m.dst);
      }

      double noise_scale = std::sqrt(std::max(m.avg_cov, 1e-4)) * 10.0;
      gtsam::SharedNoiseModel between_noise =
        gtsam::noiseModel::Diagonal::Sigmas(
          (gtsam::Vector(3) << noise_scale, noise_scale, noise_scale * 5.0).finished());

      gtsam::Pose2 rel_pose(m.dx, m.dy, m.yaw);
      new_factors.add(gtsam::BetweenFactor<gtsam::Pose2>(m.src, m.dst, rel_pose, between_noise));
    }

    // ---- iSAM2 增量更新 ----
    try {
      total_factors_added_ += new_factors.size();
      gtsam::ISAM2Result result = isam2_.update(new_factors, new_initial);
      ROS_DEBUG("[TreeGraphNode] iSAM2 update: %zu factors added, variables=%zu, "
                "relinearized=%zu, total=%zu",
                new_factors.size(),
                isam2_.getLinearizationPoint().size(),
                result.getVariablesRelinearized(),
                total_factors_added_);
    } catch (const std::exception& e) {
      ROS_WARN("[TreeGraphNode] iSAM2 update failed: %s", e.what());
      return;
    }

    // ---- 获取当前最优估计 ----
    gtsam::Values result_estimate;
    try {
      result_estimate = isam2_.calculateEstimate();
    } catch (const std::exception& e) {
      ROS_WARN("[TreeGraphNode] Failed to calculate estimate: %s", e.what());
      return;
    }

    // ---- 发布优化后的两两位姿 ----
    for (auto it = averaged.begin(); it != averaged.end(); ++it) {
      const AvgMeasurement& m = it->second;
      if (!result_estimate.exists(m.src) || !result_estimate.exists(m.dst)) continue;

      gtsam::Pose2 pose_src = result_estimate.at<gtsam::Pose2>(m.src);
      gtsam::Pose2 pose_dst = result_estimate.at<gtsam::Pose2>(m.dst);
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

    ROS_INFO("[TreeGraphNode] iSAM2: %zu pairs published, %zu drones, %zu total factors",
             averaged.size(), known_drone_ids_.size(), total_factors_added_);
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

  // iSAM2 增量优化器（持久化，跨回调复用）
  gtsam::ISAM2 isam2_;
  bool graph_initialized_;
  size_t total_factors_added_;
  std::set<uint32_t> initialized_drone_ids_;

  std::map<uint32_t, std::deque<drone_detect_lidar::TreeRelativePose::ConstPtr>> recent_measurements_;
  std::set<uint32_t> known_drone_ids_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "tree_graph_node");
  ros::NodeHandle nh("~");
  TreeGraphNode node(nh);
  ros::spin();
  return 0;
}
