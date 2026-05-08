#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/Odometry.h>
#include "drone_detect_lidar/TreeDetection.h"
#include "drone_detect_lidar/TreeRelativePose.h"

#include <Eigen/Eigen>
#include <cmath>
#include <unordered_map>

/**
 * @brief 基于三角形哈希匹配的多机相对定位节点
 *
 * 流程 (参考 HashReg):
 *   第0步: 距离过滤 — 两机间距超过阈值则跳过
 *   第1步: 收集自身 + 邻居 TreeDetection
 *   第2步: 三角形哈希匹配 + 投票验证 — 替代距离贪心匹配
 *     2a: 构造非退化三角形 (三边长排序 + 等腰/等边剔除)
 *     2b: 自身三角形建哈希表，邻居三角形粗糙匹配 (27 voxel 搜索)
 *     2c: 投票验证 — 每个候选对求变换，选一致性最好的
 *     2d: 用最佳变换收集所有对应关系
 *   第3步: 数量检查 — 最少共有树数量
 *   第4步: 2D SVD 闭式解 — 求 (dx, dy, yaw)
 *   第5步: 质量检查
 *   第6步: 发布 tree_pose_error
 */

struct Triangle2D {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector2d A, B, C;
  Eigen::Vector2d center;
  double a, b, c;  // sorted: a <= b <= c
  int idx[3];
};

struct TriMatch {
  int self_idx;
  int neighbor_idx;
};

using TriKey = std::tuple<int, int, int>;

struct TriKeyHash {
  size_t operator()(const TriKey& k) const {
    return (size_t)((std::get<0>(k) * 73856093) ^
                    (std::get<1>(k) * 19349663) ^
                    (std::get<2>(k) * 83492791));
  }
};

class TreeLocNode {
public:
  TreeLocNode(ros::NodeHandle& nh)
    : nh_(nh), has_self_trees_(false), has_neighbor_trees_(false),
      has_self_odom_(false), has_neighbor_odom_(false), last_fitness_(1e6) {

    loadParameters();

    self_tree_sub_ = nh_.subscribe("self_tree_detection", 10,
                                   &TreeLocNode::selfTreeCallback, this);
    neighbor_tree_sub_ = nh_.subscribe("neighbor_tree_detection", 10,
                                       &TreeLocNode::neighborTreeCallback, this);
    self_odom_sub_ = nh_.subscribe("self_odom", 100,
                                   &TreeLocNode::selfOdomCallback, this);
    neighbor_odom_sub_ = nh_.subscribe("neighbor_odom", 100,
                                       &TreeLocNode::neighborOdomCallback, this);

    tree_pose_error_pub_ = nh_.advertise<geometry_msgs::Vector3>("tree_pose_error", 10);
    tree_relative_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("tree_relative_pose", 10);
    tree_relative_pose_msg_pub_ = nh_.advertise<drone_detect_lidar::TreeRelativePose>("tree_relative_pose_msg", 10);
    matched_tree_pairs_pub_ = nh_.advertise<sensor_msgs::PointCloud>("matched_tree_pairs", 10);

    compute_timer_ = nh_.createTimer(ros::Duration(1.0 / compute_frequency_),
                                     &TreeLocNode::computeTimerCallback, this);

    ROS_INFO("[TreeLocNode] Initialized for drone_%d", drone_id_);
  }

private:
  ros::NodeHandle& nh_;
  ros::Subscriber self_tree_sub_;
  ros::Subscriber neighbor_tree_sub_;
  ros::Subscriber self_odom_sub_;
  ros::Subscriber neighbor_odom_sub_;
  ros::Publisher tree_pose_error_pub_;
  ros::Publisher tree_relative_pose_pub_;
  ros::Publisher tree_relative_pose_msg_pub_;
  ros::Publisher matched_tree_pairs_pub_;
  ros::Timer compute_timer_;

  int drone_id_;
  double compute_frequency_;
  double drone_dist_threshold_;
  int min_common_trees_;
  double max_translation_;
  double max_yaw_deg_;
  double max_match_residual_;

  // Triangle matching parameters (HashReg style)
  double triangle_min_side_;
  double triangle_max_side_;
  double isosceles_threshold_;
  double equilateral_threshold_;
  double rough_dis_ratio_;
  double geom_verify_dist_;

  bool has_self_trees_, has_neighbor_trees_;
  bool has_self_odom_, has_neighbor_odom_;
  drone_detect_lidar::TreeDetection::ConstPtr self_trees_;
  drone_detect_lidar::TreeDetection::ConstPtr neighbor_trees_;
  nav_msgs::Odometry::ConstPtr self_odom_;
  nav_msgs::Odometry::ConstPtr neighbor_odom_;
  double last_fitness_;

  void loadParameters() {
    nh_.param("drone_id", drone_id_, 0);
    nh_.param("compute_frequency", compute_frequency_, 5.0);
    nh_.param("drone_dist_threshold", drone_dist_threshold_, 15.0);
    nh_.param("min_common_trees", min_common_trees_, 3);
    nh_.param("max_translation", max_translation_, 3.0);
    nh_.param("max_yaw_deg", max_yaw_deg_, 20.0);
    nh_.param("max_match_residual", max_match_residual_, 0.5);

    nh_.param("triangle_min_side", triangle_min_side_, 0.3);
    nh_.param("triangle_max_side", triangle_max_side_, 20.0);
    nh_.param("isosceles_threshold", isosceles_threshold_, 0.1);
    nh_.param("equilateral_threshold", equilateral_threshold_, 0.15);
    nh_.param("rough_dis_ratio", rough_dis_ratio_, 0.05);
    nh_.param("geom_verify_dist", geom_verify_dist_, 0.3);
  }

  void selfTreeCallback(const drone_detect_lidar::TreeDetection::ConstPtr& msg) {
    self_trees_ = msg;
    has_self_trees_ = true;
  }

  void neighborTreeCallback(const drone_detect_lidar::TreeDetection::ConstPtr& msg) {
    neighbor_trees_ = msg;
    has_neighbor_trees_ = true;
  }

  void selfOdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    self_odom_ = msg;
    has_self_odom_ = true;
  }

  void neighborOdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    neighbor_odom_ = msg;
    has_neighbor_odom_ = true;
  }

  // ===== Triangle construction (HashReg build_stdesc style) =====
  std::vector<Triangle2D, Eigen::aligned_allocator<Triangle2D>>
  buildTriangles(const std::vector<drone_detect_lidar::Tree>& trees) {
    std::vector<Triangle2D, Eigen::aligned_allocator<Triangle2D>> tris;
    size_t N = trees.size();
    if (N < 3) return tris;

    // Build KDTree for KNN
    std::vector<Eigen::Vector2d> pts(N);
    for (size_t i = 0; i < N; i++) {
      pts[i] << trees[i].x, trees[i].y;
    }

    const int K = std::min((int)N, 10);

    for (size_t i = 0; i < N; i++) {
      // Find K nearest neighbors
      std::vector<std::pair<double, int>> dists;
      for (size_t j = 0; j < N; j++) {
        if (i == j) continue;
        double d = (pts[i] - pts[j]).norm();
        dists.push_back({d, (int)j});
      }
      std::sort(dists.begin(), dists.end());
      if ((int)dists.size() < 2) continue;

      int nn = std::min(K - 1, (int)dists.size());

      // Form triangles with pairs of neighbors
      for (int m = 0; m < nn - 1; m++) {
        for (int n_idx = m + 1; n_idx < nn; n_idx++) {
          int j = dists[m].second;
          int k = dists[n_idx].second;

          Eigen::Vector2d A = pts[i];
          Eigen::Vector2d B = pts[j];
          Eigen::Vector2d C = pts[k];

          // Side lengths
          double d_ab = (A - B).norm();
          double d_ac = (A - C).norm();
          double d_bc = (B - C).norm();

          // Sort: a <= b <= c
          double sides[3] = {d_ab, d_ac, d_bc};

          // Simple sort (only sides needed, vertices always {i,j,k})
          if (sides[0] > sides[1]) std::swap(sides[0], sides[1]);
          if (sides[1] > sides[2]) std::swap(sides[1], sides[2]);
          if (sides[0] > sides[1]) std::swap(sides[0], sides[1]);

          double a = sides[0], b = sides[1], c = sides[2];

          // Filter: side length range
          if (a < triangle_min_side_ || c > triangle_max_side_) continue;

          // Filter: isosceles / equilateral (HashReg style)
          if (std::abs(a - b) < isosceles_threshold_) continue;
          if (std::abs(b - c) < isosceles_threshold_) continue;
          if (std::abs(a - c) < equilateral_threshold_) continue;

          // Determine vertices A, B, C corresponding to sides a, b, c
          // After sorting, we need to find which original vertices form the triangle
          // The three vertices are always {i, j, k}
          Triangle2D tri;
          tri.A = pts[i];
          tri.B = pts[j];
          tri.C = pts[k];
          tri.a = a;
          tri.b = b;
          tri.c = c;
          tri.center = (tri.A + tri.B + tri.C) / 3.0;
          tri.idx[0] = i;
          tri.idx[1] = j;
          tri.idx[2] = k;

          tris.push_back(tri);
        }
      }
    }

    // Deduplicate: remove duplicate triangles (same 3 indices)
    std::vector<Triangle2D, Eigen::aligned_allocator<Triangle2D>> unique_tris;
    std::set<std::tuple<int,int,int>> seen;
    for (auto& tri : tris) {
      int idx[3] = {tri.idx[0], tri.idx[1], tri.idx[2]};
      std::sort(idx, idx+3);
      auto key = std::make_tuple(idx[0], idx[1], idx[2]);
      if (seen.find(key) == seen.end()) {
        seen.insert(key);
        unique_tris.push_back(tri);
      }
    }

    return unique_tris;
  }

  // ===== Triangle solver (HashReg triangle_solver, 2D version) =====
  std::pair<Eigen::Vector2d, Eigen::Matrix2d>
  triangleSolver(const Triangle2D& src, const Triangle2D& dst) {
    // Centerized vertices
    Eigen::Matrix2d src_mat, dst_mat;
    src_mat.col(0) = src.A - src.center;
    src_mat.col(1) = src.B - src.center;
    dst_mat.col(0) = dst.A - dst.center;
    dst_mat.col(1) = dst.B - dst.center;

    Eigen::Matrix2d H = src_mat * dst_mat.transpose();

    Eigen::JacobiSVD<Eigen::Matrix2d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix2d U = svd.matrixU();
    Eigen::Matrix2d V = svd.matrixV();

    Eigen::Matrix2d R = V * U.transpose();
    if (R.determinant() < 0) {
      V.col(1) *= -1.0;
      R = V * U.transpose();
    }

    Eigen::Vector2d t = dst.center - R * src.center;
    return {t, R};
  }

  // ===== 27 voxel round vectors =====
  std::vector<Eigen::Vector3i> getVoxelRound() {
    std::vector<Eigen::Vector3i> voxels;
    for (int dx = -1; dx <= 1; dx++)
      for (int dy = -1; dy <= 1; dy++)
        for (int dz = -1; dz <= 1; dz++)
          voxels.push_back(Eigen::Vector3i(dx, dy, dz));
    return voxels;
  }

  TriKey makeKey(double a, double b, double c) {
    return std::make_tuple(
      (int)std::round(a * 1000),
      (int)std::round(b * 1000),
      (int)std::round(c * 1000));
  }

  double edgeDist(const Triangle2D& t1, const Triangle2D& t2) {
    double da = t1.a - t2.a, db = t1.b - t2.b, dc = t1.c - t2.c;
    return std::sqrt(da*da + db*db + dc*dc);
  }

  void computeTimerCallback(const ros::TimerEvent&) {
    if (!has_self_trees_ || !has_neighbor_trees_ || !has_self_odom_ || !has_neighbor_odom_) {
      ROS_DEBUG_THROTTLE(1.0, "[TreeLocNode] Waiting for data");
      return;
    }

    // ===== Step 0: Distance filter =====
    Eigen::Vector3d p_self(
      self_odom_->pose.pose.position.x,
      self_odom_->pose.pose.position.y,
      self_odom_->pose.pose.position.z);
    Eigen::Vector3d p_neighbor(
      neighbor_odom_->pose.pose.position.x,
      neighbor_odom_->pose.pose.position.y,
      neighbor_odom_->pose.pose.position.z);

    double drone_dist = (p_self - p_neighbor).norm();
    if (drone_dist > drone_dist_threshold_) {
      ROS_DEBUG_THROTTLE(2.0, "[TreeLocNode] Drones too far: %.1f m", drone_dist);
      return;
    }

    // ===== Step 1: Collect trees =====
    const auto& self_trees = self_trees_->trees;
    const auto& neighbor_trees = neighbor_trees_->trees;

    if (self_trees.size() < 3 || neighbor_trees.size() < 3) {
      ROS_DEBUG_THROTTLE(2.0, "[TreeLocNode] Not enough trees: self=%zu, neighbor=%zu",
                         self_trees.size(), neighbor_trees.size());
      return;
    }

    // ===== Step 2: Triangle hash matching + voting verification =====

    // 2a. Build triangles
    auto self_tris = buildTriangles(self_trees);
    auto neighbor_tris = buildTriangles(neighbor_trees);

    if (self_tris.empty() || neighbor_tris.empty()) {
      ROS_DEBUG_THROTTLE(2.0, "[TreeLocNode] No valid triangles: self=%zu, neighbor=%zu",
                         self_tris.size(), neighbor_tris.size());
      return;
    }

    ROS_INFO("[TreeLocNode] Triangles: self=%zu, neighbor=%zu",
             self_tris.size(), neighbor_tris.size());

    // 2b. Build hash table for self triangles
    std::unordered_map<TriKey, std::vector<int>, TriKeyHash> self_tri_map;
    for (size_t i = 0; i < self_tris.size(); i++) {
      TriKey key = makeKey(self_tris[i].a, self_tris[i].b, self_tris[i].c);
      self_tri_map[key].push_back((int)i);
    }

    // 2b. Rough matching: neighbor triangles search in 27 voxels
    auto voxel_round = getVoxelRound();
    std::vector<TriMatch> candidate_matches;

    for (size_t ni = 0; ni < neighbor_tris.size(); ni++) {
      TriKey key = makeKey(neighbor_tris[ni].a, neighbor_tris[ni].b, neighbor_tris[ni].c);
      double side_norm = neighbor_tris[ni].a + neighbor_tris[ni].b + neighbor_tris[ni].c;
      double dis_threshold = side_norm * rough_dis_ratio_;

      for (auto& v : voxel_round) {
        TriKey search_key(
          std::get<0>(key) + v[0],
          std::get<1>(key) + v[1],
          std::get<2>(key) + v[2]);
        auto it = self_tri_map.find(search_key);
        if (it == self_tri_map.end()) continue;

        for (int si : it->second) {
          if (edgeDist(self_tris[si], neighbor_tris[ni]) < dis_threshold) {
            candidate_matches.push_back({si, (int)ni});
          }
        }
      }
    }

    if (candidate_matches.empty()) {
      ROS_DEBUG_THROTTLE(2.0, "[TreeLocNode] No triangle matches found");
      return;
    }

    ROS_INFO("[TreeLocNode] Candidate triangle pairs: %zu", candidate_matches.size());

    // 2c. Voting verification (HashReg candidate_frames_verify)
    int best_vote = 0;
    Eigen::Vector2d t_best(0, 0);
    Eigen::Matrix2d R_best = Eigen::Matrix2d::Identity();

    // Pre-build point list for fast verification
    std::vector<Eigen::Vector2d> self_pts(self_trees.size());
    std::vector<Eigen::Vector2d> neighbor_pts(neighbor_trees.size());
    for (size_t i = 0; i < self_trees.size(); i++)
      self_pts[i] << self_trees[i].x, self_trees[i].y;
    for (size_t i = 0; i < neighbor_trees.size(); i++)
      neighbor_pts[i] << neighbor_trees[i].x, neighbor_trees[i].y;

    for (size_t m = 0; m < candidate_matches.size(); m++) {
      std::pair<Eigen::Vector2d, Eigen::Matrix2d> result =
        triangleSolver(self_tris[candidate_matches[m].self_idx],
                        neighbor_tris[candidate_matches[m].neighbor_idx]);
      Eigen::Vector2d t = result.first;
      Eigen::Matrix2d R = result.second;

      int vote = 0;
      for (size_t i = 0; i < self_pts.size(); i++) {
        Eigen::Vector2d transformed = R * self_pts[i] + t;
        for (size_t j = 0; j < neighbor_pts.size(); j++) {
          if ((neighbor_pts[j] - transformed).norm() < geom_verify_dist_) {
            vote++;
            break;
          }
        }
      }

      if (vote > best_vote) {
        best_vote = vote;
        t_best = t;
        R_best = R;
      }
    }

    if (best_vote < 3) {
      ROS_DEBUG_THROTTLE(2.0, "[TreeLocNode] Triangle vote too low: %d", best_vote);
      return;
    }

    ROS_INFO("[TreeLocNode] Best triangle match vote: %d", best_vote);

    // 2d. Collect correspondences using best (t, R)
    struct MatchedPair {
      int self_idx;
      int neighbor_idx;
      double dist;
    };
    std::vector<MatchedPair> matched_pairs;
    std::vector<bool> neighbor_used(neighbor_trees.size(), false);

    for (size_t i = 0; i < self_pts.size(); i++) {
      Eigen::Vector2d transformed = R_best * self_pts[i] + t_best;
      double best_dist = geom_verify_dist_;
      int best_j = -1;
      for (size_t j = 0; j < neighbor_pts.size(); j++) {
        if (neighbor_used[j]) continue;
        double d = (neighbor_pts[j] - transformed).norm();
        if (d < best_dist) {
          best_dist = d;
          best_j = (int)j;
        }
      }
      if (best_j >= 0) {
        matched_pairs.push_back({(int)i, best_j, best_dist});
        neighbor_used[best_j] = true;
      }
    }

    // ===== Step 3: Quantity check =====
    if (static_cast<int>(matched_pairs.size()) < min_common_trees_) {
      ROS_WARN_THROTTLE(2.0, "[TreeLocNode] Not enough common trees: %zu (need %d)",
                        matched_pairs.size(), min_common_trees_);
      return;
    }

    // ===== Step 4: Final 2D SVD =====
    int n = matched_pairs.size();
    Eigen::MatrixXd P(2, n), Q(2, n);

    for (int i = 0; i < n; i++) {
      P(0, i) = self_trees[matched_pairs[i].self_idx].x;
      P(1, i) = self_trees[matched_pairs[i].self_idx].y;
      Q(0, i) = neighbor_trees[matched_pairs[i].neighbor_idx].x;
      Q(1, i) = neighbor_trees[matched_pairs[i].neighbor_idx].y;
    }

    Eigen::Vector2d mu_P = P.rowwise().mean();
    Eigen::Vector2d mu_Q = Q.rowwise().mean();

    Eigen::MatrixXd P_c = P.colwise() - mu_P;
    Eigen::MatrixXd Q_c = Q.colwise() - mu_Q;

    Eigen::Matrix2d H = P_c * Q_c.transpose();

    Eigen::JacobiSVD<Eigen::Matrix2d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix2d U = svd.matrixU();
    Eigen::Matrix2d V = svd.matrixV();

    Eigen::Matrix2d R = V * U.transpose();
    if (R.determinant() < 0) {
      V.col(1) *= -1.0;
      R = V * U.transpose();
    }

    Eigen::Vector2d t = mu_Q - R * mu_P;
    double yaw = std::atan2(R(1, 0), R(0, 0));

    // ===== Step 5: Quality checks =====
    double trans_norm = t.norm();
    double yaw_deg = std::abs(yaw) * 180.0 / M_PI;

    double rms_residual = 0;
    for (int i = 0; i < n; i++) {
      Eigen::Vector2d p_transformed = R * P.col(i) + t;
      Eigen::Vector2d diff = Q.col(i) - p_transformed;
      rms_residual += diff.squaredNorm();
    }
    rms_residual = std::sqrt(rms_residual / n);

    if (trans_norm > max_translation_) {
      ROS_WARN("[TreeLocNode] Translation too large: %.3f m", trans_norm);
      return;
    }
    if (yaw_deg > max_yaw_deg_) {
      ROS_WARN("[TreeLocNode] Yaw too large: %.1f deg", yaw_deg);
      return;
    }
    if (rms_residual > max_match_residual_) {
      ROS_WARN("[TreeLocNode] RMS residual too large: %.3f m", rms_residual);
      return;
    }

    // ===== Step 6: Publish =====
    last_fitness_ = rms_residual;

    geometry_msgs::Vector3 error_msg;
    error_msg.x = t(0);
    error_msg.y = t(1);
    error_msg.z = yaw;
    tree_pose_error_pub_.publish(error_msg);

    drone_detect_lidar::TreeRelativePose rel_msg;
    rel_msg.header.stamp = ros::Time::now();
    rel_msg.header.frame_id = self_odom_->header.frame_id;
    rel_msg.src_drone_id = drone_id_;
    rel_msg.dst_drone_id = neighbor_trees_->drone_id;
    rel_msg.dx = t(0);
    rel_msg.dy = t(1);
    rel_msg.yaw = yaw;
    double sigma2 = (rms_residual * rms_residual) / std::max(1, n);
    rel_msg.covariance[0] = sigma2;
    rel_msg.covariance[4] = sigma2;
    rel_msg.covariance[8] = sigma2;
    rel_msg.rms_residual = rms_residual;
    rel_msg.num_matched_trees = n;
    tree_relative_pose_msg_pub_.publish(rel_msg);

    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = self_odom_->header.frame_id;
    pose_msg.pose.position.x = trans_norm;
    pose_msg.pose.orientation.w = std::cos(yaw / 2.0);
    pose_msg.pose.orientation.z = std::sin(yaw / 2.0);
    tree_relative_pose_pub_.publish(pose_msg);

    sensor_msgs::PointCloud pair_vis;
    pair_vis.header.stamp = ros::Time::now();
    pair_vis.header.frame_id = self_odom_->header.frame_id;
    pair_vis.points.resize(n * 2);
    for (int i = 0; i < n; i++) {
      pair_vis.points[i * 2].x = P(0, i);
      pair_vis.points[i * 2].y = P(1, i);
      pair_vis.points[i * 2].z = 0.0;
      pair_vis.points[i * 2 + 1].x = Q(0, i);
      pair_vis.points[i * 2 + 1].y = Q(1, i);
      pair_vis.points[i * 2 + 1].z = 0.0;
    }
    matched_tree_pairs_pub_.publish(pair_vis);

    ROS_INFO("[TreeLocNode] drone_dist=%.1fm, matched=%d, trans=(%.3f, %.3f), "
             "yaw=%.2f deg, rms=%.3fm",
             drone_dist, n, t(0), t(1), yaw_deg, rms_residual);
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "tree_loc_node");
  ros::NodeHandle nh("~");
  TreeLocNode node(nh);
  ros::spin();
  return 0;
}
