#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include "drone_detect_lidar/TreeDetection.h"
#include "drone_detect_lidar/TreeRelativePose.h"
#include "drone_detect_lidar/triangle_matcher.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <Eigen/Eigen>
#include <cmath>
#include <unordered_map>
#include <mutex>

/**
 * @brief 基于三角形哈希匹配的多机相对定位节点
 *
 * 流程 (参考 HashReg):
 *   第0步: 距离过滤 — 两机间距超过阈值则跳过
 *   第1步: 收集自身 + 邻居 TreeDetection
 *   第2步: 三角形哈希匹配 + 投票验证
 *     2a: 构造非退化三角形 (三边长排序 + 等腰/等边剔除)
 *     2b: 自身三角形建哈希表，邻居三角形粗糙匹配 (11^3 voxel 搜索)
 *     2c: 投票验证 — 每个候选对求变换，选一致性最好的
 *     2d: 用最佳变换收集所有对应关系
 *   第3步: 数量检查 — 最少共有树数量
 *   第4步: 2D SVD 闭式解 — 求 (dx, dy, yaw)
 *   第5步: 质量检查
 *   第6步: 发布 tree_pose_error
 */

using drone_detect_lidar::Triangle2D;
using drone_detect_lidar::TriMatch;
using drone_detect_lidar::TriKey;
using drone_detect_lidar::TriKeyHash;
using drone_detect_lidar::TriangleMatchConfig;

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
    matched_tree_pairs_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("matched_tree_pairs", 10);

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
  TriangleMatchConfig tri_config_;

  std::mutex data_mutex_;
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
    nh_.param("drone_dist_threshold", drone_dist_threshold_, 10.0);
    nh_.param("min_common_trees", min_common_trees_, 3);
    nh_.param("max_translation", max_translation_, 3.0);
    nh_.param("max_yaw_deg", max_yaw_deg_, 20.0);
    nh_.param("max_match_residual", max_match_residual_, 0.5);

    nh_.param("triangle_min_side", tri_config_.triangle_min_side, tri_config_.triangle_min_side);
    nh_.param("triangle_max_side", tri_config_.triangle_max_side, tri_config_.triangle_max_side);
    nh_.param("isosceles_threshold", tri_config_.isosceles_threshold, tri_config_.isosceles_threshold);
    nh_.param("equilateral_threshold", tri_config_.equilateral_threshold, tri_config_.equilateral_threshold);
    nh_.param("rough_dis_ratio", tri_config_.rough_dis_ratio, tri_config_.rough_dis_ratio);
    nh_.param("geom_verify_dist", tri_config_.geom_verify_dist, tri_config_.geom_verify_dist);
  }

  void selfTreeCallback(const drone_detect_lidar::TreeDetection::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    self_trees_ = msg;
    has_self_trees_ = true;
  }

  void neighborTreeCallback(const drone_detect_lidar::TreeDetection::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    neighbor_trees_ = msg;
    has_neighbor_trees_ = true;
  }

  void selfOdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    self_odom_ = msg;
    has_self_odom_ = true;
  }

  void neighborOdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    neighbor_odom_ = msg;
    has_neighbor_odom_ = true;
  }

  // ===== Triangle construction (HashReg build_stdesc style) =====
  std::vector<Triangle2D, Eigen::aligned_allocator<Triangle2D>>
  buildTriangles(const std::vector<drone_detect_lidar::Tree>& trees) {
    std::vector<Triangle2D, Eigen::aligned_allocator<Triangle2D>> tris;
    size_t N = trees.size();
    if (N < 3) return tris;

    // Build PCL KDTree for efficient KNN
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->points.resize(N);
    for (size_t i = 0; i < N; i++) {
      cloud->points[i].x = trees[i].x;
      cloud->points[i].y = trees[i].y;
      cloud->points[i].z = 0.0;
    }
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);

    const int K = std::min((int)N, 10);

    std::set<std::tuple<int,int,int>> seen;

    for (size_t i = 0; i < N; i++) {
      // Find K nearest neighbors via KDTree
      std::vector<int> nn_indices(K);
      std::vector<float> nn_sqr_dists(K);
      int found = kdtree.nearestKSearch(cloud->points[i], K, nn_indices, nn_sqr_dists);
      if (found < 3) continue;

      // Form triangles with pairs of neighbors (skip index 0 = self)
      for (int m = 1; m < found - 1; m++) {
        for (int n_idx = m + 1; n_idx < found; n_idx++) {
          int j = nn_indices[m];
          int k = nn_indices[n_idx];

          Eigen::Vector2d A = Eigen::Vector2d(trees[i].x, trees[i].y);
          Eigen::Vector2d B = Eigen::Vector2d(trees[j].x, trees[j].y);
          Eigen::Vector2d C = Eigen::Vector2d(trees[k].x, trees[k].y);

          // Side lengths
          double d_ab = (A - B).norm();
          double d_ac = (A - C).norm();
          double d_bc = (B - C).norm();

          // Sort: a <= b <= c
          double sides[3] = {d_ab, d_ac, d_bc};
          if (sides[0] > sides[1]) std::swap(sides[0], sides[1]);
          if (sides[1] > sides[2]) std::swap(sides[1], sides[2]);
          if (sides[0] > sides[1]) std::swap(sides[0], sides[1]);

          double a = sides[0], b = sides[1], c = sides[2];

          // Filter: side length range
          if (a < tri_config_.triangle_min_side || c > tri_config_.triangle_max_side) continue;

          // Filter: isosceles / equilateral (HashReg style)
          if (std::abs(a - b) < tri_config_.isosceles_threshold) continue;
          if (std::abs(b - c) < tri_config_.isosceles_threshold) continue;
          if (std::abs(a - c) < tri_config_.equilateral_threshold) continue;

          // Deduplicate
          int idx[3] = {static_cast<int>(i), j, k};
          std::sort(idx, idx + 3);
          auto key = std::make_tuple(idx[0], idx[1], idx[2]);
          if (seen.count(key)) continue;
          seen.insert(key);

          Triangle2D tri;
          tri.A = A;
          tri.B = B;
          tri.C = C;
          tri.a = a;
          tri.b = b;
          tri.c = c;
          tri.center = (A + B + C) / 3.0;
          tri.idx[0] = i;
          tri.idx[1] = j;
          tri.idx[2] = k;

          tris.push_back(tri);
        }
      }
    }

    return tris;
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

  // ===== Voxel search round vectors =====
  // Search ±voxel_radius bins in each dimension (bin = 1mm for edge length hash key).
  // Forest uses ±8 bins (~±8m per edge); drone_detect_lidar uses ±5 (~±5m) to balance
  // recall vs. search cost. 11^3 = 1331 voxels vs. original 27.
  static const int kVoxelRadius = 3;

  std::vector<Eigen::Vector3i> getVoxelRound() {
    std::vector<Eigen::Vector3i> voxels;
    voxels.reserve((2 * kVoxelRadius + 1) * (2 * kVoxelRadius + 1) * (2 * kVoxelRadius + 1));
    for (int dx = -kVoxelRadius; dx <= kVoxelRadius; dx++)
      for (int dy = -kVoxelRadius; dy <= kVoxelRadius; dy++)
        for (int dz = -kVoxelRadius; dz <= kVoxelRadius; dz++)
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
    // Thread-safe snapshot of shared data
    drone_detect_lidar::TreeDetection::ConstPtr self_trees_snap;
    drone_detect_lidar::TreeDetection::ConstPtr neighbor_trees_snap;
    nav_msgs::Odometry::ConstPtr self_odom_snap;
    nav_msgs::Odometry::ConstPtr neighbor_odom_snap;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (!has_self_trees_ || !has_neighbor_trees_ ||
          !has_self_odom_ || !has_neighbor_odom_) {
        ROS_DEBUG_THROTTLE(1.0, "[TreeLocNode] Waiting for data");
        return;
      }
      self_trees_snap = self_trees_;
      neighbor_trees_snap = neighbor_trees_;
      self_odom_snap = self_odom_;
      neighbor_odom_snap = neighbor_odom_;
    }

    // ===== Step 0: Distance filter =====
    Eigen::Vector3d p_self(
      self_odom_snap->pose.pose.position.x,
      self_odom_snap->pose.pose.position.y,
      self_odom_snap->pose.pose.position.z);
    Eigen::Vector3d p_neighbor(
      neighbor_odom_snap->pose.pose.position.x,
      neighbor_odom_snap->pose.pose.position.y,
      neighbor_odom_snap->pose.pose.position.z);

    double drone_dist = (p_self - p_neighbor).norm();
    if (drone_dist > drone_dist_threshold_) {
      ROS_DEBUG_THROTTLE(2.0, "[TreeLocNode] Drones too far: %.1f m", drone_dist);
      return;
    }

    // ===== Step 1: Collect trees =====
    const auto& self_trees = self_trees_snap->trees;
    const auto& neighbor_trees = neighbor_trees_snap->trees;

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

    // 2b. Rough matching: neighbor triangles search in hash space
    auto voxel_round = getVoxelRound();
    std::vector<TriMatch> candidate_matches;

    for (size_t ni = 0; ni < neighbor_tris.size(); ni++) {
      TriKey key = makeKey(neighbor_tris[ni].a, neighbor_tris[ni].b, neighbor_tris[ni].c);
      double side_norm = neighbor_tris[ni].a + neighbor_tris[ni].b + neighbor_tris[ni].c;
      double dis_threshold = side_norm * tri_config_.rough_dis_ratio;

      for (auto& v : voxel_round) {
        TriKey search_key(
          std::get<0>(key) + v[0],
          std::get<1>(key) + v[1],
          std::get<2>(key) + v[2]);
        auto it = self_tri_map.find(search_key);
        if (it == self_tri_map.end()) continue;

        for (int si : it->second) {
          if (edgeDist(self_tris[si], neighbor_tris[ni]) < dis_threshold) {
            // Centroid distance filter (forest_loop_detector style):
            // Both drones report trees in world frame, so matching triangles
            // from the SAME trees should have nearly identical centroids.
            // We use 3m as a conservative bound to account for detection noise.
            double centroid_dist = (self_tris[si].center - neighbor_tris[ni].center).norm();
            if (centroid_dist > 3.0) continue;

            candidate_matches.push_back({si, (int)ni});
          }
        }
      }
    }

    if (candidate_matches.empty()) {
      ROS_DEBUG_THROTTLE(2.0, "[TreeLocNode] No triangle matches found");
      return;
    }

    ROS_DEBUG("[TreeLocNode] Candidate triangle pairs after centroid filter: %zu",
              candidate_matches.size());

    // ===== Step 0b: Odometry-based relative pose prior =====
    // Use drone odometry as an initial guess for the relative transform.
    // Any tree-matching result that deviates too far from this prior is
    // likely a false match (e.g. matching triangles from different tree groups).
    Eigen::Vector2d t_odom(
      p_neighbor[0] - p_self[0],
      p_neighbor[1] - p_self[1]);

    // 2c. Voting verification with odometry prior (HashReg candidate_frames_verify)
    int best_vote = 0;
    Eigen::Vector2d t_best(0, 0);
    Eigen::Matrix2d R_best = Eigen::Matrix2d::Identity();
    double best_odom_dist = 1e9;  // distance from odometry prior (for tie-breaking)

    // Pre-build point list for fast verification
    std::vector<Eigen::Vector2d> self_pts(self_trees.size());
    std::vector<Eigen::Vector2d> neighbor_pts(neighbor_trees.size());
    for (size_t i = 0; i < self_trees.size(); i++)
      self_pts[i] << self_trees[i].x, self_trees[i].y;
    for (size_t i = 0; i < neighbor_trees.size(); i++)
      neighbor_pts[i] << neighbor_trees[i].x, neighbor_trees[i].y;

    // Odometry consistency tolerance: translation within 2m of odometry prior
    // yaw within 30 degrees. These are generous enough for odometry drift.
    const double kMaxOdomTransDist = 2.0;
    const double kMaxOdomYawDeg = 30.0;

    for (size_t m = 0; m < candidate_matches.size(); m++) {
      std::pair<Eigen::Vector2d, Eigen::Matrix2d> result =
        triangleSolver(self_tris[candidate_matches[m].self_idx],
                        neighbor_tris[candidate_matches[m].neighbor_idx]);
      Eigen::Vector2d t = result.first;
      Eigen::Matrix2d R = result.second;

      // Check consistency with odometry prior
      double trans_diff = (t - t_odom).norm();
      if (trans_diff > kMaxOdomTransDist) {
        continue;  // Translation too far from odometry, likely false match
      }
      double yaw = std::atan2(R(1, 0), R(0, 0));
      double yaw_diff_deg = std::abs(yaw) * 180.0 / M_PI;
      // Note: odometry gives translation only (no yaw diff available since we
      // don't have relative yaw from odom directly). We just check that the
      // resulting yaw is within a reasonable range.
      if (yaw_diff_deg > kMaxOdomYawDeg + max_yaw_deg_) {
        continue;  // Yaw too large
      }

      int vote = 0;
      for (size_t i = 0; i < self_pts.size(); i++) {
        Eigen::Vector2d transformed = R * self_pts[i] + t;
        for (size_t j = 0; j < neighbor_pts.size(); j++) {
          if ((neighbor_pts[j] - transformed).norm() < tri_config_.geom_verify_dist) {
            vote++;
            break;
          }
        }
      }

      // Prefer higher vote; break ties by choosing the one closer to odometry prior
      double odom_dist = trans_diff;
      if (vote > best_vote || (vote == best_vote && vote > 0 && odom_dist < best_odom_dist)) {
        best_vote = vote;
        t_best = t;
        R_best = R;
        best_odom_dist = odom_dist;
      }
    }

    if (best_vote < 3) {
      ROS_DEBUG_THROTTLE(2.0, "[TreeLocNode] Triangle vote too low: %d", best_vote);
      return;
    }

    ROS_INFO("[TreeLocNode] Best triangle match vote: %d, odom_prior_diff=%.3f m",
             best_vote, best_odom_dist);

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
      double best_dist = tri_config_.geom_verify_dist;
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

    // ===== Step 4: Weighted 2D SVD =====
    // Weight each pair by the geometric mean of the two trees' detection confidence,
    // giving more influence to high-quality tree detections (high linearity, roundness, point count).
    int n = matched_pairs.size();
    Eigen::VectorXd weights(n);
    double weight_sum = 0.0;
    for (int i = 0; i < n; i++) {
      double w_self  = self_trees[matched_pairs[i].self_idx].confidence;
      double w_neigh = neighbor_trees[matched_pairs[i].neighbor_idx].confidence;
      // Geometric mean: down-weights pairs where either detection is low confidence
      weights(i) = std::sqrt(std::max(w_self, 1e-6) * std::max(w_neigh, 1e-6));
      weight_sum += weights(i);
    }
    // Normalize so weights sum to 1 (numerical stability, doesn't change the solution)
    weights /= weight_sum;

    // Weighted centroids
    Eigen::Vector2d mu_P(0, 0), mu_Q(0, 0);
    for (int i = 0; i < n; i++) {
      mu_P(0) += weights(i) * self_trees[matched_pairs[i].self_idx].x;
      mu_P(1) += weights(i) * self_trees[matched_pairs[i].self_idx].y;
      mu_Q(0) += weights(i) * neighbor_trees[matched_pairs[i].neighbor_idx].x;
      mu_Q(1) += weights(i) * neighbor_trees[matched_pairs[i].neighbor_idx].y;
    }

    // Weighted centered points (scaled by sqrt(w_i))
    Eigen::MatrixXd P_w(2, n), Q_w(2, n);
    for (int i = 0; i < n; i++) {
      double sqrt_w = std::sqrt(weights(i));
      P_w(0, i) = sqrt_w * (self_trees[matched_pairs[i].self_idx].x - mu_P(0));
      P_w(1, i) = sqrt_w * (self_trees[matched_pairs[i].self_idx].y - mu_P(1));
      Q_w(0, i) = sqrt_w * (neighbor_trees[matched_pairs[i].neighbor_idx].x - mu_Q(0));
      Q_w(1, i) = sqrt_w * (neighbor_trees[matched_pairs[i].neighbor_idx].y - mu_Q(1));
    }

    Eigen::Matrix2d H = P_w * Q_w.transpose();

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

    // Weighted RMS residual (using same weights as SVD)
    double wrms_residual = 0;
    for (int i = 0; i < n; i++) {
      double sx = self_trees[matched_pairs[i].self_idx].x;
      double sy = self_trees[matched_pairs[i].self_idx].y;
      double nx = neighbor_trees[matched_pairs[i].neighbor_idx].x;
      double ny = neighbor_trees[matched_pairs[i].neighbor_idx].y;
      double dx = nx - (R(0,0) * sx + R(0,1) * sy + t(0));
      double dy = ny - (R(1,0) * sx + R(1,1) * sy + t(1));
      wrms_residual += weights(i) * (dx*dx + dy*dy);
    }
    wrms_residual = std::sqrt(wrms_residual);  // weights already sum to 1

    if (trans_norm > max_translation_) {
      ROS_WARN("[TreeLocNode] Translation too large: %.3f m", trans_norm);
      return;
    }
    if (yaw_deg > max_yaw_deg_) {
      ROS_WARN("[TreeLocNode] Yaw too large: %.1f deg", yaw_deg);
      return;
    }
    if (wrms_residual > max_match_residual_) {
      ROS_WARN("[TreeLocNode] Weighted RMS residual too large: %.3f m", wrms_residual);
      return;
    }

    // ===== Step 6: Publish =====
    last_fitness_ = wrms_residual;

    geometry_msgs::Vector3 error_msg;
    error_msg.x = t(0);
    error_msg.y = t(1);
    error_msg.z = yaw;
    tree_pose_error_pub_.publish(error_msg);

    drone_detect_lidar::TreeRelativePose rel_msg;
    rel_msg.header.stamp = ros::Time::now();
    rel_msg.header.frame_id = self_odom_snap->header.frame_id;
    rel_msg.src_drone_id = drone_id_;
    rel_msg.dst_drone_id = neighbor_trees_snap->drone_id;
    rel_msg.dx = t(0);
    rel_msg.dy = t(1);
    rel_msg.yaw = yaw;
    double sigma2 = (wrms_residual * wrms_residual) / std::max(1, n);
    rel_msg.covariance[0] = sigma2;
    rel_msg.covariance[4] = sigma2;
    rel_msg.covariance[8] = sigma2;
    rel_msg.rms_residual = wrms_residual;
    rel_msg.num_matched_trees = n;
    tree_relative_pose_msg_pub_.publish(rel_msg);

    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = self_odom_snap->header.frame_id;
    pose_msg.pose.position.x = trans_norm;
    pose_msg.pose.orientation.w = std::cos(yaw / 2.0);
    pose_msg.pose.orientation.z = std::sin(yaw / 2.0);
    tree_relative_pose_pub_.publish(pose_msg);

    sensor_msgs::PointCloud2 pair_vis;
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl_cloud.reserve(n * 2);
    for (int i = 0; i < n; i++) {
      pcl_cloud.push_back(pcl::PointXYZ(
        self_trees[matched_pairs[i].self_idx].x,
        self_trees[matched_pairs[i].self_idx].y, 0.0));
      pcl_cloud.push_back(pcl::PointXYZ(
        neighbor_trees[matched_pairs[i].neighbor_idx].x,
        neighbor_trees[matched_pairs[i].neighbor_idx].y, 0.0));
    }
    pcl::toROSMsg(pcl_cloud, pair_vis);
    pair_vis.header.stamp = ros::Time::now();
    pair_vis.header.frame_id = self_odom_snap->header.frame_id;
    matched_tree_pairs_pub_.publish(pair_vis);

    ROS_INFO("[TreeLocNode] drone_dist=%.1fm, matched=%d, trans=(%.3f, %.3f), "
             "yaw=%.2f deg, wrms=%.3fm",
             drone_dist, n, t(0), t(1), yaw_deg, wrms_residual);
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "tree_loc_node");
  ros::NodeHandle nh("~");
  TreeLocNode node(nh);
  ros::spin();
  return 0;
}
