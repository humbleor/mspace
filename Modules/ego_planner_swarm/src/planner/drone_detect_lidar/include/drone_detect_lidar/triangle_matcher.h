#ifndef _DRONE_DETECT_LIDAR_TRIANGLE_MATCHER_H_
#define _DRONE_DETECT_LIDAR_TRIANGLE_MATCHER_H_

#include <tuple>
#include <set>
#include <vector>
#include <Eigen/Eigen>

namespace drone_detect_lidar {

struct Triangle2D {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector2d A, B, C;
  Eigen::Vector2d center;
  double a, b, c;  // sorted: a < b < c
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

struct TriangleMatchConfig {
  double triangle_min_side;
  double triangle_max_side;
  double isosceles_threshold;
  double equilateral_threshold;
  double rough_dis_ratio;
  double geom_verify_dist;

  TriangleMatchConfig()
    : triangle_min_side(0.3), triangle_max_side(20.0),
      isosceles_threshold(0.1), equilateral_threshold(0.15),
      rough_dis_ratio(0.05), geom_verify_dist(0.3) {}
};

} // namespace drone_detect_lidar

#endif // _DRONE_DETECT_LIDAR_TRIANGLE_MATCHER_H_
