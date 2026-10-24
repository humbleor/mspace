#pragma once
#include <Eigen/Geometry>
#include <cmath>
#include <stdexcept>

namespace swarm_frame {
inline double wrap(double a) { return std::atan2(std::sin(a), std::cos(a)); }
inline bool validQuaternion(const Eigen::Quaterniond& q) {
  return q.coeffs().allFinite() && std::abs(q.squaredNorm() - 1.0) < 0.02;
}
inline double yaw(const Eigen::Quaterniond& q) {
  return std::atan2(2*(q.w()*q.z()+q.x()*q.y()), 1-2*(q.y()*q.y()+q.z()*q.z()));
}
inline Eigen::Quaterniond yawQuaternion(double a) {
  return Eigen::Quaterniond(Eigen::AngleAxisd(a, Eigen::Vector3d::UnitZ()));
}
inline double tilt(const Eigen::Quaterniond& q) {
  const double c = (q * Eigen::Vector3d::UnitZ()).z();
  return std::acos(std::max(-1.0, std::min(1.0, c)));
}
// T_I_B applied to a body point: p_imu = p_lio + R_lio * t_IB.
inline Eigen::Vector3d bodyPoint(const Eigen::Vector3d& p, const Eigen::Quaterniond& q,
                                const Eigen::Vector3d& translation) {
  return p + q * translation;
}
// Body orientation in the LIO frame: q_lio_body = q_lio_imu * R_IB.
inline Eigen::Quaterniond bodyOrientation(const Eigen::Quaterniond& q, const Eigen::Quaterniond& rotation) {
  return (q * rotation).normalized();
}
struct YawTransform {
  double angle = 0;
  Eigen::Vector3d translation = Eigen::Vector3d::Zero();
  Eigen::Matrix3d rotation() const { return yawQuaternion(angle).toRotationMatrix(); }
  Eigen::Vector3d point(const Eigen::Vector3d& p) const { return rotation()*p+translation; }
  Eigen::Vector3d vector(const Eigen::Vector3d& v) const { return rotation()*v; }
  Eigen::Vector3d inversePoint(const Eigen::Vector3d& p) const { return rotation().transpose()*(p-translation); }
  Eigen::Vector3d inverseVector(const Eigen::Vector3d& v) const { return rotation().transpose()*v; }
  Eigen::Quaterniond orientation(const Eigen::Quaterniond& q) const { return (yawQuaternion(angle)*q).normalized(); }
  double inverseYaw(double a) const { return wrap(a-angle); }
  static YawTransform fromInitial(const Eigen::Vector3d& desired, double desired_yaw,
                                 const Eigen::Vector3d& p, const Eigen::Quaterniond& q) {
    if (!desired.allFinite() || !p.allFinite() || !std::isfinite(desired_yaw) || !validQuaternion(q))
      throw std::invalid_argument("non-finite pose or invalid quaternion");
    YawTransform out;
    out.angle = wrap(desired_yaw-yaw(q.normalized()));
    out.translation = desired-out.rotation()*p;
    return out;
  }
};
}  // namespace swarm_frame
