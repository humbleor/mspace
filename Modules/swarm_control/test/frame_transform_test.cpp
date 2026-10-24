#include "swarm_frame_transform.h"
#include <cassert>
#include <iostream>
using swarm_frame::YawTransform;
using swarm_frame::yawQuaternion;
using swarm_frame::yaw;
int main() {
  const double pi = std::acos(-1.0);
  const Eigen::Vector3d initial(0, 5, 0), origin(2, -3, 1);
  const auto t = YawTransform::fromInitial(initial, 0, origin, yawQuaternion(pi / 2));
  assert(t.point(origin).isApprox(initial, 1e-10));
  assert(t.point(origin + Eigen::Vector3d(1, 0, 0)).isApprox(Eigen::Vector3d(0, 4, 0), 1e-10));
  assert(std::abs(yaw(t.orientation(yawQuaternion(pi / 2)))) < 1e-10);
  const Eigen::Vector3d target(3, 7, 2), velocity(1, 2, 3);
  assert(t.point(t.inversePoint(target)).isApprox(target, 1e-10));
  assert(t.vector(t.inverseVector(velocity)).isApprox(velocity, 1e-10));
  assert(std::abs(swarm_frame::wrap(t.inverseYaw(0) - pi / 2)) < 1e-10);
  const auto wrapped = YawTransform::fromInitial(initial, -pi + .01, origin, yawQuaternion(pi - .01));
  assert(std::abs(wrapped.angle - .02) < 1e-10);
  assert((yawQuaternion(pi / 2) * Eigen::Vector3d(1, 0, 0)).isApprox(Eigen::Vector3d(0, 1, 0), 1e-10));
  assert(!swarm_frame::validQuaternion(Eigen::Quaterniond(0, 0, 0, 0)));
  assert(!swarm_frame::validQuaternion(Eigen::Quaterniond(2, 0, 0, 0)));
  assert(swarm_frame::validQuaternion(yawQuaternion(pi)));
  // IMU->body extrinsics: body origin sits 1 m along the LIO X axis, mounts rotated 90 deg.
  const Eigen::Vector3d body_translation(1, 0, 0);
  assert(swarm_frame::bodyPoint(origin, yawQuaternion(pi / 2), body_translation)
             .isApprox(origin + Eigen::Vector3d(0, 1, 0), 1e-10));
  assert(std::abs(yaw(swarm_frame::bodyOrientation(yawQuaternion(pi / 2), yawQuaternion(pi / 2))) - pi) < 1e-10);
  std::cout << "frame transform: placement, origins, 90deg, inverse, wrap, velocity, quaternion, extrinsics passed\n";
}
