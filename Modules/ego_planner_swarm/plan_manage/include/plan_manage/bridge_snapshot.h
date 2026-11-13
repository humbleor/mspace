#pragma once

#include <cmath>
#include <traj_utils/Bspline.h>

namespace ego_planner
{
  // The caller must have verified the peer identity, coordinate epoch and clock bound.
  inline bool usableBridgeSnapshot(const traj_utils::Bspline &msg, int peer_id,
                                   const ros::Time &now, const ros::Time &newest_start,
                                   double clock_bound, bool allow_early_buffer = false)
  {
    const auto count = msg.pos_pts.size();
    if (peer_id < 1 || peer_id > 50 || msg.drone_id != peer_id ||
        msg.order != 3 || count < 4 || count > 9996 || msg.knots.size() != count + 4 ||
        now.isZero() || msg.start_time.isZero() || msg.start_time <= newest_start ||
        !std::isfinite(clock_bound) || clock_bound < 0 || clock_bound > 0.25)
      return false;

    for (size_t i = 0; i < msg.knots.size(); ++i)
      if (!std::isfinite(msg.knots[i]) || (i && msg.knots[i] <= msg.knots[i - 1]))
        return false;
    for (const auto &point : msg.pos_pts)
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
        return false;

    const double duration = msg.knots[count] - msg.knots[3];
    const double age = (now - msg.start_time).toSec();
    const double earliest = allow_early_buffer ? -clock_bound : clock_bound;
    return std::isfinite(duration) && age >= earliest && age + clock_bound < duration;
  }
}
