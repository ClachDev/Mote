#ifndef MOTE_NAV__WHEEL_SPEED_HPP_
#define MOTE_NAV__WHEEL_SPEED_HPP_

#include <cmath>

namespace mote_nav
{

/**
 * @brief Surface speed of the faster (outer) wheel for a differential drive
 *        commanded at linear velocity v and angular velocity w.
 *
 * The wheel speeds are v +/- (S/2)*w, so the outer wheel runs at
 * |v| + (S/2)*|w| where S is the wheel separation.
 */
inline double maxWheelSpeed(double v, double w, double wheel_separation)
{
  return std::abs(v) + 0.5 * wheel_separation * std::abs(w);
}

/**
 * @brief Body yaw rate with both wheels at their limit in opposite directions.
 *
 * The fastest the chassis can turn, and so the bound on any yaw rate an
 * odometry source may claim.
 */
inline double maxYawRate(double max_wheel_speed, double wheel_separation)
{
  return 2.0 * max_wheel_speed / wheel_separation;
}

}  // namespace mote_nav

#endif  // MOTE_NAV__WHEEL_SPEED_HPP_
