#ifndef MOTE_NAV__WHEEL_SPEED_LIMIT_CRITIC_HPP_
#define MOTE_NAV__WHEEL_SPEED_LIMIT_CRITIC_HPP_

#include <cmath>

#include "dwb_core/trajectory_critic.hpp"
#include "dwb_msgs/msg/trajectory2_d.hpp"

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
 * @class WheelSpeedLimitCritic
 * @brief Rejects trajectories whose implied per-wheel speed exceeds the servo limit.
 *
 * DWB samples a v x w rectangle with no notion of the differential-drive
 * coupling between linear and angular velocity. This critic marks any sample
 * needing a wheel faster than max_wheel_speed as illegal, so DWB only commands
 * (v, w) pairs the hardware can actually deliver.
 */
class WheelSpeedLimitCritic : public dwb_core::TrajectoryCritic
{
public:
  void onInit() override;
  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj) override;

private:
  double max_wheel_speed_{0.218};
  double wheel_separation_{0.22};
  double tolerance_{1.02};
};

}  // namespace mote_nav

#endif  // MOTE_NAV__WHEEL_SPEED_LIMIT_CRITIC_HPP_
