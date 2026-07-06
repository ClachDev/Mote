#include "mote_nav/wheel_speed_limit_critic.hpp"

#include <string>

#include "dwb_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mote_nav
{

void WheelSpeedLimitCritic::onInit()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("Failed to lock node in WheelSpeedLimitCritic");
  }

  const std::string prefix = dwb_plugin_name_ + "." + name_ + ".";

  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "max_wheel_speed", rclcpp::ParameterValue(0.218));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "wheel_separation", rclcpp::ParameterValue(0.22));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "tolerance", rclcpp::ParameterValue(1.02));

  node->get_parameter(prefix + "max_wheel_speed", max_wheel_speed_);
  node->get_parameter(prefix + "wheel_separation", wheel_separation_);
  node->get_parameter(prefix + "tolerance", tolerance_);
}

double WheelSpeedLimitCritic::scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj)
{
  const double v = traj.velocity.x;
  const double w = traj.velocity.theta;
  const double max_wheel = maxWheelSpeed(v, w, wheel_separation_);

  if (max_wheel > max_wheel_speed_ * tolerance_) {
    throw dwb_core::IllegalTrajectoryException(name_, "wheel speed limit exceeded");
  }

  return 0.0;
}

}  // namespace mote_nav

PLUGINLIB_EXPORT_CLASS(mote_nav::WheelSpeedLimitCritic, dwb_core::TrajectoryCritic)
