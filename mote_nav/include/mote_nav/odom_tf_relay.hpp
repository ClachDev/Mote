#ifndef MOTE_NAV__ODOM_TF_RELAY_HPP_
#define MOTE_NAV__ODOM_TF_RELAY_HPP_

#include <array>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace mote_nav
{

/**
 * @brief Rotate vector v by quaternion q = (x, y, z, w).
 *
 * The expression order is load-bearing: it is what makes the relay's output
 * bit-identical to the Python implementation it replaces, so a bag recorded
 * under one can be compared against the other with an exact equality. The
 * target is compiled with -ffp-contract=off for the same reason — on aarch64
 * an FMA contraction here would round differently from Python's separate
 * multiply and add.
 */
inline std::array<double, 3> rotateVector(
  const std::array<double, 4> & q, const std::array<double, 3> & v)
{
  const double x = q[0];
  const double y = q[1];
  const double z = q[2];
  const double w = q[3];
  const double ux = 2.0 * (y * v[2] - z * v[1]);
  const double uy = 2.0 * (z * v[0] - x * v[2]);
  const double uz = 2.0 * (x * v[1] - y * v[0]);
  return {
    v[0] + w * ux + (y * uz - z * uy),
    v[1] + w * uy + (z * ux - x * uz),
    v[2] + w * uz + (x * uy - y * ux),
  };
}

/**
 * @brief The wheel pose inverted into a TF leaf.
 *
 * The message carries the base pose expressed in the odometry frame; the
 * transform returned is its inverse, stamped and framed so it hangs off
 * `msg.child_frame_id` (the base) as a leaf named `child_frame`.
 */
geometry_msgs::msg::TransformStamped invertOdometry(
  const nav_msgs::msg::Odometry & msg, const std::string & child_frame);

/**
 * @class OdomTfRelay
 * @brief Republish wheel odometry as an inverted TF leaf so a lidar-odometry
 *        node can own the odom->base edge while still receiving the wheel
 *        odometry as a prior.
 *
 * diff_drive publishes the wheel pose (base in odom) on a topic; this node
 * broadcasts its inverse as base_frame -> child_frame (a leaf). kinematic_icp,
 * configured with wheel_odom_frame = child_frame, reads that leaf as its motion
 * prior and is then free to publish the real odom -> base transform itself.
 *
 * It is a component rather than a standalone node because it is woken at the
 * controller's update rate to do twenty floating-point operations: the wake-up
 * and the message hop cost more than the arithmetic, so it is loaded into the
 * container that also holds its one consumer.
 */
class OdomTfRelay : public rclcpp::Node
{
public:
  explicit OdomTfRelay(const rclcpp::NodeOptions & options);

private:
  void onOdom(const nav_msgs::msg::Odometry::ConstSharedPtr msg);

  std::string child_frame_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
};

}  // namespace mote_nav

#endif  // MOTE_NAV__ODOM_TF_RELAY_HPP_
