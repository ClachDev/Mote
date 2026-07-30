#include "mote_nav/icp_odom_gate.hpp"

#include <cmath>
#include <memory>
#include <string>

#include "mote_nav/wheel_speed.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace mote_nav
{

Increment relativeMotion(
  double ax, double ay, double ayaw, double bx, double by, double byaw)
{
  const double dx = bx - ax;
  const double dy = by - ay;
  const double c = std::cos(-ayaw);
  const double s = std::sin(-ayaw);
  return {
    c * dx - s * dy,
    s * dx + c * dy,
    std::atan2(std::sin(byaw - ayaw), std::cos(byaw - ayaw)),
  };
}

bool exceedsEnvelope(const Increment & icp, double dt, const GateLimits & limits)
{
  if (!(dt > 0.0)) {
    return false;
  }
  return std::hypot(icp.x, icp.y) / dt > limits.max_speed ||
         std::abs(icp.yaw) / dt > limits.max_yaw_rate;
}

GateResult gateIncrement(
  const Increment & icp, const Increment & wheel, double dt, const GateLimits & limits)
{
  if (!exceedsEnvelope(icp, dt, limits)) {
    return {icp, false};
  }
  // The substitute is bounded too. It is trusted precisely when the other
  // source has been judged untrustworthy, so leaving it unchecked would make
  // the wheel odometry a way past the gate rather than the answer to it.
  return {clampToEnvelope(wheel, dt, limits), true};
}

Increment clampToEnvelope(const Increment & icp, double dt, const GateLimits & limits)
{
  Increment out = icp;
  const double d = std::hypot(icp.x, icp.y);
  const double max_d = limits.max_speed * dt;
  if (d > max_d && d > 0.0) {
    const double k = max_d / d;
    out.x = icp.x * k;
    out.y = icp.y * k;
  }
  const double max_yaw = limits.max_yaw_rate * dt;
  if (std::abs(out.yaw) > max_yaw) {
    out.yaw = std::copysign(max_yaw, out.yaw);
  }
  return out;
}

IcpOdomGate::IcpOdomGate(const rclcpp::NodeOptions & options)
: rclcpp::Node("icp_odom_gate", options), last_stamp_(0, 0, RCL_ROS_TIME)
{
  odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
  base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
  wheel_odom_frame_ = declare_parameter<std::string>("wheel_odom_frame", "odom_wheel");
  tf_timeout_ = declare_parameter<double>("tf_timeout", 0.05);

  // robot.yaml's measured hardware envelope, overlaid by the launch file.
  const double max_wheel_speed = declare_parameter<double>("max_wheel_speed", 0.218);
  const double wheel_separation = declare_parameter<double>("wheel_separation", 0.22);
  const double tolerance = declare_parameter<double>("tolerance", 1.15);
  limits_.max_speed = max_wheel_speed * tolerance;
  limits_.max_yaw_rate = maxYawRate(max_wheel_speed, wheel_separation) * tolerance;

  RCLCPP_INFO(
    get_logger(), "gating %s->%s at %.3f m/s and %.3f rad/s (x%.2f of the drive envelope)",
    odom_frame_.c_str(), base_frame_.c_str(), limits_.max_speed, limits_.max_yaw_rate,
    tolerance);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  // Its own spin thread, because the wheel-prior lookup blocks inside the
  // odometry callback: on this component's single-threaded executor the TF
  // updates it is waiting for would otherwise queue behind it and never arrive.
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, true);
  broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>("~/odometry", rclcpp::QoS(1));

  // Matches the publisher kinematic_icp creates: system defaults, depth 1.
  subscription_ = create_subscription<nav_msgs::msg::Odometry>(
    "odom_in", rclcpp::SystemDefaultsQoS().keep_last(1).durability_volatile(),
    std::bind(&IcpOdomGate::onOdom, this, std::placeholders::_1));
}

Increment IcpOdomGate::wheelIncrement(
  const rclcpp::Time & from, const rclcpp::Time & to, const Increment & icp, double dt,
  bool & ok) const
{
  try {
    // base at `to` expressed in base at `from`, held together by the wheel
    // odometry frame -- the same query kinematic_icp makes for its prior.
    const auto tf = tf_buffer_->lookupTransform(
      base_frame_, from, base_frame_, to, wheel_odom_frame_,
      rclcpp::Duration::from_seconds(tf_timeout_));
    ok = true;
    return {
      tf.transform.translation.x,
      tf.transform.translation.y,
      tf2::getYaw(tf.transform.rotation),
    };
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(
      get_logger(), "no wheel prior for the rejected increment (%s); clamping instead", ex.what());
    ok = false;
    return clampToEnvelope(icp, dt, limits_);
  }
}

void IcpOdomGate::onOdom(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  const rclcpp::Time stamp(msg->header.stamp);
  const double mx = msg->pose.pose.position.x;
  const double my = msg->pose.pose.position.y;
  const double myaw = tf2::getYaw(msg->pose.pose.orientation);

  Increment applied;
  double dt = 0.0;

  if (!have_last_) {
    // kinematic_icp initialises its pose from the wheel odometry, so its first
    // pose is already in the wheel frame and the published edge is continuous
    // with it.
    x_ = mx;
    y_ = my;
    yaw_ = myaw;
  } else {
    dt = (stamp - last_stamp_).seconds();
    const Increment icp = relativeMotion(last_x_, last_y_, last_yaw_, mx, my, myaw);

    applied = icp;
    // The wheel prior is only fetched for an increment that fails, so the
    // blocking lookup stays off the ordinary path.
    if (exceedsEnvelope(icp, dt, limits_)) {
      bool ok = false;
      const Increment wheel = wheelIncrement(last_stamp_, stamp, icp, dt, ok);
      applied = gateIncrement(icp, wheel, dt, limits_).delta;
      ++rejected_;
      RCLCPP_WARN(
        get_logger(),
        "rejected icp increment %u: %.3f m/s, %.3f rad/s over %.3f s "
        "(limits %.3f, %.3f) -- substituted %s [%.4f, %.4f, %.4f]",
        static_cast<unsigned>(rejected_), std::hypot(icp.x, icp.y) / dt,
        std::abs(icp.yaw) / dt, dt, limits_.max_speed, limits_.max_yaw_rate,
        ok ? "wheel odometry" : "a clamp", applied.x, applied.y, applied.yaw);
    }

    const double c = std::cos(yaw_);
    const double s = std::sin(yaw_);
    x_ += c * applied.x - s * applied.y;
    y_ += s * applied.x + c * applied.y;
    yaw_ = std::atan2(std::sin(yaw_ + applied.yaw), std::cos(yaw_ + applied.yaw));
  }

  have_last_ = true;
  last_x_ = mx;
  last_y_ = my;
  last_yaw_ = myaw;
  last_stamp_ = stamp;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw_);

  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = msg->header.stamp;
  tf.header.frame_id = odom_frame_;
  tf.child_frame_id = base_frame_;
  tf.transform.translation.x = x_;
  tf.transform.translation.y = y_;
  tf.transform.translation.z = 0.0;
  tf.transform.rotation = tf2::toMsg(q);
  broadcaster_->sendTransform(tf);

  nav_msgs::msg::Odometry out = *msg;
  out.header.frame_id = odom_frame_;
  out.child_frame_id = base_frame_;
  out.pose.pose.position.x = x_;
  out.pose.pose.position.y = y_;
  out.pose.pose.position.z = 0.0;
  out.pose.pose.orientation = tf2::toMsg(q);
  // The twist is recomputed rather than carried over: the incoming one is the
  // velocity of the increment that was just rejected, and a message pairing a
  // corrected pose with the uncorrected speed that produced it is a trap.
  out.twist.twist.linear.x = dt > 0.0 ? applied.x / dt : 0.0;
  out.twist.twist.linear.y = dt > 0.0 ? applied.y / dt : 0.0;
  out.twist.twist.linear.z = 0.0;
  out.twist.twist.angular.x = 0.0;
  out.twist.twist.angular.y = 0.0;
  out.twist.twist.angular.z = dt > 0.0 ? applied.yaw / dt : 0.0;
  odom_publisher_->publish(out);
}

}  // namespace mote_nav

RCLCPP_COMPONENTS_REGISTER_NODE(mote_nav::IcpOdomGate)
