#include "mote_nav/icp_odom_gate.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iterator>
#include <memory>
#include <string>

#include "mote_nav/wheel_speed.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace mote_nav
{

namespace
{

double wrap(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}

constexpr double kNsToS = 1e-9;

}  // namespace

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
    wrap(byaw - ayaw),
  };
}

Pose2 compose(const Pose2 & a, const Increment & d)
{
  const double c = std::cos(a.yaw);
  const double s = std::sin(a.yaw);
  return {
    a.x + c * d.x - s * d.y,
    a.y + s * d.x + c * d.y,
    wrap(a.yaw + d.yaw),
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

WheelHistory::WheelHistory(std::size_t capacity)
: capacity_(std::max<std::size_t>(capacity, 2))
{
}

void WheelHistory::add(std::int64_t t, const Pose2 & pose)
{
  if (!samples_.empty()) {
    if (t == samples_.back().t) {
      return;
    }
    if (t < samples_.back().t) {
      samples_.clear();
    }
  }
  samples_.push_back({t, pose});
  while (samples_.size() > capacity_) {
    samples_.pop_front();
  }
}

bool WheelHistory::poseAt(std::int64_t t, Pose2 & out) const
{
  if (samples_.empty() || t < samples_.front().t || t > samples_.back().t) {
    return false;
  }
  const auto b = std::lower_bound(
    samples_.begin(), samples_.end(), t,
    [](const Sample & s, std::int64_t v) {return s.t < v;});
  if (b->t == t) {
    out = b->pose;
    return true;
  }
  const auto a = std::prev(b);
  const double f = static_cast<double>(t - a->t) / static_cast<double>(b->t - a->t);
  out.x = a->pose.x + f * (b->pose.x - a->pose.x);
  out.y = a->pose.y + f * (b->pose.y - a->pose.y);
  out.yaw = wrap(a->pose.yaw + f * wrap(b->pose.yaw - a->pose.yaw));
  return true;
}

bool WheelHistory::increment(std::int64_t from, std::int64_t to, Increment & out) const
{
  Pose2 a;
  Pose2 b;
  if (!poseAt(from, a) || !poseAt(to, b)) {
    return false;
  }
  out = relativeMotion(a.x, a.y, a.yaw, b.x, b.y, b.yaw);
  return true;
}

void WheelHistory::dropBefore(std::int64_t t)
{
  while (samples_.size() >= 2 && samples_[1].t <= t) {
    samples_.pop_front();
  }
}

GatedOdometry::GatedOdometry(const GateLimits & limits, double max_propagation)
: limits_(limits),
  max_propagation_ns_(static_cast<std::int64_t>(std::llround(max_propagation * 1e9)))
{
}

ScanUpdate GatedOdometry::onScan(std::int64_t t, const Pose2 & icp)
{
  ScanUpdate u;
  if (!have_scan_) {
    // kinematic_icp initialises its pose from the wheel odometry, so its first
    // pose is already in the wheel frame and the published edge is continuous
    // with it.
    anchor_ = icp;
    u.first = true;
  } else {
    if (t < scan_t_) {
      // The clock went backwards; let publication restart with it.
      have_published_ = false;
    }
    u.dt = static_cast<double>(t - scan_t_) * kNsToS;
    u.icp = relativeMotion(last_icp_.x, last_icp_.y, last_icp_.yaw, icp.x, icp.y, icp.yaw);

    Increment wheel;
    const bool have_wheel = wheel_.increment(scan_t_, t, wheel);

    u.applied = u.icp;
    if (exceedsEnvelope(u.icp, u.dt, limits_)) {
      // With no wheel prior the degraded substitute is the increment itself,
      // held to the envelope.
      const Increment substitute =
        have_wheel ? wheel : clampToEnvelope(u.icp, u.dt, limits_);
      u.applied = gateIncrement(u.icp, substitute, u.dt, limits_).delta;
      u.rejected = true;
      u.wheel_prior = have_wheel;
    }

    const Pose2 next_anchor = compose(anchor_, u.applied);
    if (have_wheel) {
      const Pose2 predicted = compose(anchor_, clampToEnvelope(wheel, u.dt, limits_));
      u.step = relativeMotion(
        predicted.x, predicted.y, predicted.yaw, next_anchor.x, next_anchor.y, next_anchor.yaw);
      u.have_step = true;
    }
    anchor_ = next_anchor;
  }

  have_scan_ = true;
  scan_t_ = t;
  last_icp_ = icp;
  wheel_.dropBefore(t);
  return u;
}

void GatedOdometry::onWheel(std::int64_t t, const Pose2 & wheel)
{
  if (!wheel_.empty() && t < wheel_.latest()) {
    have_published_ = false;
  }
  wheel_.add(t, wheel);
}

bool GatedOdometry::propagate(std::int64_t t, Pose2 & out) const
{
  if (t < scan_t_ || t - scan_t_ > max_propagation_ns_) {
    return false;
  }
  Increment inc;
  if (!wheel_.increment(scan_t_, t, inc)) {
    return false;
  }
  out = compose(anchor_, clampToEnvelope(inc, static_cast<double>(t - scan_t_) * kNsToS, limits_));
  return true;
}

bool GatedOdometry::next(std::int64_t & t, Pose2 & pose)
{
  if (!have_scan_) {
    return false;
  }
  std::int64_t ct = scan_t_;
  Pose2 cp = anchor_;
  if (!wheel_.empty() && wheel_.latest() > scan_t_) {
    Pose2 p;
    if (propagate(wheel_.latest(), p)) {
      ct = wheel_.latest();
      cp = p;
    }
  }
  if (have_published_ && ct <= published_t_) {
    return false;
  }
  have_published_ = true;
  published_t_ = ct;
  t = ct;
  pose = cp;
  return true;
}

IcpOdomGate::IcpOdomGate(const rclcpp::NodeOptions & options)
: rclcpp::Node("icp_odom_gate", options)
{
  odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
  base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
  // A few scans' worth: one late or dropped scan is carried on the wheels, a
  // lidar that has stopped is not.
  const double max_propagation = declare_parameter<double>("max_propagation", 0.5);

  // robot.yaml's measured hardware envelope, overlaid by the launch file.
  const double max_wheel_speed = declare_parameter<double>("max_wheel_speed", 0.218);
  const double wheel_separation = declare_parameter<double>("wheel_separation", 0.22);
  const double tolerance = declare_parameter<double>("tolerance", 1.15);
  limits_.max_speed = max_wheel_speed * tolerance;
  limits_.max_yaw_rate = maxYawRate(max_wheel_speed, wheel_separation) * tolerance;
  odometry_ = std::make_unique<GatedOdometry>(limits_, max_propagation);

  RCLCPP_INFO(
    get_logger(),
    "gating %s->%s at %.3f m/s and %.3f rad/s (x%.2f of the drive envelope), "
    "propagated on wheels for up to %.2f s",
    odom_frame_.c_str(), base_frame_.c_str(), limits_.max_speed, limits_.max_yaw_rate,
    tolerance, max_propagation);

  broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>("~/odometry", rclcpp::QoS(1));

  // Matches the publisher kinematic_icp creates: system defaults, depth 1.
  subscription_ = create_subscription<nav_msgs::msg::Odometry>(
    "odom_in", rclcpp::SystemDefaultsQoS().keep_last(1).durability_volatile(),
    std::bind(&IcpOdomGate::onOdom, this, std::placeholders::_1));
  wheel_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
    "wheel_odom_in", rclcpp::QoS(10),
    std::bind(&IcpOdomGate::onWheel, this, std::placeholders::_1));
}

void IcpOdomGate::publishTransform()
{
  std::int64_t t = 0;
  Pose2 pose;
  if (!odometry_->next(t, pose)) {
    return;
  }
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, pose.yaw);

  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = rclcpp::Time(t, RCL_ROS_TIME);
  tf.header.frame_id = odom_frame_;
  tf.child_frame_id = base_frame_;
  tf.transform.translation.x = pose.x;
  tf.transform.translation.y = pose.y;
  tf.transform.translation.z = 0.0;
  tf.transform.rotation = tf2::toMsg(q);
  broadcaster_->sendTransform(tf);
}

void IcpOdomGate::onWheel(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  odometry_->onWheel(
    rclcpp::Time(msg->header.stamp).nanoseconds(),
    {msg->pose.pose.position.x, msg->pose.pose.position.y,
      tf2::getYaw(msg->pose.pose.orientation)});
  publishTransform();
}

void IcpOdomGate::onOdom(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  const auto u = odometry_->onScan(
    rclcpp::Time(msg->header.stamp).nanoseconds(),
    {msg->pose.pose.position.x, msg->pose.pose.position.y,
      tf2::getYaw(msg->pose.pose.orientation)});

  if (u.rejected) {
    ++rejected_;
    RCLCPP_WARN(
      get_logger(),
      "rejected icp increment %u: %.3f m/s, %.3f rad/s over %.3f s "
      "(limits %.3f, %.3f) -- substituted %s [%.4f, %.4f, %.4f]",
      static_cast<unsigned>(rejected_), std::hypot(u.icp.x, u.icp.y) / u.dt,
      std::abs(u.icp.yaw) / u.dt, u.dt, limits_.max_speed, limits_.max_yaw_rate,
      u.wheel_prior ? "wheel odometry" : "a clamp", u.applied.x, u.applied.y, u.applied.yaw);
  }
  if (u.have_step) {
    RCLCPP_DEBUG(
      get_logger(), "re-anchored: step %.4f m, %.4f rad",
      std::hypot(u.step.x, u.step.y), u.step.yaw);
  }

  publishTransform();

  const Pose2 & pose = odometry_->anchor();
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, pose.yaw);

  nav_msgs::msg::Odometry out = *msg;
  out.header.frame_id = odom_frame_;
  out.child_frame_id = base_frame_;
  out.pose.pose.position.x = pose.x;
  out.pose.pose.position.y = pose.y;
  out.pose.pose.position.z = 0.0;
  out.pose.pose.orientation = tf2::toMsg(q);
  // The twist is recomputed rather than carried over: the incoming one is the
  // velocity of the increment that was just rejected, and a message pairing a
  // corrected pose with the uncorrected speed that produced it is a trap.
  const double dt = u.dt;
  out.twist.twist.linear.x = dt > 0.0 ? u.applied.x / dt : 0.0;
  out.twist.twist.linear.y = dt > 0.0 ? u.applied.y / dt : 0.0;
  out.twist.twist.linear.z = 0.0;
  out.twist.twist.angular.x = 0.0;
  out.twist.twist.angular.y = 0.0;
  out.twist.twist.angular.z = dt > 0.0 ? u.applied.yaw / dt : 0.0;
  odom_publisher_->publish(out);
}

}  // namespace mote_nav

RCLCPP_COMPONENTS_REGISTER_NODE(mote_nav::IcpOdomGate)
