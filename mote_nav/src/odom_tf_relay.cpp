#include "mote_nav/odom_tf_relay.hpp"

#include <memory>
#include <string>
#include <utility>

namespace mote_nav
{

geometry_msgs::msg::TransformStamped invertOdometry(
  const nav_msgs::msg::Odometry & msg, const std::string & child_frame)
{
  const auto & p = msg.pose.pose.position;
  const auto & q = msg.pose.pose.orientation;
  const std::array<double, 4> q_inv{-q.x, -q.y, -q.z, q.w};
  const std::array<double, 3> ti = rotateVector(q_inv, {p.x, p.y, p.z});

  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = msg.header.stamp;
  t.header.frame_id = msg.child_frame_id;  // base_footprint
  t.child_frame_id = child_frame;          // odom_wheel (leaf)
  t.transform.translation.x = -ti[0];
  t.transform.translation.y = -ti[1];
  t.transform.translation.z = -ti[2];
  t.transform.rotation.x = q_inv[0];
  t.transform.rotation.y = q_inv[1];
  t.transform.rotation.z = q_inv[2];
  t.transform.rotation.w = q_inv[3];
  return t;
}

OdomTfRelay::OdomTfRelay(const rclcpp::NodeOptions & options)
: rclcpp::Node("odom_tf_relay", options)
{
  child_frame_ = declare_parameter<std::string>("child_frame", "odom_wheel");
  broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  subscription_ = create_subscription<nav_msgs::msg::Odometry>(
    "odom_in", rclcpp::QoS(10),
    [this](const nav_msgs::msg::Odometry::ConstSharedPtr msg) {this->onOdom(msg);});
}

void OdomTfRelay::onOdom(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  broadcaster_->sendTransform(invertOdometry(*msg, child_frame_));
}

}  // namespace mote_nav

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(mote_nav::OdomTfRelay)
