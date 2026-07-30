#ifndef MOTE_NAV__ICP_ODOM_GATE_HPP_
#define MOTE_NAV__ICP_ODOM_GATE_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace mote_nav
{

/// Planar motion over one interval, expressed in the body frame it started in.
struct Increment
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

/// The envelope a differential drive can actually produce, plus the slack
/// allowed for ordinary scan-match noise on top of it.
struct GateLimits
{
  double max_speed{0.0};     ///< m/s, translation
  double max_yaw_rate{0.0};  ///< rad/s
};

/// What the gate decided for one interval.
struct GateResult
{
  Increment delta;
  bool rejected{false};
};

/**
 * @brief Motion of pose b expressed in pose a's body frame.
 */
Increment relativeMotion(
  double ax, double ay, double ayaw, double bx, double by, double byaw);

/**
 * @brief Whether an increment claims motion outside the drive's envelope.
 *
 * A non-positive `dt` implies no speed at all, so nothing can exceed anything.
 */
bool exceedsEnvelope(const Increment & icp, double dt, const GateLimits & limits);

/**
 * @brief The increment held to the envelope, keeping its direction.
 *
 * The degraded answer, used only when the wheel prior is unavailable: refusing
 * the impossible part of the motion beats refusing all of it.
 */
Increment clampToEnvelope(const Increment & icp, double dt, const GateLimits & limits);

/**
 * @brief Accept a lidar-odometry increment, or substitute the wheel one.
 *
 * The drive cannot exceed `limits`, and wheel slip cannot make the *lidar*
 * over-read (slip makes the wheels over-read), so an increment above the
 * envelope is a scan-match excursion rather than motion anybody missed. The
 * substitute is the wheel increment over the same interval: it is what the
 * actuators did, and it is the prior the scan match started from before it
 * wandered off.
 *
 * The substitute is itself held to the envelope, so whatever comes back is
 * inside it: the wheel increment is trusted at exactly the moment the other
 * source has been judged untrustworthy, and an unchecked substitute would be a
 * way past the gate rather than the answer to it.
 *
 * A non-positive `dt` yields no judgement and the increment is passed through —
 * two poses at one stamp imply no speed at all.
 */
GateResult gateIncrement(
  const Increment & icp, const Increment & wheel, double dt, const GateLimits & limits);

/**
 * @class IcpOdomGate
 * @brief Rejects physically impossible increments before they reach odom->base.
 *
 * kinematic_icp occasionally emits a pose implying a body speed the drive
 * cannot produce: measured on real mapping bags at up to 1.2 m/s against a
 * 0.218 m/s hardware limit, including 0.12 m in a single scan while the wheels
 * reported the robot stationary. The jumps are single frames, but they are
 * *steps*, not spikes — the scan match re-registers from the displaced pose and
 * never gives the displacement back, so each one is permanent error in the map
 * frame and in every zone taught in it.
 *
 * Because kinematic_icp broadcasts odom->base itself, nothing downstream can
 * retract a bad transform after the fact. So it is configured to publish only
 * its odometry topic, in a frame of its own, and this node owns the odom->base
 * edge: it accumulates ICP's increments, and where one exceeds the envelope it
 * accumulates the wheel increment instead. The published pose is then
 * permanently offset from ICP's internal pose by exactly the excursions it
 * absorbed, which is the point; ICP is unaffected, since its motion prior comes
 * from the wheel-odometry TF leaf and never from its own output.
 *
 * The envelope comes from `robot.yaml`'s `max_wheel_speed` and
 * `wheel_separation` — the same two numbers `WheelSpeedLimitCritic` bounds Nav2
 * with, through the same `maxYawRate` helper, so there is one description of
 * what the hardware can do rather than two that can drift.
 */
class IcpOdomGate : public rclcpp::Node
{
public:
  explicit IcpOdomGate(const rclcpp::NodeOptions & options);

private:
  void onOdom(const nav_msgs::msg::Odometry::ConstSharedPtr msg);

  /// Wheel increment between two stamps, read from TF the way kinematic_icp
  /// reads its own prior. Falls back to the ICP increment clamped to the
  /// envelope when TF cannot answer, so a missing prior degrades to a bound
  /// rather than to a pass-through.
  Increment wheelIncrement(
    const rclcpp::Time & from, const rclcpp::Time & to, const Increment & icp, double dt,
    bool & ok) const;

  std::string odom_frame_;
  std::string base_frame_;
  std::string wheel_odom_frame_;
  GateLimits limits_;
  double tf_timeout_{0.05};

  bool have_last_{false};
  double last_x_{0.0};
  double last_y_{0.0};
  double last_yaw_{0.0};
  rclcpp::Time last_stamp_;

  double x_{0.0};
  double y_{0.0};
  double yaw_{0.0};
  std::size_t rejected_{0};

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
};

}  // namespace mote_nav

#endif  // MOTE_NAV__ICP_ODOM_GATE_HPP_
