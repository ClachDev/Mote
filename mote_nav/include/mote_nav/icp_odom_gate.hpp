#ifndef MOTE_NAV__ICP_ODOM_GATE_HPP_
#define MOTE_NAV__ICP_ODOM_GATE_HPP_

#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.hpp"

namespace mote_nav
{

/// Planar motion over one interval, expressed in the body frame it started in.
struct Increment
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

/// A planar pose in some odometry frame.
struct Pose2
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
 * @brief Pose `a` moved by `d`, where `d` is expressed in `a`'s body frame.
 */
Pose2 compose(const Pose2 & a, const Increment & d);

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
 * @brief Recent wheel-odometry poses, interpolated at arbitrary stamps.
 *
 * Stamps are nanoseconds. A sample older than the latest means the clock went
 * backwards (a restarted sim, a looped bag), so the history restarts from it
 * rather than refusing every sample that follows.
 */
class WheelHistory
{
public:
  explicit WheelHistory(std::size_t capacity = 500);

  void add(std::int64_t t, const Pose2 & pose);

  /// Linear in translation, the short way round in yaw. False outside the span
  /// of the samples held: nothing is extrapolated.
  bool poseAt(std::int64_t t, Pose2 & out) const;

  /// Motion between two stamps, in the body frame at `from`.
  bool increment(std::int64_t from, std::int64_t to, Increment & out) const;

  /// Forget everything older than `t` except the last sample at or before it,
  /// which `poseAt(t)` still needs.
  void dropBefore(std::int64_t t);

  bool empty() const {return samples_.empty();}
  std::size_t size() const {return samples_.size();}
  std::int64_t latest() const {return samples_.back().t;}

private:
  struct Sample
  {
    std::int64_t t;
    Pose2 pose;
  };
  std::deque<Sample> samples_;
  std::size_t capacity_;
};

/// What one lidar increment did to the gated track.
struct ScanUpdate
{
  Increment icp;           ///< kinematic_icp's increment since the previous scan
  Increment applied;       ///< what the gate accumulated: `icp` or its substitute
  double dt{0.0};
  bool first{false};       ///< the scan that seeded the track; nothing was judged
  bool rejected{false};
  bool wheel_prior{false};  ///< a rejection substituted wheels rather than a clamp
  /// The jump re-anchoring makes: the wheel-propagated pose at this scan's stamp
  /// against the gated pose that replaces it, in the former's body frame.
  bool have_step{false};
  Increment step;
};

/**
 * @brief odom->base at wheel rate: lidar increments gated and accumulated at
 * each scan, carried forward between scans on the wheel increments.
 *
 * A complementary filter in its simplest form. The anchor is the gated pose at
 * the latest scan stamp, exactly what the gate alone produced; the published
 * pose is the anchor composed with the wheel motion since that stamp. Each scan
 * moves the anchor and so discards the wheel motion it overlaps, which is where
 * the wheels' drift goes; between scans the wheels supply what the lidar has
 * not yet answered.
 *
 * Every publication is stamped later than the one before. A scan's increment
 * arrives after wheel poses later than its own stamp have been published, so it
 * only moves the anchor; the next wheel sample publishes the corrected pose at
 * its own, newer stamp.
 *
 * The wheel motion is held to the envelope like everything else that reaches
 * odom->base. Propagation stops `max_propagation` seconds after the anchor, so
 * a lidar that stops answering leaves odom->base stale, as it was before
 * propagation, rather than running on wheels alone.
 */
class GatedOdometry
{
public:
  GatedOdometry(const GateLimits & limits, double max_propagation);

  ScanUpdate onScan(std::int64_t t, const Pose2 & icp);
  void onWheel(std::int64_t t, const Pose2 & wheel);

  /// The newest pose that can be published, if it is later than the last one
  /// this returned.
  bool next(std::int64_t & t, Pose2 & pose);

  const Pose2 & anchor() const {return anchor_;}

private:
  /// The anchor carried forward to `t` on the wheels, if the history spans it.
  bool propagate(std::int64_t t, Pose2 & out) const;

  GateLimits limits_;
  std::int64_t max_propagation_ns_;
  WheelHistory wheel_;

  bool have_scan_{false};
  std::int64_t scan_t_{0};
  Pose2 last_icp_;
  Pose2 anchor_;

  bool have_published_{false};
  std::int64_t published_t_{0};
};

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
 * odom->base is published at the wheel odometry's rate, not the lidar's (see
 * `GatedOdometry`): a control loop that waits for a transform newer than
 * `now()` otherwise runs at the scan rate. The wheel poses come from the same
 * topic `OdomTfRelay` inverts into kinematic_icp's prior.
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
  void onWheel(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void publishTransform();

  std::string odom_frame_;
  std::string base_frame_;
  GateLimits limits_;
  std::unique_ptr<GatedOdometry> odometry_;
  std::size_t rejected_{0};

  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_subscription_;
};

}  // namespace mote_nav

#endif  // MOTE_NAV__ICP_ODOM_GATE_HPP_
