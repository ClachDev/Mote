#ifndef MOTE_HEALTH__HEALTH_MONITOR_HPP_
#define MOTE_HEALTH__HEALTH_MONITOR_HPP_

#include <filesystem>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "mote_health/config.hpp"
#include "mote_health/health_rollup.hpp"
#include "mote_health/sd_notify.hpp"

namespace mote_health
{

/// Robot-level health monitor.
///
/// Watches the liveness of the safety-critical subsystems (lidar scan, filtered
/// scan, wheel/joint feedback, odometry TF) plus non-critical ones (camera,
/// localisation TF) and folds in the host status published by system_monitor.
/// Every `period` seconds it publishes /diagnostics_agg (one DiagnosticStatus
/// per subsystem plus a rolled-up `mote` status) and /health (a single
/// human-readable OK / DEGRADED: ... / FAULT: ... line).
///
/// It is C++ rather than Python because what a monitor costs is the number of
/// times it is woken, not what it computes: measured on mote-01, an rclpy
/// wake-up is ~0.78 ms of CPU per message and this node consumes ~152 msg/s.
/// See docs/tuning/2026-08-11-monitor-cpu.md.
class HealthMonitor : public rclcpp::Node
{
public:
  explicit HealthMonitor(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /// Run one aggregation period. Public so a test can drive it directly.
  void tick();

private:
  using DiagnosticArray = diagnostic_msgs::msg::DiagnosticArray;
  using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;

  void on_diagnostics(const DiagnosticArray & msg);
  std::optional<DiagnosticStatus> read_selfcheck();
  static DiagnosticStatus status_msg(
    const std::string & name, uint8_t value, const std::string & message, const Values & values);

  Config config_;
  std::vector<std::shared_ptr<rclcpp::GenericSubscription>> topic_subs_;
  rclcpp::Subscription<DiagnosticArray>::SharedPtr diagnostics_sub_;
  std::map<std::string, DiagnosticStatus> forwarded_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::filesystem::path selfcheck_path_;
  std::optional<std::filesystem::file_time_type> selfcheck_mtime_;
  std::optional<DiagnosticStatus> selfcheck_status_;

  rclcpp::Publisher<DiagnosticArray>::SharedPtr agg_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr health_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  Clock::time_point last_tick_{};
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  SdNotifier sd_;
};

}  // namespace mote_health

#endif  // MOTE_HEALTH__HEALTH_MONITOR_HPP_
