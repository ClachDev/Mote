#include "mote_health/health_monitor.hpp"

#include <algorithm>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "tf2/exceptions.h"

#include "mote_health/mote_home.hpp"

namespace mote_health
{

// The roll-up header carries the levels by value so it stays message-free.
static_assert(level::OK == diagnostic_msgs::msg::DiagnosticStatus::OK);
static_assert(level::WARN == diagnostic_msgs::msg::DiagnosticStatus::WARN);
static_assert(level::ERROR == diagnostic_msgs::msg::DiagnosticStatus::ERROR);
static_assert(level::STALE == diagnostic_msgs::msg::DiagnosticStatus::STALE);

HealthMonitor::HealthMonitor(const rclcpp::NodeOptions & options)
: rclcpp::Node("health_monitor", options),
  config_(load_config(config_path()))
{
  for (const TopicEntry & entry : config_.topics) {
    // Subscribed generically: the watch only counts and timestamps, so the
    // callback never touches a field, and a serialized message never has to
    // become one. The type is still needed — it is what selects the
    // typesupport, only the delivered object changes.
    auto watch = entry.watch;
    topic_subs_.push_back(
      create_generic_subscription(
        watch->topic(), entry.type, rclcpp::QoS(10),
        [watch](std::shared_ptr<const rclcpp::SerializedMessage>) {watch->on_msg();}));
  }

  if (!config_.tf.empty()) {
    // The listener defaults to spin_thread=true, so /tf is taken on a thread of
    // its own rather than competing with the 1 Hz tick — where rclpy's default
    // was false. The buffer is mutex-protected and the listener's callback
    // group is deliberately not added to this node's executor, so the only
    // consequence is the extra thread. `tf_listener_` is declared after
    // `tf_buffer_` and so is destroyed before it.
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);
  }

  if (config_.subscribe_diagnostics) {
    // Deserialized, unlike the freshness watches above: this callback reads
    // status.name and status.level. It is also the one low-rate subscription
    // here, published once per aggregation period.
    diagnostics_sub_ = create_subscription<DiagnosticArray>(
      "diagnostics", rclcpp::QoS(10),
      [this](const DiagnosticArray & msg) {on_diagnostics(msg);});
  }

  selfcheck_path_ = mote_home::path("self_check_status.yaml");

  agg_pub_ = create_publisher<DiagnosticArray>("diagnostics_agg", 10);
  health_pub_ = create_publisher<std_msgs::msg::String>("health", 10);

  // Last, not in the member-init list: loading five typesupport libraries and
  // opening the subscriptions takes long enough that counting it into the first
  // window would report every topic's opening rate low.
  last_tick_ = Clock::now();

  // The node clock rather than the wall clock, as rclpy's create_timer uses.
  timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Duration::from_seconds(config_.period), [this]() {tick();});

  // systemd watchdog integration (no-op outside a Type=notify service).
  sd_.ready("health monitor up");
}

void HealthMonitor::on_diagnostics(const DiagnosticArray & msg)
{
  // Only the named first-party statuses feed the roll-up, matched exactly:
  // /diagnostics is a shared topic — controller_manager publishes its own
  // loop-jitter status there — and folding a third party's level in would
  // attribute it to one of ours. Other publishers stay visible on /diagnostics
  // itself.
  for (const DiagnosticStatus & status : msg.status) {
    if (
      std::find(
        config_.diagnostic_statuses.begin(), config_.diagnostic_statuses.end(),
        status.name) != config_.diagnostic_statuses.end())
    {
      forwarded_[status.name] = status;
    }
  }
}

void HealthMonitor::tick()
{
  const Clock::time_point now_steady = Clock::now();
  const double window = std::chrono::duration<double>(now_steady - last_tick_).count();
  last_tick_ = now_steady;
  const rclcpp::Time now_ros = now();

  std::vector<DiagnosticStatus> statuses;
  Summary summary;

  for (const TopicEntry & entry : config_.topics) {
    const Verdict verdict = entry.watch->evaluate(window, now_steady);
    statuses.push_back(
      status_msg(entry.watch->name(), verdict.level, verdict.message, verdict.values));
    summary.add(entry.watch->name(), verdict.level, verdict.message);
  }

  if (tf_buffer_) {
    for (const TfWatch & watch : config_.tf) {
      std::optional<double> age;
      std::string error;
      try {
        const auto tf = tf_buffer_->lookupTransform(
          watch.parent(), watch.child(), tf2::TimePointZero);
        age = (now_ros - rclcpp::Time(tf.header.stamp)).nanoseconds() / 1e9;
      } catch (const tf2::TransformException & exc) {
        error = exc.what();
      }
      const Verdict verdict = watch.evaluate(age, error);
      statuses.push_back(
        status_msg(watch.name(), verdict.level, verdict.message, verdict.values));
      summary.add(watch.name(), verdict.level, verdict.message);
    }
  }

  for (const std::string & name : config_.diagnostic_statuses) {
    const auto found = forwarded_.find(name);
    if (found == forwarded_.end()) {
      // A monitor that is not running is simply absent: its own liveness is not
      // this monitor's to assert.
      continue;
    }
    const DiagnosticStatus & status = found->second;
    statuses.push_back(status);
    summary.add(name == "system" ? "host" : name, status.level, one_line(status.message));
  }

  const std::optional<DiagnosticStatus> selfcheck = read_selfcheck();
  if (selfcheck.has_value()) {
    statuses.push_back(*selfcheck);
    // A failed pre-flight is informational at runtime (bringup would not have
    // started on a hard failure); surface it without forcing FAULT.
    summary.add_capped("self_check", selfcheck->level, selfcheck->message);
  }

  const std::string summary_text = summary.text();

  DiagnosticArray arr;
  arr.header.stamp = now_ros;
  arr.status.push_back(
    status_msg(
      "mote", summary.overall(), summary_text,
      {{"subsystems", std::to_string(statuses.size())}}));
  arr.status.insert(arr.status.end(), statuses.begin(), statuses.end());
  agg_pub_->publish(arr);

  std_msgs::msg::String health;
  health.data = summary_text;
  health_pub_->publish(health);

  // Prove liveness to systemd only after a successful publish.
  sd_.watchdog();

  if (summary.overall() >= level::WARN) {
    RCLCPP_WARN_THROTTLE(get_logger(), steady_clock_, 5000, "%s", summary_text.c_str());
  }
}

std::optional<HealthMonitor::DiagnosticStatus> HealthMonitor::read_selfcheck()
{
  std::error_code ec;
  const auto mtime = std::filesystem::last_write_time(selfcheck_path_, ec);
  if (ec) {
    return std::nullopt;
  }
  // Re-read only when the file changes so this stays cheap on every tick.
  if (selfcheck_mtime_.has_value() && *selfcheck_mtime_ == mtime) {
    return selfcheck_status_;
  }
  selfcheck_mtime_ = mtime;

  YAML::Node data;
  try {
    data = YAML::LoadFile(selfcheck_path_.string());
  } catch (const std::exception &) {
    return selfcheck_status_;
  }

  const bool passed = data["ok"] && data["ok"].as<bool>(false);
  std::vector<std::string> failed;
  const YAML::Node checks = data["checks"];
  for (size_t i = 0; checks && i < checks.size(); ++i) {
    const YAML::Node check = checks[i];
    if (!(check["passed"] && check["passed"].as<bool>(false))) {
      failed.push_back(check["name"] ? check["name"].as<std::string>("") : "");
    }
  }

  std::string message = "ready";
  if (!passed) {
    message = "failed: ";
    for (size_t i = 0; i < failed.size(); ++i) {
      message += (i > 0 ? ", " : "") + failed[i];
    }
  }
  selfcheck_status_ = status_msg(
    "self_check", passed ? level::OK : level::WARN, message,
    {{"at", data["timestamp"] ? data["timestamp"].as<std::string>("") : ""}});
  return selfcheck_status_;
}

HealthMonitor::DiagnosticStatus HealthMonitor::status_msg(
  const std::string & name, uint8_t value, const std::string & message, const Values & values)
{
  DiagnosticStatus status;
  status.name = name;
  status.level = value;
  status.message = message;
  status.hardware_id = "mote";
  status.values.reserve(values.size());
  for (const auto & [key, value] : values) {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = key;
    kv.value = value;
    status.values.push_back(std::move(kv));
  }
  return status;
}

}  // namespace mote_health
