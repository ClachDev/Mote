// The roll-up rules: freshness, rate, severity, and the one-line summary.
//
// Deliberately free of rclcpp, tf2 and yaml-cpp. These are the decisions the
// monitor exists to make — a stale critical subsystem is a FAULT, a stale
// non-critical one is DEGRADED, a fresh-but-slow one is DEGRADED — and they are
// what `test_health_rollup.cpp` holds. The node contributes only the arrivals
// and the transform lookups.

#ifndef MOTE_HEALTH__HEALTH_ROLLUP_HPP_
#define MOTE_HEALTH__HEALTH_ROLLUP_HPP_

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace mote_health
{

/// diagnostic_msgs::msg::DiagnosticStatus levels, by value.
///
/// Spelled out rather than included so this header stays message-free; the node
/// static_asserts them against the real constants.
namespace level
{
constexpr uint8_t OK = 0;
constexpr uint8_t WARN = 1;
constexpr uint8_t ERROR = 2;
constexpr uint8_t STALE = 3;
}  // namespace level

using Clock = std::chrono::steady_clock;

/// A DiagnosticStatus's key/value pairs, in the order they are published.
using Values = std::vector<std::pair<std::string, std::string>>;

struct Verdict
{
  uint8_t level;
  std::string message;
  Values values;
};

/// "OK" / "DEGRADED" / "FAULT" / "STALE", else "UNKNOWN".
std::string level_name(uint8_t level);

/// How much a missing/stale subsystem degrades the robot summary.
///
/// "info" reports the subsystem without degrading it — for edges legitimately
/// absent in a healthy state (map->odom exists only once a mission localises).
/// Throws std::invalid_argument on a severity with no reading, naming the
/// subsystem, because a typo would otherwise silently pick a level.
uint8_t severity_level(
  const std::optional<std::string> & severity,
  const std::optional<bool> & critical,
  const std::string & name);

/// Collapse whitespace so a summary stays a single line.
///
/// Third-party diagnostic messages can carry embedded newlines, which would
/// otherwise shatter the one-line /health summary into several messages.
std::string one_line(const std::string & text);

/// One decimal place, as Python's f"{x:.1f}" writes it.
std::string fixed1(double value);

/// Freshness + rate tracker for one subscribed topic.
class TopicWatch
{
public:
  TopicWatch(
    std::string name, std::string topic, std::optional<double> min_rate, double timeout,
    uint8_t fault_level);

  /// Record an arrival. The payload is never read, and arrives serialized.
  void on_msg(Clock::time_point now = Clock::now());

  /// Verdict for the window just closed; resets the count.
  Verdict evaluate(double window, Clock::time_point now = Clock::now());

  const std::string & name() const {return name_;}
  const std::string & topic() const {return topic_;}
  uint8_t fault_level() const {return fault_level_;}
  uint64_t count() const {return count_;}
  bool ever_seen() const {return last_stamp_.has_value();}

  /// Backdate the last arrival, so staleness can be tested without waiting.
  void set_last_stamp(std::optional<Clock::time_point> stamp) {last_stamp_ = stamp;}

private:
  std::string name_;
  std::string topic_;
  std::optional<double> min_rate_;
  double timeout_;
  uint8_t fault_level_;
  std::optional<Clock::time_point> last_stamp_;
  uint64_t count_{0};
};

/// Freshness tracker for one TF edge.
class TfWatch
{
public:
  TfWatch(std::string name, std::string parent, std::string child, double timeout,
    uint8_t fault_level);

  /// `age` is the transform's age in seconds, or nullopt when the lookup failed
  /// — in which case `error` says why. The lookup itself is the node's, so this
  /// stays testable without a TF tree.
  Verdict evaluate(const std::optional<double> & age, const std::string & error) const;

  const std::string & name() const {return name_;}
  const std::string & parent() const {return parent_;}
  const std::string & child() const {return child_;}
  uint8_t fault_level() const {return fault_level_;}

private:
  std::string name_;
  std::string parent_;
  std::string child_;
  double timeout_;
  uint8_t fault_level_;
};

/// The robot-level summary, accumulated one subsystem at a time.
class Summary
{
public:
  /// Fold in a subsystem: it raises the overall level, and names itself in the
  /// summary once it is WARN or worse.
  void add(const std::string & label, uint8_t level, const std::string & message);

  /// Fold in a subsystem that may report but never exceed WARN.
  ///
  /// The pre-flight verdict is the only such input: bringup would not have
  /// started on a hard failure, so a failed self-check is informational at
  /// runtime rather than a reason to call the robot faulted.
  void add_capped(const std::string & label, uint8_t level, const std::string & message);

  uint8_t overall() const {return overall_;}

  /// "OK", or "DEGRADED: scan stale (3.0s > 2.0s), host ...". Always one line.
  std::string text() const;

private:
  uint8_t overall_{level::OK};
  std::vector<std::string> faults_;
};

}  // namespace mote_health

#endif  // MOTE_HEALTH__HEALTH_ROLLUP_HPP_
