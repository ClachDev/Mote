#include "mote_health/health_rollup.hpp"

#include <algorithm>
#include <charconv>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>

namespace mote_health
{

std::string level_name(uint8_t value)
{
  switch (value) {
    case level::OK: return "OK";
    case level::WARN: return "DEGRADED";
    case level::ERROR: return "FAULT";
    case level::STALE: return "STALE";
    default: return "UNKNOWN";
  }
}

uint8_t severity_level(
  const std::optional<std::string> & severity,
  const std::optional<bool> & critical,
  const std::string & name)
{
  // Back-compat with the boolean form.
  const std::string value =
    severity.value_or(critical.value_or(false) ? "critical" : "degraded");
  if (value == "critical") {return level::ERROR;}
  if (value == "degraded") {return level::WARN;}
  if (value == "info") {return level::OK;}
  throw std::invalid_argument("unknown severity '" + value + "' for " + name);
}

std::string one_line(const std::string & text)
{
  // The whitespace set is spelled out rather than taken from isspace(), whose
  // answer is the locale's. This is Python's str.split() for ASCII input, which
  // is what a diagnostic message is.
  static constexpr char WHITESPACE[] = " \t\n\r\v\f";
  std::string out;
  size_t cursor = 0;
  while (cursor < text.size()) {
    const size_t start = text.find_first_not_of(WHITESPACE, cursor);
    if (start == std::string::npos) {
      break;
    }
    size_t end = text.find_first_of(WHITESPACE, start);
    if (end == std::string::npos) {
      end = text.size();
    }
    if (!out.empty()) {
      out += ' ';
    }
    out.append(text, start, end - start);
    cursor = end;
  }
  return out;
}

std::string fixed1(double value)
{
  // to_chars rather than snprintf("%.1f"): printf's decimal point is the C
  // locale's, so on a machine whose LC_NUMERIC uses a comma this would publish
  // `rate_hz: 10,5` where Python published `10.5` — silently, and only on that
  // machine. to_chars never consults the locale.
  char buf[64];
  const auto result = std::to_chars(buf, buf + sizeof(buf), value, std::chars_format::fixed, 1);
  if (result.ec != std::errc()) {
    return "nan";
  }
  return std::string(buf, result.ptr);
}

TopicWatch::TopicWatch(
  std::string name, std::string topic, std::optional<double> min_rate, double timeout,
  uint8_t fault_level)
: name_(std::move(name)),
  topic_(std::move(topic)),
  min_rate_(min_rate),
  timeout_(timeout),
  fault_level_(fault_level)
{
}

void TopicWatch::on_msg(Clock::time_point now)
{
  last_stamp_ = now;
  ++count_;
}

Verdict TopicWatch::evaluate(double window, Clock::time_point now)
{
  const double rate = window > 0.0 ? static_cast<double>(count_) / window : 0.0;
  count_ = 0;

  Values values{{"topic", topic_}, {"rate_hz", fixed1(rate)}};
  if (!last_stamp_.has_value()) {
    return {fault_level_, "no messages received", values};
  }
  const double age = std::chrono::duration<double>(now - *last_stamp_).count();
  values.emplace_back("age_s", fixed1(age));
  if (age > timeout_) {
    return {fault_level_, "stale (" + fixed1(age) + "s > " + fixed1(timeout_) + "s)", values};
  }
  if (min_rate_.has_value() && rate < *min_rate_) {
    // A degraded rate never exceeds the subsystem's own fault level.
    const uint8_t degraded = std::min<uint8_t>(level::WARN, fault_level_);
    return {
      degraded, "slow (" + fixed1(rate) + " < " + fixed1(*min_rate_) + " Hz)", values};
  }
  return {level::OK, "ok", values};
}

TfWatch::TfWatch(
  std::string name, std::string parent, std::string child, double timeout, uint8_t fault_level)
: name_(std::move(name)),
  parent_(std::move(parent)),
  child_(std::move(child)),
  timeout_(timeout),
  fault_level_(fault_level)
{
}

Verdict TfWatch::evaluate(const std::optional<double> & age, const std::string & error) const
{
  Values values{{"transform", parent_ + "->" + child_}};
  if (!age.has_value()) {
    values.emplace_back("error", one_line(error).substr(0, 80));
    return {fault_level_, "unavailable", values};
  }
  values.emplace_back("age_s", fixed1(*age));
  if (*age > timeout_) {
    return {fault_level_, "stale (" + fixed1(*age) + "s > " + fixed1(timeout_) + "s)", values};
  }
  return {level::OK, "ok", values};
}

void Summary::add(const std::string & label, uint8_t value, const std::string & message)
{
  overall_ = std::max(overall_, value);
  if (value >= level::WARN) {
    faults_.push_back(label + " " + message);
  }
}

void Summary::add_capped(const std::string & label, uint8_t value, const std::string & message)
{
  if (value >= level::WARN) {
    faults_.push_back(label + " " + message);
    overall_ = std::max<uint8_t>(overall_, level::WARN);
  }
}

std::string Summary::text() const
{
  std::string out = level_name(overall_);
  if (!faults_.empty()) {
    out += ": ";
    for (size_t i = 0; i < faults_.size(); ++i) {
      if (i > 0) {
        out += ", ";
      }
      out += faults_[i];
    }
  }
  // One line, always: /health is meant for `ros2 topic echo` and log greps.
  return one_line(out);
}

}  // namespace mote_health
