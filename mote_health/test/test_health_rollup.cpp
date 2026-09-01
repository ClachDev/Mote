// health_monitor freshness/rate logic drives the OK/DEGRADED/FAULT roll-up.
//
// Ported from mote_bringup/test/test_health_monitor.py, case for case: these
// decisions are what the monitor is, and they did not change when it stopped
// being Python.

#include <gtest/gtest.h>

#include <clocale>
#include <optional>
#include <string>

#include "mote_health/health_rollup.hpp"

using mote_health::Clock;
using mote_health::Summary;
using mote_health::TfWatch;
using mote_health::TopicWatch;
using mote_health::Verdict;
using mote_health::one_line;
using mote_health::severity_level;
namespace level = mote_health::level;

namespace
{

TopicWatch make_watch(
  const std::string & severity = "critical", std::optional<double> min_rate = 5.0,
  double timeout = 2.0)
{
  return TopicWatch(
    "scan", "/scan", min_rate, timeout,
    severity_level(severity, std::nullopt, "scan"));
}

}  // namespace

TEST(TopicWatch, NeverReceivedCriticalIsFault)
{
  const Verdict verdict = make_watch("critical").evaluate(1.0);
  EXPECT_EQ(verdict.level, level::ERROR);
  EXPECT_NE(verdict.message.find("no messages"), std::string::npos);
}

TEST(TopicWatch, NeverReceivedDegradedIsWarn)
{
  EXPECT_EQ(make_watch("degraded").evaluate(1.0).level, level::WARN);
}

TEST(TopicWatch, FreshAndFastIsOk)
{
  TopicWatch watch = make_watch();
  for (int i = 0; i < 10; ++i) {
    watch.on_msg();
  }
  const Verdict verdict = watch.evaluate(1.0);  // 10 msgs / 1s = 10 Hz
  EXPECT_EQ(verdict.level, level::OK);
  EXPECT_EQ(verdict.message, "ok");
  ASSERT_EQ(verdict.values[1].first, "rate_hz");
  EXPECT_GE(std::stod(verdict.values[1].second), 5.0);
}

TEST(TopicWatch, FreshButSlowIsDegraded)
{
  TopicWatch watch = make_watch();
  watch.on_msg();  // a single message this window -> ~1 Hz
  const Verdict verdict = watch.evaluate(1.0);
  EXPECT_EQ(verdict.level, level::WARN);
  EXPECT_NE(verdict.message.find("slow"), std::string::npos);
}

TEST(TopicWatch, StaleCriticalIsFault)
{
  TopicWatch watch = make_watch("critical", 5.0, 2.0);
  watch.on_msg();
  watch.set_last_stamp(Clock::now() - std::chrono::seconds(10));  // last seen 10s ago
  const Verdict verdict = watch.evaluate(1.0);
  EXPECT_EQ(verdict.level, level::ERROR);
  EXPECT_NE(verdict.message.find("stale"), std::string::npos);
}

TEST(TopicWatch, StaleDegradedIsWarn)
{
  TopicWatch watch = make_watch("degraded", 5.0, 2.0);
  watch.on_msg();
  watch.set_last_stamp(Clock::now() - std::chrono::seconds(10));
  EXPECT_EQ(watch.evaluate(1.0).level, level::WARN);
}

TEST(TopicWatch, RecoveryBackToOk)
{
  TopicWatch watch = make_watch("critical", 5.0, 2.0);
  watch.set_last_stamp(Clock::now() - std::chrono::seconds(10));
  EXPECT_EQ(watch.evaluate(1.0).level, level::ERROR);
  for (int i = 0; i < 10; ++i) {
    watch.on_msg();
  }
  EXPECT_EQ(watch.evaluate(1.0).level, level::OK);
}

TEST(TopicWatch, InfoSeverityNeverDegrades)
{
  // An `info` subsystem is reported but must not degrade the robot summary.
  // map->odom is legitimately absent when only the hardware base runs, so
  // scoring it would leave a healthy idle robot permanently DEGRADED.
  TopicWatch watch = make_watch("info", 5.0, 2.0);
  const Verdict never = watch.evaluate(1.0);
  EXPECT_EQ(never.level, level::OK);
  EXPECT_NE(never.message.find("no messages"), std::string::npos);
  watch.on_msg();  // fresh but slow
  EXPECT_EQ(watch.evaluate(1.0).level, level::OK);
}

TEST(TopicWatch, ZeroWindowScoresZeroRateWithoutDividing)
{
  // The first tick after a clock jump can close a zero-length window; a rate of
  // count/0 would be inf and read as a healthy topic.
  TopicWatch watch = make_watch();
  watch.on_msg();
  const Verdict verdict = watch.evaluate(0.0);
  EXPECT_EQ(verdict.values[1].second, "0.0");
  EXPECT_EQ(verdict.level, level::WARN);
}

TEST(TopicWatch, ValuesAreOrderedTopicRateAge)
{
  // /diagnostics_agg consumers read these positionally in the fleet layer, and
  // the Python monitor emitted them in insertion order.
  TopicWatch watch = make_watch();
  watch.on_msg();
  const Verdict verdict = watch.evaluate(1.0);
  ASSERT_EQ(verdict.values.size(), 3u);
  EXPECT_EQ(verdict.values[0].first, "topic");
  EXPECT_EQ(verdict.values[0].second, "/scan");
  EXPECT_EQ(verdict.values[1].first, "rate_hz");
  EXPECT_EQ(verdict.values[2].first, "age_s");
}

TEST(TfWatch, InfoSeverityUnavailableIsOk)
{
  const TfWatch watch(
    "localization", "map", "odom", 2.0, severity_level("info", std::nullopt, "localization"));
  EXPECT_EQ(watch.fault_level(), level::OK);
  const Verdict verdict = watch.evaluate(std::nullopt, "\"map\" does not exist");
  EXPECT_EQ(verdict.level, level::OK);
  EXPECT_EQ(verdict.message, "unavailable");
}

TEST(TfWatch, UnavailableCriticalIsFaultAndCarriesTheReason)
{
  const TfWatch watch(
    "odometry", "odom", "base_footprint", 2.0,
    severity_level("critical", std::nullopt, "odometry"));
  const Verdict verdict = watch.evaluate(std::nullopt, "canTransform\n  failed");
  EXPECT_EQ(verdict.level, level::ERROR);
  ASSERT_EQ(verdict.values.size(), 2u);
  EXPECT_EQ(verdict.values[0].second, "odom->base_footprint");
  EXPECT_EQ(verdict.values[1].first, "error");
  // Collapsed to one line and truncated, as the summary needs it.
  EXPECT_EQ(verdict.values[1].second, "canTransform failed");
}

TEST(TfWatch, ErrorIsTruncatedToEightyCharacters)
{
  const TfWatch watch("odometry", "odom", "base_footprint", 2.0, level::ERROR);
  const Verdict verdict = watch.evaluate(std::nullopt, std::string(200, 'x'));
  EXPECT_EQ(verdict.values[1].second.size(), 80u);
}

TEST(TfWatch, StaleAndFresh)
{
  const TfWatch watch("odometry", "odom", "base_footprint", 2.0, level::ERROR);
  const Verdict stale = watch.evaluate(3.04, "");
  EXPECT_EQ(stale.level, level::ERROR);
  EXPECT_EQ(stale.message, "stale (3.0s > 2.0s)");
  const Verdict fresh = watch.evaluate(0.5, "");
  EXPECT_EQ(fresh.level, level::OK);
  EXPECT_EQ(fresh.message, "ok");
}

TEST(Severity, BooleanCriticalStillSupported)
{
  EXPECT_EQ(severity_level(std::nullopt, true, "x"), level::ERROR);
  EXPECT_EQ(severity_level(std::nullopt, false, "x"), level::WARN);
  // Absent entirely is the same as critical: false.
  EXPECT_EQ(severity_level(std::nullopt, std::nullopt, "x"), level::WARN);
}

TEST(Severity, UnknownSeverityRejected)
{
  EXPECT_THROW(severity_level("catastrophic", std::nullopt, "x"), std::invalid_argument);
}

TEST(OneLine, CollapsesEmbeddedNewlines)
{
  // A third-party diagnostic message must not break the /health summary.
  // controller_manager publishes a multi-line "High execution jitter" status on
  // the shared /diagnostics topic, and embedding such a message verbatim would
  // split the single-line summary across several messages.
  const std::string messy =
    "High execution jitter or mean error :\n[ mote_hardware  mote_hardware ]\n";
  EXPECT_EQ(
    one_line(messy),
    "High execution jitter or mean error : [ mote_hardware mote_hardware ]");
  EXPECT_EQ(one_line(messy).find('\n'), std::string::npos);
}

TEST(Formatting, NumbersDoNotFollowTheLocale)
{
  // The Python monitor formatted with f"{x:.1f}", which has no locale. printf's
  // "%.1f" does, so under LC_NUMERIC=de_DE it would publish `rate_hz: 10,5` and
  // `slow (1,0 < 5,0 Hz)` — on that machine only, and without failing anything.
  // `fixed1` uses std::to_chars, which never consults the locale, and
  // `one_line` scans an explicit whitespace set rather than calling isspace().
  const char * candidates[] = {"de_DE.UTF-8", "de_DE.utf8", "fr_FR.UTF-8", "C"};
  const char * applied = nullptr;
  for (const char * name : candidates) {
    if (std::setlocale(LC_ALL, name) != nullptr) {
      applied = name;
      break;
    }
  }
  ASSERT_NE(applied, nullptr);

  EXPECT_EQ(mote_health::fixed1(10.5), "10.5") << "locale " << applied;
  EXPECT_EQ(mote_health::fixed1(0.0), "0.0");
  EXPECT_EQ(mote_health::fixed1(-1.25), "-1.2") << "half-even, as Python rounds";
  EXPECT_EQ(one_line("a\tb\nc\v d"), "a b c d");

  TopicWatch watch = make_watch();
  watch.on_msg();
  EXPECT_EQ(watch.evaluate(1.0).message, "slow (1.0 < 5.0 Hz)");

  std::setlocale(LC_ALL, "C");
}

TEST(SummaryText, HealthyRobotIsBareOk)
{
  Summary summary;
  summary.add("scan", level::OK, "ok");
  summary.add("camera", level::OK, "ok");
  EXPECT_EQ(summary.overall(), level::OK);
  EXPECT_EQ(summary.text(), "OK");
}

TEST(SummaryText, FaultsAreNamedInOrder)
{
  Summary summary;
  summary.add("scan", level::ERROR, "stale (3.0s > 2.0s)");
  summary.add("camera", level::WARN, "slow (1.0 < 5.0 Hz)");
  summary.add("host", level::OK, "ok");
  EXPECT_EQ(summary.overall(), level::ERROR);
  EXPECT_EQ(
    summary.text(),
    "FAULT: scan stale (3.0s > 2.0s), camera slow (1.0 < 5.0 Hz)");
}

TEST(SummaryText, SelfCheckReportsButNeverFaults)
{
  // A failed pre-flight is informational at runtime: bringup would not have
  // started on a hard failure, so it must not take the robot to FAULT.
  Summary summary;
  summary.add_capped("self_check", level::ERROR, "failed: lidar");
  EXPECT_EQ(summary.overall(), level::WARN);
  EXPECT_EQ(summary.text(), "DEGRADED: self_check failed: lidar");
}

TEST(SummaryText, PassingSelfCheckAddsNothing)
{
  Summary summary;
  summary.add_capped("self_check", level::OK, "ready");
  EXPECT_EQ(summary.text(), "OK");
}
