// The health monitor as a running node: generic subscriptions still report.
//
// test_health_rollup.cpp covers the roll-up decisions as plain function calls.
// What it cannot cover is the delivery underneath them. The watched topics are
// subscribed generically — the callback only counts arrivals, so nothing is
// gained by building a message first — and that is exactly the kind of change
// which fails silently: a subscription that delivers nothing leaves a node
// which still runs, still publishes on time, and reports every subsystem as
// missing. So this drives the real node with a real publisher and asserts both
// halves: that a serialized arrival still reaches the summary, and that a
// forwarded /diagnostics status still degrades it.

#include <gtest/gtest.h>

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"

#include "mote_health/health_monitor.hpp"

using namespace std::chrono_literals;
using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;

namespace
{

constexpr const char * CONFIG = R"(
period: 0.2
topics:
  - name: scan
    topic: /scan
    type: sensor_msgs/msg/LaserScan
    min_rate: 5.0
    timeout: 2.0
    severity: critical
tf: []
subscribe_diagnostics: true
)";

constexpr double SCAN_RATE = 20.0;

/// Publishes scans at a rate comfortably above the configured floor, and a
/// `system` status on the shared /diagnostics the roll-up must lift.
class Fixture : public rclcpp::Node
{
public:
  Fixture()
  : rclcpp::Node("fixture")
  {
    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
    diag_pub_ = create_publisher<DiagnosticArray>("/diagnostics", 10);
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / SCAN_RATE), [this]() {publish();});

    health_sub_ = create_subscription<std_msgs::msg::String>(
      "/health", 10,
      [this](const std_msgs::msg::String & msg) {summaries.push_back(msg.data);});
    agg_sub_ = create_subscription<DiagnosticArray>(
      "/diagnostics_agg", 10, [this](const DiagnosticArray & msg) {aggregates.push_back(msg);});
  }

  /// A named first-party status, published with the next scan.
  void report(const std::string & name, uint8_t level, const std::string & message)
  {
    DiagnosticStatus status;
    status.name = name;
    status.level = level;
    status.message = message;
    pending_ = status;
  }

  int published{0};
  std::vector<std::string> summaries;
  std::vector<DiagnosticArray> aggregates;

private:
  void publish()
  {
    sensor_msgs::msg::LaserScan scan;
    scan.header.stamp = now();
    scan.header.frame_id = "laser";
    scan.ranges.assign(360, 1.0F);
    scan_pub_->publish(scan);
    ++published;

    if (pending_.has_value()) {
      DiagnosticArray arr;
      arr.header.stamp = now();
      arr.status.push_back(*pending_);
      diag_pub_->publish(arr);
    }
  }

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr health_sub_;
  rclcpp::Subscription<DiagnosticArray>::SharedPtr agg_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::optional<DiagnosticStatus> pending_;
};

class HealthMonitorNodeTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    // A stray ROS_DOMAIN_ID here would put this test on the same graph as a
    // real robot. Claim an unused domain and stay on localhost.
    std::random_device rd;
    setenv("ROS_DOMAIN_ID", std::to_string(64 + rd() % 137).c_str(), 1);
    setenv("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST", 1);

    home_ = std::filesystem::temp_directory_path() /
      ("mote_health_node_" + std::to_string(::getpid()));
    std::filesystem::create_directories(home_);
    std::ofstream(home_ / "health.yaml") << CONFIG;
    setenv("MOTE_HOME", home_.c_str(), 1);

    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
    unsetenv("MOTE_HOME");
    std::filesystem::remove_all(home_);
  }

  /// Spin both nodes for `seconds` of wall time.
  static void spin(
    const std::shared_ptr<rclcpp::Node> & a, const std::shared_ptr<rclcpp::Node> & b,
    double seconds)
  {
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(a);
    executor.add_node(b);
    const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
    while (std::chrono::steady_clock::now() < deadline) {
      executor.spin_once(20ms);
    }
  }

  static std::filesystem::path home_;
};

std::filesystem::path HealthMonitorNodeTest::home_;

}  // namespace

TEST_F(HealthMonitorNodeTest, ASerializedScanStillReachesTheSummary)
{
  auto monitor = std::make_shared<mote_health::HealthMonitor>();
  auto fixture = std::make_shared<Fixture>();
  spin(monitor, fixture, 2.0);

  ASSERT_GT(fixture->published, 10) << "the publisher itself never ran";
  ASSERT_FALSE(fixture->summaries.empty()) << "nothing published on /health";
  EXPECT_EQ(fixture->summaries.back(), "OK");

  // The aggregate keeps its shape: the mote roll-up first, then one status per
  // subsystem, and the measured rate is a real one rather than zero.
  const DiagnosticArray & last = fixture->aggregates.back();
  ASSERT_EQ(last.status.size(), 2u);
  EXPECT_EQ(last.status[0].name, "mote");
  EXPECT_EQ(last.status[0].level, DiagnosticStatus::OK);
  EXPECT_EQ(last.status[0].hardware_id, "mote");
  EXPECT_EQ(last.status[1].name, "scan");
  for (const auto & kv : last.status[1].values) {
    if (kv.key == "rate_hz") {
      EXPECT_GE(std::stod(kv.value), 5.0) << kv.value;
    }
  }
}

TEST_F(HealthMonitorNodeTest, PublishesAtTheConfiguredCadence)
{
  // period: 0.2 above, so ~5 summaries a second. A monitor whose timer never
  // fires reports nothing while looking perfectly alive.
  auto monitor = std::make_shared<mote_health::HealthMonitor>();
  auto fixture = std::make_shared<Fixture>();
  spin(monitor, fixture, 2.0);
  EXPECT_GE(fixture->summaries.size(), 6u) << fixture->summaries.size();
  EXPECT_EQ(fixture->summaries.size(), fixture->aggregates.size());
}

TEST_F(HealthMonitorNodeTest, ANamedDiagnosticStatusIsLiftedIntoTheRollUp)
{
  auto monitor = std::make_shared<mote_health::HealthMonitor>();
  auto fixture = std::make_shared<Fixture>();
  fixture->report("system", DiagnosticStatus::WARN, "under-voltage");
  spin(monitor, fixture, 2.0);

  const DiagnosticArray & last = fixture->aggregates.back();
  ASSERT_EQ(last.status.size(), 3u);
  EXPECT_EQ(last.status[2].name, "system");
  // `system` is reported to an operator as `host`.
  EXPECT_EQ(last.status[0].message, "DEGRADED: host under-voltage");
  EXPECT_EQ(fixture->summaries.back(), "DEGRADED: host under-voltage");
}

TEST_F(HealthMonitorNodeTest, AnUnnamedDiagnosticStatusIsIgnored)
{
  // /diagnostics is shared — controller_manager publishes its own loop-jitter
  // status there — so a third party's level must not become the robot's.
  auto monitor = std::make_shared<mote_health::HealthMonitor>();
  auto fixture = std::make_shared<Fixture>();
  fixture->report("mote_hardware", DiagnosticStatus::ERROR, "High execution jitter");
  spin(monitor, fixture, 2.0);

  EXPECT_EQ(fixture->summaries.back(), "OK");
  EXPECT_EQ(fixture->aggregates.back().status.size(), 2u);
}
