// health.yaml is parsed into watches, and the per-robot override is honoured.

#include <gtest/gtest.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <set>
#include <string>

#include "mote_health/config.hpp"
#include "mote_health/mote_home.hpp"

using mote_health::Config;
using mote_health::default_diagnostic_statuses;
using mote_health::config_path;
using mote_health::load_config;
using mote_health::parse_config;
namespace level = mote_health::level;

TEST(Config, SpecsBecomeWatches)
{
  const Config cfg = parse_config(
    R"(
period: 0.5
topics:
  - name: scan
    topic: /scan
    type: sensor_msgs/msg/LaserScan
    min_rate: 5.0
    timeout: 2.0
    severity: critical
  - name: camera
    topic: /image_raw/compressed
    type: sensor_msgs/msg/CompressedImage
    severity: degraded
tf:
  - name: localization
    parent: map
    child: odom
    timeout: 5.0
    severity: info
)");
  EXPECT_DOUBLE_EQ(cfg.period, 0.5);
  ASSERT_EQ(cfg.topics.size(), 2u);
  EXPECT_EQ(cfg.topics[0].type, "sensor_msgs/msg/LaserScan");
  EXPECT_EQ(cfg.topics[0].watch->name(), "scan");
  EXPECT_EQ(cfg.topics[0].watch->fault_level(), level::ERROR);
  EXPECT_EQ(cfg.topics[1].watch->fault_level(), level::WARN);
  ASSERT_EQ(cfg.tf.size(), 1u);
  EXPECT_EQ(cfg.tf[0].fault_level(), level::OK);
  // Defaults, unstated above.
  EXPECT_TRUE(cfg.subscribe_diagnostics);
  EXPECT_EQ(cfg.diagnostic_statuses, default_diagnostic_statuses());
}

TEST(Config, DefaultsWhenTheFileSaysNothing)
{
  const Config cfg = parse_config("topics: []\ntf: []\n");
  EXPECT_DOUBLE_EQ(cfg.period, 1.0);
  EXPECT_TRUE(cfg.topics.empty());
  EXPECT_TRUE(cfg.tf.empty());
}

TEST(Config, AnEmptyFileIsRefusedRatherThanWatchingNothing)
{
  // The trigger is a truncated write or an override created before it was
  // edited — `override_path` only tests that the file exists. Accepting it
  // would give a monitor reporting `OK` with nothing under it, which is the
  // failure the monitor exists to prevent.
  EXPECT_THROW(parse_config(""), std::invalid_argument);
  EXPECT_THROW(parse_config("# nothing yet\n"), std::invalid_argument);
  EXPECT_THROW(parse_config("- not\n- a mapping\n"), std::invalid_argument);
  // An explicit empty watch list is a different statement, and is allowed.
  EXPECT_NO_THROW(parse_config("topics: []\ntf: []\n"));
}

TEST(Config, AKeyWrittenWithNoValueIsAbsent)
{
  // yaml-cpp hands back a truthy Null node whose as<string>() is "null", which
  // would be refused as an unknown severity where PyYAML's None was read as
  // absent.
  const Config cfg = parse_config(
    "topics:\n  - name: scan\n    topic: /scan\n"
    "    type: sensor_msgs/msg/LaserScan\n    severity:\n    min_rate:\n    timeout:\n");
  ASSERT_EQ(cfg.topics.size(), 1u);
  // severity absent -> the boolean form's default, which is `critical: false`.
  EXPECT_EQ(cfg.topics[0].watch->fault_level(), level::WARN);
}

TEST(Config, UnknownSeverityIsRefusedNamingTheSubsystem)
{
  // A monitor that guesses at how loudly a subsystem reports is worse than one
  // that refuses to start, and the message has to say which entry.
  try {
    parse_config(
      "topics:\n  - name: scan\n    topic: /scan\n"
      "    type: sensor_msgs/msg/LaserScan\n    severity: catastrophic\n");
    FAIL() << "a severity with no reading was accepted";
  } catch (const std::invalid_argument & exc) {
    EXPECT_NE(std::string(exc.what()).find("scan"), std::string::npos);
    EXPECT_NE(std::string(exc.what()).find("catastrophic"), std::string::npos);
  }
}

TEST(Config, MissingTopicIsRefused)
{
  EXPECT_THROW(
    parse_config("topics:\n  - name: scan\n    type: sensor_msgs/msg/LaserScan\n"),
    std::invalid_argument);
}

TEST(Config, DiagnosticStatusesAreOverridable)
{
  const Config cfg = parse_config("diagnostic_statuses: [slip]\nsubscribe_diagnostics: false\n");
  EXPECT_EQ(cfg.diagnostic_statuses, std::vector<std::string>{"slip"});
  EXPECT_FALSE(cfg.subscribe_diagnostics);
}

TEST(Config, OverrideHonoursMoteHome)
{
  // The per-robot override must resolve through MOTE_HOME, not a literal
  // ~/.mote. A hardcoded ~/.mote looks identical on a robot, where the two are
  // the same path, and is wrong for the sim and for tests.
  const auto tmp = std::filesystem::temp_directory_path() /
    ("mote_health_cfg_" + std::to_string(::getpid()));
  std::filesystem::create_directories(tmp);
  {
    std::ofstream out(tmp / "health.yaml");
    out << "period: 9.5\ntopics: []\ntf: []\n";
  }
  setenv("MOTE_HOME", tmp.c_str(), 1);
  EXPECT_EQ(config_path(), (tmp / "health.yaml").string());
  EXPECT_DOUBLE_EQ(load_config(config_path()).period, 9.5);

  // Empty MOTE_HOME: no override present, so the packaged default is used and
  // it defines the real subsystems.
  const auto empty = tmp / "empty";
  std::filesystem::create_directories(empty);
  setenv("MOTE_HOME", empty.c_str(), 1);
  const Config packaged = load_config(config_path());
  std::set<std::string> names;
  for (const auto & entry : packaged.topics) {
    names.insert(entry.watch->name());
  }
  EXPECT_TRUE(names.count("scan"));
  EXPECT_TRUE(names.count("joint_states"));
  // The packaged file is what ties the roll-up to the monitors publishing on
  // the shared /diagnostics; mote_bringup's test_health_config.py holds the
  // other end, that those names are the ones those nodes publish.
  EXPECT_EQ(packaged.diagnostic_statuses, default_diagnostic_statuses());

  unsetenv("MOTE_HOME");
  std::filesystem::remove_all(tmp);
}
