// The MOTE_HOME rule, which is mote_bringup/mote_home.py's and must stay it.

#include <gtest/gtest.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>

#include <unistd.h>

#include "mote_health/mote_home.hpp"

namespace mote_home = mote_health::mote_home;

TEST(MoteHome, DefaultsToTildeMoteExpanded)
{
  unsetenv("MOTE_HOME");
  setenv("HOME", "/home/somebody", 1);
  EXPECT_EQ(mote_home::dir(), "/home/somebody/.mote");
  EXPECT_EQ(mote_home::path("health.yaml"), "/home/somebody/.mote/health.yaml");
}

TEST(MoteHome, EnvironmentWins)
{
  setenv("MOTE_HOME", "/srv/state", 1);
  EXPECT_EQ(mote_home::dir(), "/srv/state");
  EXPECT_EQ(mote_home::path("robot.yaml"), "/srv/state/robot.yaml");
  unsetenv("MOTE_HOME");
}

TEST(MoteHome, EnvironmentIsAlsoTildeExpanded)
{
  // Python resolves MOTE_HOME through the same expanduser, so `MOTE_HOME=~/x`
  // must not become a directory literally named "~".
  setenv("HOME", "/home/somebody", 1);
  setenv("MOTE_HOME", "~/state", 1);
  EXPECT_EQ(mote_home::dir(), "/home/somebody/state");
  unsetenv("MOTE_HOME");
}

TEST(MoteHome, OverrideFallsBackToThePackagedDefault)
{
  const auto tmp = std::filesystem::temp_directory_path() /
    ("mote_home_test_" + std::to_string(::getpid()));
  std::filesystem::create_directories(tmp);
  setenv("MOTE_HOME", tmp.c_str(), 1);

  EXPECT_EQ(mote_home::override_path("health.yaml", "/packaged/health.yaml"),
    "/packaged/health.yaml");

  std::ofstream(tmp / "health.yaml") << "period: 1.0\n";
  EXPECT_EQ(
    mote_home::override_path("health.yaml", "/packaged/health.yaml"),
    (tmp / "health.yaml").string());

  unsetenv("MOTE_HOME");
  std::filesystem::remove_all(tmp);
}
