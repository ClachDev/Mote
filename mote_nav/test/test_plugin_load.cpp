#include <gtest/gtest.h>

#include "dwb_core/trajectory_critic.hpp"
#include "pluginlib/class_loader.hpp"

TEST(PluginLoad, WheelSpeedLimitCriticResolves)
{
  pluginlib::ClassLoader<dwb_core::TrajectoryCritic> loader(
    "dwb_core", "dwb_core::TrajectoryCritic");

  dwb_core::TrajectoryCritic::Ptr critic;
  ASSERT_NO_THROW(
    critic = loader.createSharedInstance("mote_nav::WheelSpeedLimitCritic"));
  ASSERT_NE(critic, nullptr);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
