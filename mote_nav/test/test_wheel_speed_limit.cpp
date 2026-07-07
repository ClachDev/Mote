#include <gtest/gtest.h>

#include "mote_nav/wheel_speed_limit_critic.hpp"

namespace
{
constexpr double kLimit = 0.218;
constexpr double kSeparation = 0.22;
}  // namespace

TEST(WheelSpeedMath, StraightDriveAtBoundary)
{
  const double s = mote_nav::maxWheelSpeed(0.218, 0.0, kSeparation);
  EXPECT_NEAR(s, 0.218, 1e-9);
  EXPECT_FALSE(s > kLimit);
}

TEST(WheelSpeedMath, InPlaceTurnFeasible)
{
  const double s = mote_nav::maxWheelSpeed(0.0, 1.5, kSeparation);
  EXPECT_NEAR(s, 0.165, 1e-9);
  EXPECT_FALSE(s > kLimit);
}

TEST(WheelSpeedMath, CombinedInfeasible)
{
  const double s = mote_nav::maxWheelSpeed(0.3, 1.5, kSeparation);
  EXPECT_NEAR(s, 0.465, 1e-9);
  EXPECT_TRUE(s > kLimit);
}

TEST(WheelSpeedMath, SymmetricInAngularVelocity)
{
  EXPECT_DOUBLE_EQ(
    mote_nav::maxWheelSpeed(0.1, 1.0, kSeparation),
    mote_nav::maxWheelSpeed(0.1, -1.0, kSeparation));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
