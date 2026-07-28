#include <gtest/gtest.h>

#include <cmath>

#include "mote_nav/icp_odom_gate.hpp"
#include "mote_nav/wheel_speed.hpp"

using mote_nav::clampToEnvelope;
using mote_nav::exceedsEnvelope;
using mote_nav::gateIncrement;
using mote_nav::GateLimits;
using mote_nav::Increment;
using mote_nav::relativeMotion;

namespace
{

// robot.yaml's measured envelope, and the tolerance the launch file applies.
constexpr double kMaxWheelSpeed = 0.218;
constexpr double kWheelSeparation = 0.22;
constexpr double kTolerance = 1.15;

GateLimits limits()
{
  return {
    kMaxWheelSpeed * kTolerance,
    mote_nav::maxYawRate(kMaxWheelSpeed, kWheelSeparation) * kTolerance,
  };
}

// One lidar interval.
constexpr double kDt = 0.1;

}  // namespace

TEST(IcpOdomGate, PassesOrdinaryMotion)
{
  // The fastest the drive actually goes, seen straight.
  const Increment icp{0.0218, 0.0, 0.0};
  const auto r = gateIncrement(icp, {0.0, 0.0, 0.0}, kDt, limits());
  EXPECT_FALSE(r.rejected);
  EXPECT_DOUBLE_EQ(r.delta.x, icp.x);
}

TEST(IcpOdomGate, PassesTheFastestLegitimateFrameMeasured)
{
  // 0.2453 m/s, the quickest interval in the bags that is not an excursion.
  const Increment icp{0.02453, 0.0, 0.0};
  EXPECT_FALSE(exceedsEnvelope(icp, kDt, limits()));
}

TEST(IcpOdomGate, RejectsTheSlowestExcursionMeasured)
{
  // 0.273 m/s, the mildest excursion in the bags.
  const Increment icp{0.0273, 0.0, 0.0};
  EXPECT_TRUE(exceedsEnvelope(icp, kDt, limits()));
}

TEST(IcpOdomGate, SubstitutesTheWheelIncrementWhenRejecting)
{
  // 1.2 m/s claimed while the wheels reported the robot stationary: the worst
  // frame in 20260706_133149.
  const Increment icp{-0.1206, -0.0019, 0.0098};
  const Increment wheel{0.0, 0.0, 0.0};
  const auto r = gateIncrement(icp, wheel, kDt, limits());
  EXPECT_TRUE(r.rejected);
  EXPECT_DOUBLE_EQ(r.delta.x, 0.0);
  EXPECT_DOUBLE_EQ(r.delta.y, 0.0);
  EXPECT_DOUBLE_EQ(r.delta.yaw, 0.0);
}

TEST(IcpOdomGate, WhatComesOutIsAlwaysInsideTheEnvelope)
{
  // The invariant, checked where it is easiest to break: a wheel increment that
  // is itself impossible must not ride through on the back of a rejection.
  const auto lim = limits();
  const Increment icp{0.12, 0.0, 0.0};
  const Increment bogus_wheel{0.09, 0.0, 0.0};
  const auto r = gateIncrement(icp, bogus_wheel, kDt, lim);
  EXPECT_TRUE(r.rejected);
  EXPECT_FALSE(exceedsEnvelope(r.delta, kDt, lim));
  EXPECT_NEAR(std::hypot(r.delta.x, r.delta.y) / kDt, lim.max_speed, 1e-12);
}

TEST(IcpOdomGate, RejectsOnYawAloneWithinTheTranslationLimit)
{
  // Standing still and claiming three times the fastest the chassis can spin.
  const Increment icp{0.0, 0.0, 3 * mote_nav::maxYawRate(kMaxWheelSpeed, kWheelSeparation) * kDt};
  EXPECT_TRUE(exceedsEnvelope(icp, kDt, limits()));
}

TEST(IcpOdomGate, PassesTheFastestYawMeasured)
{
  // 1.974 rad/s, the quickest turn in the bags -- the kinematic maximum, and
  // the gate must not clip it.
  const Increment icp{0.0, 0.0, 1.974 * kDt};
  EXPECT_FALSE(exceedsEnvelope(icp, kDt, limits()));
}

TEST(IcpOdomGate, JudgesNothingWithoutElapsedTime)
{
  const Increment icp{5.0, 0.0, 0.0};
  EXPECT_FALSE(exceedsEnvelope(icp, 0.0, limits()));
  const auto r = gateIncrement(icp, {0.0, 0.0, 0.0}, 0.0, limits());
  EXPECT_FALSE(r.rejected);
  EXPECT_DOUBLE_EQ(r.delta.x, 5.0);
}

TEST(IcpOdomGate, ClampKeepsDirectionAndMeetsTheLimit)
{
  const auto lim = limits();
  const Increment icp{0.06, 0.08, 0.0};  // 1.0 m/s along a 3-4-5 direction
  const auto out = clampToEnvelope(icp, kDt, lim);
  EXPECT_NEAR(std::hypot(out.x, out.y) / kDt, lim.max_speed, 1e-12);
  EXPECT_NEAR(std::atan2(out.y, out.x), std::atan2(icp.y, icp.x), 1e-12);
}

TEST(IcpOdomGate, ClampLeavesAnAcceptableIncrementAlone)
{
  const Increment icp{0.02, 0.0, 0.05};
  const auto out = clampToEnvelope(icp, kDt, limits());
  EXPECT_DOUBLE_EQ(out.x, icp.x);
  EXPECT_DOUBLE_EQ(out.yaw, icp.yaw);
}

TEST(IcpOdomGate, ClampBoundsYawKeepingItsSign)
{
  const auto lim = limits();
  const Increment icp{0.0, 0.0, -1.0};
  const auto out = clampToEnvelope(icp, kDt, lim);
  EXPECT_NEAR(out.yaw, -lim.max_yaw_rate * kDt, 1e-12);
}

TEST(IcpOdomGate, RelativeMotionIsExpressedInTheStartingBodyFrame)
{
  // Facing +y, then displaced along world +y: one metre straight ahead.
  const auto d = relativeMotion(0.0, 0.0, M_PI_2, 0.0, 1.0, M_PI_2);
  EXPECT_NEAR(d.x, 1.0, 1e-12);
  EXPECT_NEAR(d.y, 0.0, 1e-12);
  EXPECT_NEAR(d.yaw, 0.0, 1e-12);
}

TEST(IcpOdomGate, RelativeMotionWrapsYawTheShortWay)
{
  const auto d = relativeMotion(0.0, 0.0, 3.0, 0.0, 0.0, -3.0);
  EXPECT_NEAR(d.yaw, 2 * M_PI - 6.0, 1e-12);
}

TEST(IcpOdomGate, AbsorbedExcursionLeavesAConstantOffsetNotADrift)
{
  // Accumulate a straight run at a plausible speed, with one bogus frame in the
  // middle, and check the gated track ends where the wheels say rather than
  // carrying the excursion -- and that the frames after it are untouched.
  const auto lim = limits();
  const Increment good{0.02, 0.0, 0.0};
  const Increment bogus{0.12, 0.0, 0.0};
  double gated = 0.0;
  double raw = 0.0;
  for (int i = 0; i < 10; ++i) {
    const Increment icp = (i == 5) ? bogus : good;
    raw += icp.x;
    gated += gateIncrement(icp, good, kDt, lim).delta.x;
  }
  EXPECT_NEAR(gated, 10 * good.x, 1e-12);
  EXPECT_NEAR(raw - gated, bogus.x - good.x, 1e-12);
}
