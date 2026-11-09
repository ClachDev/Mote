#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>

#include "mote_nav/icp_odom_gate.hpp"
#include "mote_nav/wheel_speed.hpp"

using mote_nav::clampToEnvelope;
using mote_nav::exceedsEnvelope;
using mote_nav::GatedOdometry;
using mote_nav::gateIncrement;
using mote_nav::GateLimits;
using mote_nav::Increment;
using mote_nav::Pose2;
using mote_nav::relativeMotion;
using mote_nav::WheelHistory;

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

namespace
{

constexpr std::int64_t kMs = 1000000;
// An epoch-sized stamp, so nanosecond arithmetic is exercised at real scale.
constexpr std::int64_t kT0 = 1762000000LL * 1000000000LL;

// Wheel rate and a speed comfortably inside the envelope: 0.15 m/s straight.
constexpr std::int64_t kWheelPeriod = 20 * kMs;
constexpr double kSpeed = 0.15;

Pose2 straight(std::int64_t t)
{
  return {kSpeed * static_cast<double>(t - kT0) * 1e-9, 0.0, 0.0};
}

GatedOdometry odometry()
{
  return GatedOdometry(limits(), 0.5);
}

}  // namespace

TEST(WheelHistory, InterpolatesBetweenSamples)
{
  WheelHistory h;
  h.add(kT0, {0.0, 0.0, 0.0});
  h.add(kT0 + 20 * kMs, {0.004, 0.002, 0.02});
  Pose2 p;
  ASSERT_TRUE(h.poseAt(kT0 + 5 * kMs, p));
  EXPECT_NEAR(p.x, 0.001, 1e-12);
  EXPECT_NEAR(p.y, 0.0005, 1e-12);
  EXPECT_NEAR(p.yaw, 0.005, 1e-12);
}

TEST(WheelHistory, InterpolatesYawTheShortWayAcrossPi)
{
  WheelHistory h;
  h.add(kT0, {0.0, 0.0, M_PI - 0.01});
  h.add(kT0 + 20 * kMs, {0.0, 0.0, -M_PI + 0.01});
  Pose2 p;
  ASSERT_TRUE(h.poseAt(kT0 + 10 * kMs, p));
  EXPECT_NEAR(std::abs(p.yaw), M_PI, 1e-12);
}

TEST(WheelHistory, NeverExtrapolates)
{
  WheelHistory h;
  h.add(kT0, {0.0, 0.0, 0.0});
  h.add(kT0 + 20 * kMs, {0.004, 0.0, 0.0});
  Pose2 p;
  EXPECT_FALSE(h.poseAt(kT0 - 1, p));
  EXPECT_FALSE(h.poseAt(kT0 + 20 * kMs + 1, p));
  EXPECT_TRUE(h.poseAt(kT0 + 20 * kMs, p));
}

TEST(WheelHistory, IncrementIsInTheBodyFrameAtTheStart)
{
  WheelHistory h;
  h.add(kT0, {1.0, 1.0, M_PI_2});
  h.add(kT0 + 100 * kMs, {1.0, 1.02, M_PI_2});
  Increment d;
  ASSERT_TRUE(h.increment(kT0, kT0 + 100 * kMs, d));
  EXPECT_NEAR(d.x, 0.02, 1e-12);
  EXPECT_NEAR(d.y, 0.0, 1e-12);
}

TEST(WheelHistory, DropBeforeKeepsTheSampleThatSpansTheStamp)
{
  WheelHistory h;
  for (int i = 0; i < 10; ++i) {
    h.add(kT0 + i * kWheelPeriod, straight(kT0 + i * kWheelPeriod));
  }
  const std::int64_t t = kT0 + 4 * kWheelPeriod + 5 * kMs;
  h.dropBefore(t);
  Pose2 p;
  EXPECT_TRUE(h.poseAt(t, p));
  EXPECT_FALSE(h.poseAt(kT0 + 4 * kWheelPeriod - 1, p));
}

TEST(WheelHistory, ARepeatedStampIsIgnoredAndABackwardsClockRestarts)
{
  WheelHistory h;
  h.add(kT0, {0.0, 0.0, 0.0});
  h.add(kT0 + kWheelPeriod, {0.003, 0.0, 0.0});
  h.add(kT0 + kWheelPeriod, {9.0, 0.0, 0.0});
  Pose2 p;
  ASSERT_TRUE(h.poseAt(kT0 + kWheelPeriod, p));
  EXPECT_DOUBLE_EQ(p.x, 0.003);
  h.add(kT0 - kWheelPeriod, {0.0, 0.0, 0.0});
  EXPECT_EQ(h.size(), 1u);
}

TEST(GatedOdometry, PublishesNothingBeforeTheFirstScan)
{
  auto o = odometry();
  o.onWheel(kT0, straight(kT0));
  o.onWheel(kT0 + kWheelPeriod, straight(kT0 + kWheelPeriod));
  std::int64_t t;
  Pose2 p;
  EXPECT_FALSE(o.next(t, p));
}

TEST(GatedOdometry, CarriesTheAnchorForwardOnEachWheelSample)
{
  auto o = odometry();
  o.onWheel(kT0, straight(kT0));
  const Pose2 seed{2.0, 1.0, M_PI_2};
  EXPECT_TRUE(o.onScan(kT0, seed).first);
  std::int64_t t;
  Pose2 p;
  ASSERT_TRUE(o.next(t, p));
  EXPECT_EQ(t, kT0);

  for (int i = 1; i <= 4; ++i) {
    const std::int64_t tw = kT0 + i * kWheelPeriod;
    o.onWheel(tw, straight(tw));
    ASSERT_TRUE(o.next(t, p));
    EXPECT_EQ(t, tw);
    // Facing +y, so forward wheel motion moves the gated pose along +y.
    EXPECT_NEAR(p.x, 2.0, 1e-12);
    EXPECT_NEAR(p.y, 1.0 + kSpeed * i * 0.02, 1e-12);
    EXPECT_FALSE(o.next(t, p));
  }
}

TEST(GatedOdometry, ALateScanMovesTheAnchorButNeverPublishesBackwards)
{
  auto o = odometry();
  o.onWheel(kT0, straight(kT0));
  o.onScan(kT0, straight(kT0));
  std::int64_t t;
  Pose2 p;
  o.next(t, p);
  // Wheels run 140 ms ahead before the scan stamped at 100 ms is answered.
  for (int i = 1; i <= 7; ++i) {
    o.onWheel(kT0 + i * kWheelPeriod, straight(kT0 + i * kWheelPeriod));
    ASSERT_TRUE(o.next(t, p));
  }
  const std::int64_t published = t;

  // ICP says the robot went 5 mm less far than the wheels did.
  Pose2 icp = straight(kT0 + 100 * kMs);
  icp.x -= 0.005;
  const auto u = o.onScan(kT0 + 100 * kMs, icp);
  EXPECT_FALSE(u.rejected);
  ASSERT_TRUE(u.have_step);
  EXPECT_NEAR(u.step.x, -0.005, 1e-9);
  EXPECT_FALSE(o.next(t, p));

  const std::int64_t tw = kT0 + 8 * kWheelPeriod;
  o.onWheel(tw, straight(tw));
  ASSERT_TRUE(o.next(t, p));
  EXPECT_GT(t, published);
  EXPECT_EQ(t, tw);
  // The anchor at 100 ms, then the wheels' 60 ms since.
  EXPECT_NEAR(p.x, straight(tw).x - 0.005, 1e-9);
}

TEST(GatedOdometry, ARejectedIncrementLeavesThePoseOnTheWheels)
{
  auto o = odometry();
  for (int i = 0; i <= 5; ++i) {
    o.onWheel(kT0 + i * kWheelPeriod, straight(kT0 + i * kWheelPeriod));
  }
  o.onScan(kT0, straight(kT0));
  // 0.12 m in one scan: the worst excursion measured.
  Pose2 icp = straight(kT0 + 100 * kMs);
  icp.x += 0.12;
  const auto u = o.onScan(kT0 + 100 * kMs, icp);
  EXPECT_TRUE(u.rejected);
  EXPECT_TRUE(u.wheel_prior);
  EXPECT_NEAR(o.anchor().x, straight(kT0 + 100 * kMs).x, 1e-12);
  ASSERT_TRUE(u.have_step);
  EXPECT_NEAR(std::hypot(u.step.x, u.step.y), 0.0, 1e-12);
}

TEST(GatedOdometry, WithoutAWheelPriorARejectionIsClamped)
{
  auto o = odometry();
  o.onScan(kT0, {0.0, 0.0, 0.0});
  const auto u = o.onScan(kT0 + 100 * kMs, {0.12, 0.0, 0.0});
  EXPECT_TRUE(u.rejected);
  EXPECT_FALSE(u.wheel_prior);
  EXPECT_FALSE(u.have_step);
  EXPECT_NEAR(o.anchor().x, limits().max_speed * 0.1, 1e-12);
}

TEST(GatedOdometry, WheelPropagationIsHeldToTheEnvelope)
{
  auto o = odometry();
  o.onWheel(kT0, {0.0, 0.0, 0.0});
  o.onScan(kT0, {0.0, 0.0, 0.0});
  std::int64_t t;
  Pose2 p;
  o.next(t, p);
  // A wheel claiming 1 m/s and a spin far past the chassis's limit.
  o.onWheel(kT0 + 50 * kMs, {0.05, 0.0, 1.0});
  ASSERT_TRUE(o.next(t, p));
  EXPECT_NEAR(p.x, limits().max_speed * 0.05, 1e-12);
  EXPECT_NEAR(p.yaw, limits().max_yaw_rate * 0.05, 1e-12);
}

TEST(GatedOdometry, StopsPropagatingWhenTheLidarStopsAnswering)
{
  auto o = odometry();
  o.onWheel(kT0, straight(kT0));
  o.onScan(kT0, straight(kT0));
  std::int64_t t;
  Pose2 p;
  o.next(t, p);
  int published = 0;
  for (int i = 1; i <= 50; ++i) {
    o.onWheel(kT0 + i * kWheelPeriod, straight(kT0 + i * kWheelPeriod));
    published += o.next(t, p) ? 1 : 0;
  }
  // 0.5 s at 20 ms: 25 samples, and then odom->base goes stale.
  EXPECT_EQ(published, 25);
  EXPECT_EQ(t, kT0 + 500 * kMs);
}

TEST(GatedOdometry, WithNoWheelsItPublishesAtEachScanAsBefore)
{
  auto o = odometry();
  std::int64_t t;
  Pose2 p;
  for (int i = 0; i < 3; ++i) {
    o.onScan(kT0 + i * 100 * kMs, {0.02 * i, 0.0, 0.0});
    ASSERT_TRUE(o.next(t, p));
    EXPECT_EQ(t, kT0 + i * 100 * kMs);
    EXPECT_NEAR(p.x, 0.02 * i, 1e-12);
  }
}

TEST(GatedOdometry, WheelsBehindTheScanPublishTheAnchorThenWait)
{
  auto o = odometry();
  o.onWheel(kT0, straight(kT0));
  o.onScan(kT0, straight(kT0));
  std::int64_t t;
  Pose2 p;
  o.next(t, p);
  o.onWheel(kT0 + 40 * kMs, straight(kT0 + 40 * kMs));
  o.next(t, p);
  o.onScan(kT0 + 100 * kMs, straight(kT0 + 100 * kMs));
  ASSERT_TRUE(o.next(t, p));
  EXPECT_EQ(t, kT0 + 100 * kMs);
  o.onWheel(kT0 + 80 * kMs, straight(kT0 + 80 * kMs));
  EXPECT_FALSE(o.next(t, p));
  o.onWheel(kT0 + 120 * kMs, straight(kT0 + 120 * kMs));
  ASSERT_TRUE(o.next(t, p));
  EXPECT_EQ(t, kT0 + 120 * kMs);
  EXPECT_NEAR(p.x, straight(kT0 + 120 * kMs).x, 1e-12);
}

TEST(GatedOdometry, PublicationStampsOnlyEverIncrease)
{
  // Scans arriving 30-90 ms after their stamp, interleaved with wheels at
  // 50 Hz, the way the executor sees them.
  auto o = odometry();
  std::int64_t last = 0;
  std::int64_t t;
  Pose2 p;
  int scan = 0;
  for (int i = 0; i <= 200; ++i) {
    const std::int64_t tw = kT0 + i * kWheelPeriod;
    o.onWheel(tw, straight(tw));
    if (o.next(t, p)) {
      EXPECT_GT(t, last);
      last = t;
    }
    const std::int64_t ts = kT0 + scan * 100 * kMs;
    if (tw >= ts + (30 + 15 * (scan % 5)) * kMs) {
      o.onScan(ts, straight(ts));
      ++scan;
      if (o.next(t, p)) {
        EXPECT_GT(t, last);
        last = t;
      }
    }
  }
  EXPECT_EQ(last, kT0 + 200 * kWheelPeriod);
}
