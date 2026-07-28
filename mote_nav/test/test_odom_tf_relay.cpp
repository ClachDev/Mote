#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>

#include "mote_nav/odom_tf_relay.hpp"

namespace
{

// EXPECT_DOUBLE_EQ passes anything within 4 ulps, which is exactly the size of
// the difference these tests exist to catch. Comparing the bit patterns is the
// only way to assert what the file claims.
std::uint64_t bits(double v)
{
  std::uint64_t u;
  std::memcpy(&u, &v, sizeof u);
  return u;
}

#define EXPECT_BITS_EQ(a, b) EXPECT_EQ(bits(a), bits(b))

nav_msgs::msg::Odometry makeOdom(double x, double y, double yaw)
{
  nav_msgs::msg::Odometry msg;
  msg.header.frame_id = "odom";
  msg.header.stamp.sec = 12;
  msg.header.stamp.nanosec = 345000000u;
  msg.child_frame_id = "base_footprint";
  msg.pose.pose.position.x = x;
  msg.pose.pose.position.y = y;
  msg.pose.pose.orientation.z = std::sin(0.5 * yaw);
  msg.pose.pose.orientation.w = std::cos(0.5 * yaw);
  return msg;
}

}  // namespace

TEST(OdomTfRelay, IdentityPoseInvertsToIdentity)
{
  const auto t = mote_nav::invertOdometry(makeOdom(0.0, 0.0, 0.0), "odom_wheel");
  EXPECT_DOUBLE_EQ(t.transform.translation.x, 0.0);
  EXPECT_DOUBLE_EQ(t.transform.translation.y, 0.0);
  EXPECT_DOUBLE_EQ(t.transform.translation.z, 0.0);
  EXPECT_DOUBLE_EQ(t.transform.rotation.w, 1.0);
}

TEST(OdomTfRelay, PureTranslationNegates)
{
  const auto t = mote_nav::invertOdometry(makeOdom(2.0, -3.0, 0.0), "odom_wheel");
  EXPECT_DOUBLE_EQ(t.transform.translation.x, -2.0);
  EXPECT_DOUBLE_EQ(t.transform.translation.y, 3.0);
}

// A quarter turn puts the odom origin on the inverted leaf's -y axis: the base
// looks along +y in odom, so odom lies behind and to its right.
TEST(OdomTfRelay, RotationRotatesTheTranslationIntoTheBaseFrame)
{
  const auto t = mote_nav::invertOdometry(makeOdom(1.0, 0.0, M_PI_2), "odom_wheel");
  EXPECT_NEAR(t.transform.translation.x, 0.0, 1e-12);
  EXPECT_NEAR(t.transform.translation.y, 1.0, 1e-12);
  EXPECT_NEAR(t.transform.rotation.z, -std::sin(M_PI_4), 1e-12);
  EXPECT_NEAR(t.transform.rotation.w, std::cos(M_PI_4), 1e-12);
}

// Composing the pose with the emitted transform must return the origin,
// whatever the pose — the property that makes the leaf a true inverse.
TEST(OdomTfRelay, IsTheInverseOfThePose)
{
  for (const double yaw : {0.3, -1.9, 2.7}) {
    const auto odom = makeOdom(1.5, -0.75, yaw);
    const auto t = mote_nav::invertOdometry(odom, "odom_wheel");

    const auto & q = odom.pose.pose.orientation;
    const std::array<double, 4> q_fwd{q.x, q.y, q.z, q.w};
    const auto back = mote_nav::rotateVector(
      q_fwd,
      {t.transform.translation.x, t.transform.translation.y, t.transform.translation.z});

    EXPECT_NEAR(odom.pose.pose.position.x + back[0], 0.0, 1e-12);
    EXPECT_NEAR(odom.pose.pose.position.y + back[1], 0.0, 1e-12);
    EXPECT_NEAR(odom.pose.pose.position.z + back[2], 0.0, 1e-12);
  }
}

// The leaf hangs off the message's own child frame, not off a configured
// parent: kinematic_icp owns odom->base, so anchoring to anything else would
// make the relay a second writer of that edge.
TEST(OdomTfRelay, FramesComeFromTheMessageAndTheParameter)
{
  const auto t = mote_nav::invertOdometry(makeOdom(0.1, 0.2, 0.3), "wheel_prior");
  EXPECT_EQ(t.header.frame_id, "base_footprint");
  EXPECT_EQ(t.child_frame_id, "wheel_prior");
}

TEST(OdomTfRelay, CarriesTheMessageStampNotTheWallClock)
{
  const auto odom = makeOdom(0.1, 0.2, 0.3);
  const auto t = mote_nav::invertOdometry(odom, "odom_wheel");
  EXPECT_EQ(t.header.stamp.sec, odom.header.stamp.sec);
  EXPECT_EQ(t.header.stamp.nanosec, odom.header.stamp.nanosec);
}

// The exact values the Python implementation produced for this input, printed
// from it before it was deleted. Bit-identical output is what lets a bag
// recorded under either implementation be compared with an exact equality.
TEST(OdomTfRelay, MatchesThePythonImplementationBitForBit)
{
  nav_msgs::msg::Odometry odom;
  odom.child_frame_id = "base_footprint";
  odom.pose.pose.position.x = 1.234567890123;
  odom.pose.pose.position.y = -0.987654321098;
  odom.pose.pose.position.z = 0.042;
  odom.pose.pose.orientation.x = 0.1;
  odom.pose.pose.orientation.y = 0.2;
  odom.pose.pose.orientation.z = 0.3;
  odom.pose.pose.orientation.w = 0.927361849549570;

  const auto t = mote_nav::invertOdometry(odom, "odom_wheel");
  EXPECT_BITS_EQ(t.transform.translation.x, -0.31146662401722747);
  EXPECT_BITS_EQ(t.transform.translation.y, 1.414845598924649);
  EXPECT_BITS_EQ(t.transform.translation.z, -0.6344946072530234);
}

// The case above does not discriminate: on aarch64 it gives the same answer
// whether or not the compiler contracts the multiply-adds into FMA, so it
// cannot tell whether -ffp-contract=off survived. This one can. Measured on the
// Pi 5, a yaw-only pose at (1.5, 2.5) — the shape of every pose this node will
// ever see — comes out one ulp apart in x between the two builds, and the value
// below is the one Python produces. Across 200k random planar poses the two
// builds disagree on 43% of components, so dropping the flag is not a
// theoretical concern; it is only invisible if the test vector is unlucky.
TEST(OdomTfRelay, IsNotReRoundedByFusedMultiplyAdd)
{
  const double yaw = 0.5;
  nav_msgs::msg::Odometry odom;
  odom.child_frame_id = "base_footprint";
  odom.pose.pose.position.x = 1.5;
  odom.pose.pose.position.y = 2.5;
  odom.pose.pose.orientation.z = std::sin(0.5 * yaw);
  odom.pose.pose.orientation.w = std::cos(0.5 * yaw);

  const auto t = mote_nav::invertOdometry(odom, "odom_wheel");
  EXPECT_BITS_EQ(t.transform.translation.x, -2.514937689346066);
  EXPECT_BITS_EQ(t.transform.translation.y, -1.4748180968196274);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
