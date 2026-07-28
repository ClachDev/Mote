#include <gtest/gtest.h>

#include <cmath>

#include "mote_hardware/arm_joint.hpp"

// These mirror mote_arm/test/test_config.py. The conversions and the clamp now
// live on both sides of the language boundary — the realtime hardware and the
// Python bench tools — so both sides are held to the same cases.

namespace mote_hardware
{

namespace
{

// elbow_flex as shipped in robot.yaml: a wide band, zero well off mid-scale.
ArmJoint elbow()
{
  return ArmJoint{"elbow_flex", 3, -3.291, 0.103, 2931, 1};
}

ArmJoint inverted()
{
  return ArmJoint{"inverted", 4, -1.0, 1.0, 2048, -1};
}

}  // namespace

TEST(ArmJointClamp, InsideBandIsUnchanged)
{
  const auto joint = elbow();
  EXPECT_DOUBLE_EQ(joint.clamp_rad(-1.0), -1.0);
  EXPECT_DOUBLE_EQ(joint.clamp_rad(0.0), 0.0);
}

TEST(ArmJointClamp, SaturatesAtBothLimits)
{
  const auto joint = elbow();
  EXPECT_DOUBLE_EQ(joint.clamp_rad(-10.0), joint.min_rad);
  EXPECT_DOUBLE_EQ(joint.clamp_rad(10.0), joint.max_rad);
  // The limits themselves are inside the band, not outside it.
  EXPECT_DOUBLE_EQ(joint.clamp_rad(joint.min_rad), joint.min_rad);
  EXPECT_DOUBLE_EQ(joint.clamp_rad(joint.max_rad), joint.max_rad);
}

TEST(ArmJointConversion, ZeroCountIsZeroRadians)
{
  const auto joint = elbow();
  EXPECT_DOUBLE_EQ(joint.counts_to_rad(joint.zero_counts), 0.0);
  EXPECT_EQ(joint.rad_to_counts(0.0), joint.zero_counts);
}

TEST(ArmJointConversion, RoundTripsThroughRadians)
{
  const auto joint = elbow();
  for (int counts : {2400, 2931, 3100, 4000}) {
    EXPECT_EQ(joint.rad_to_counts(joint.counts_to_rad(counts)), counts);
  }
}

TEST(ArmJointConversion, OneRevolutionIsFullScale)
{
  const auto joint = elbow();
  // 4096 counts = 2*pi, so a quarter turn is 1024 counts.
  EXPECT_NEAR(
    joint.counts_to_rad(joint.zero_counts + 1024), M_PI / 2.0, 1e-9);
}

TEST(ArmJointConversion, InvertFlipsDirection)
{
  const auto joint = inverted();
  EXPECT_LT(joint.counts_to_rad(joint.zero_counts + 100), 0.0);
  EXPECT_GT(joint.counts_to_rad(joint.zero_counts - 100), 0.0);
  EXPECT_LT(joint.rad_to_counts(0.5), joint.zero_counts);
}

TEST(ArmJointConversion, SaturatesAtTheEncoderEdge)
{
  // rad_to_counts deliberately does not soft-clamp — callers clamp first — but
  // it must never emit a count the servo cannot accept.
  const auto joint = elbow();
  EXPECT_EQ(joint.rad_to_counts(100.0), ARM_COUNTS_PER_REV - 1);
  EXPECT_EQ(joint.rad_to_counts(-100.0), 0);
}

TEST(ArmJointConversion, ClampedCommandStaysInsideTheEncoderRange)
{
  // The path write() actually takes: clamp, then convert.
  const auto joint = elbow();
  for (double rad : {-100.0, -3.5, -1.0, 0.0, 0.5, 100.0}) {
    const int counts = joint.rad_to_counts(joint.clamp_rad(rad));
    EXPECT_GE(counts, 0);
    EXPECT_LT(counts, ARM_COUNTS_PER_REV);
    const double back = joint.counts_to_rad(counts);
    EXPECT_GE(back, joint.min_rad - 1e-3);
    EXPECT_LE(back, joint.max_rad + 1e-3);
  }
}

}  // namespace mote_hardware
