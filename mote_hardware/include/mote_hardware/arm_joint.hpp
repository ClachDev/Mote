#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>

// Pure math and configuration for one SO-101 arm servo, kept free of hardware
// and ROS dependencies so the safety-critical parts (soft-limit clamping, the
// encoder<->radian conversion) can be unit tested (see test/test_arm_joint.cpp).
//
// This mirrors mote_arm/config.py's JointSpec, which the Python bench tools
// (arm_check, arm_pose, arm_gains) still use. Both read the same `arm:` section
// of robot.yaml, so the values cannot drift; the conversions are duplicated
// because the realtime side must be C++ and the bench side must not need a
// running control stack.

namespace mote_hardware
{

// STS3215 encoders report a single 12-bit turn: 4096 counts = 2*pi.
constexpr int ARM_COUNTS_PER_REV = 4096;
constexpr double RAD_PER_COUNT = 2.0 * M_PI / ARM_COUNTS_PER_REV;

struct ArmJoint
{
  std::string name;
  int id = 0;
  double min_rad = 0.0;
  double max_rad = 0.0;
  // Raw encoder count corresponding to 0 rad (the joint's mechanical zero).
  int home_counts = ARM_COUNTS_PER_REV / 2;
  // -1 if the joint's positive direction is opposite the servo's.
  int sign = 1;

  // Clamp a commanded angle to the joint's soft limits.
  double clamp_rad(double rad) const
  {
    return std::max(min_rad, std::min(max_rad, rad));
  }

  // Convert a raw encoder reading to radians about the joint zero.
  double counts_to_rad(int counts) const
  {
    return sign * (counts - home_counts) * RAD_PER_COUNT;
  }

  // Convert a joint angle to a raw encoder goal, saturated at the encoder
  // edge. The angle is *not* soft-clamped here — callers clamp with clamp_rad
  // first so a limit breach is a deliberate, visible decision rather than a
  // silent saturation at 0 or 4095.
  int rad_to_counts(double rad) const
  {
    const int counts =
      static_cast<int>(std::lround(home_counts + sign * rad / RAD_PER_COUNT));
    return std::max(0, std::min(ARM_COUNTS_PER_REV - 1, counts));
  }
};

}  // namespace mote_hardware
