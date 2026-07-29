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
// (arm_check, arm_pose, arm_gains, arm_calibrate) still use. Both describe the
// same joints, so the values cannot drift; the conversions are duplicated
// because the realtime side must be C++ and the bench side must not need a
// running control stack.
//
// `zero_counts` is deliberately not called "home": "home" is the name of a
// taught rest pose in arm_poses.yaml, while zero is the encoder count that
// reads 0 rad — after calibration, the middle of the joint's travel. The two
// were both called "home" until the calibration work renamed them apart.

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
  // Raw encoder count that reads 0 rad.
  int zero_counts = ARM_COUNTS_PER_REV / 2;
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
    return sign * (counts - zero_counts) * RAD_PER_COUNT;
  }

  // Convert a joint angle to a raw encoder goal, saturated at the encoder
  // edge. The angle is *not* soft-clamped here — callers clamp with clamp_rad
  // first so a limit breach is a deliberate, visible decision rather than a
  // silent saturation at 0 or 4095.
  //
  // Open question inherited from the calibration work: it is not yet confirmed
  // that a servo's homing-offset register (written by `pixi run arm-calibrate`)
  // applies to commanded goals as well as to feedback. If it turns out to need
  // compensating for, it has to be compensated here *and* in
  // mote_arm/config.py's rad_to_counts — the two must not diverge, or the arm
  // lands somewhere different depending on which one commanded it.
  int rad_to_counts(double rad) const
  {
    const int counts =
      static_cast<int>(std::lround(zero_counts + sign * rad / RAD_PER_COUNT));
    return std::max(0, std::min(ARM_COUNTS_PER_REV - 1, counts));
  }
};

}  // namespace mote_hardware
