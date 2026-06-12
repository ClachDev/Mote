#include <gtest/gtest.h>

#include "mote_hardware/encoder.hpp"

namespace mote_hardware
{

TEST(EncoderDelta, NoRollover)
{
  EXPECT_EQ(encoder_delta(100, 90), 10);
  EXPECT_EQ(encoder_delta(90, 100), -10);
  EXPECT_EQ(encoder_delta(2000, 2000), 0);
}

TEST(EncoderDelta, RolloverForward)
{
  // 4090 -> 5 is 11 ticks forward through the wrap, not 4085 backward
  EXPECT_EQ(encoder_delta(5, 4090), 11);
  EXPECT_EQ(encoder_delta(0, 4095), 1);
}

TEST(EncoderDelta, RolloverBackward)
{
  // 5 -> 4090 is 11 ticks backward through the wrap
  EXPECT_EQ(encoder_delta(4090, 5), -11);
  EXPECT_EQ(encoder_delta(4095, 0), -1);
}

TEST(EncoderDelta, HalfRevolutionBoundary)
{
  // Exactly half a revolution is ambiguous; the convention is to keep it
  // as forward motion (only deltas strictly greater than the threshold wrap).
  EXPECT_EQ(encoder_delta(2048, 0), 2048);
  // One tick past half a revolution forward reads as backward
  EXPECT_EQ(encoder_delta(2049, 0), -2047);
}

TEST(EncoderDelta, AccumulatesFullRevolution)
{
  // Sweep a full revolution in steps; accumulated deltas must total one rev
  const int16_t step = 64;
  int32_t total = 0;
  int16_t last = 4000;  // start near the wrap to cross it mid-sweep
  for (int i = 1; i <= 64; ++i) {
    int16_t raw = static_cast<int16_t>((4000 + i * step) % 4096);
    total += encoder_delta(raw, last);
    last = raw;
  }
  EXPECT_EQ(total, 4096);
  EXPECT_DOUBLE_EQ(ticks_to_radians(total), TWO_PI);
}

TEST(TicksToRadians, Conversion)
{
  EXPECT_DOUBLE_EQ(ticks_to_radians(0.0), 0.0);
  EXPECT_DOUBLE_EQ(ticks_to_radians(4096.0), TWO_PI);
  EXPECT_DOUBLE_EQ(ticks_to_radians(1024.0), TWO_PI / 4.0);
  EXPECT_DOUBLE_EQ(ticks_to_radians(-2048.0), -M_PI);
}

TEST(WheelSign, LeftInvertedRightNot)
{
  // The left wheel is mounted facing the opposite direction. The same sign
  // is applied in read() and write(), so a positive velocity command must
  // come back as a positive measured velocity on both wheels.
  EXPECT_DOUBLE_EQ(WHEEL_SIGN[LEFT], -1.0);
  EXPECT_DOUBLE_EQ(WHEEL_SIGN[RIGHT], 1.0);
  for (int wheel : {LEFT, RIGHT}) {
    const double commanded = 2.5;  // rad/s
    const double servo_speed = WHEEL_SIGN[wheel] * commanded;   // write()
    const double measured = WHEEL_SIGN[wheel] * servo_speed;    // read()
    EXPECT_DOUBLE_EQ(measured, commanded);
  }
}

}  // namespace mote_hardware
