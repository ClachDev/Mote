#pragma once

#include <cmath>
#include <cstdint>

// Pure math for the STS3215 wheel encoders, kept free of hardware and ROS
// dependencies so it can be unit tested (see test/test_encoder.cpp).

namespace mote_hardware
{

constexpr int LEFT = 0;
constexpr int RIGHT = 1;

// Left wheel is mounted facing the opposite direction, so its sign is
// negated in both read (position/velocity) and write (commanded speed).
constexpr double WHEEL_SIGN[2] = {-1.0, 1.0};

constexpr double TICKS_PER_REV = 4096.0;  // 12-bit magnetic encoder
constexpr int16_t ROLLOVER_THRESHOLD = 2048;  // half of 4096
constexpr double TWO_PI = 2.0 * M_PI;

// Shortest signed tick delta between consecutive raw encoder readings
// (each in 0–4095), unwrapping the 12-bit rollover. A jump larger than
// half a revolution is assumed to be a wrap, so per-cycle motion must
// stay under half a revolution — guaranteed by the control loop rate.
constexpr int16_t encoder_delta(int16_t raw, int16_t last)
{
  int16_t delta = static_cast<int16_t>(raw - last);
  if (delta > ROLLOVER_THRESHOLD) {
    delta = static_cast<int16_t>(delta - TICKS_PER_REV);
  }
  if (delta < -ROLLOVER_THRESHOLD) {
    delta = static_cast<int16_t>(delta + TICKS_PER_REV);
  }
  return delta;
}

constexpr double ticks_to_radians(double ticks)
{
  return (ticks / TICKS_PER_REV) * TWO_PI;
}

}  // namespace mote_hardware
