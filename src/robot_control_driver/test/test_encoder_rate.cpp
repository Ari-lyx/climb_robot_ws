#include <gtest/gtest.h>
#include "encoder_rate.hpp"

TEST(EncoderRate, TracksMeasuredMotionAndStopsWithoutThresholding) {
  climb_robot::EncoderRate rate;
  EXPECT_DOUBLE_EQ(rate.update(0.0, 0), 0.0);
  EXPECT_NEAR(rate.update(0.001, 1000000), 1.0, 1e-10);
  EXPECT_NEAR(rate.update(0.00102, 2000000), 0.02, 1e-10);
  EXPECT_DOUBLE_EQ(rate.update(0.00102, 3000000), 0.0);
}
TEST(EncoderRate, UnwrapsAngleAndPreservesDuplicateSampleVelocity) {
  climb_robot::EncoderRate rate;
  rate.update(3.1415, 1000000);
  const auto velocity = rate.update(-3.1415, 2000000);
  EXPECT_NEAR(velocity, 0.185307179586, 1e-9);
  EXPECT_DOUBLE_EQ(rate.update(-3.1415, 2000000), velocity);
}
TEST(EncoderRate, DoesNotDifferentiateAcrossResetOrLargeTimeJump) {
  climb_robot::EncoderRate rate;
  rate.update(1.0, 1000000000);
  EXPECT_DOUBLE_EQ(rate.update(-1.0, 0), 0.0);
  EXPECT_NEAR(rate.update(-0.999, 1000000), 1.0, 1e-10);
  EXPECT_DOUBLE_EQ(rate.update(1.5, 2000000000), 0.0);
  rate.reset();
  EXPECT_DOUBLE_EQ(rate.update(2.0, 2001000000), 0.0);
}
