#include <gtest/gtest.h>
#include "magnet_math.hpp"
TEST(Magnet, ContactAndCapture) {
 EXPECT_DOUBLE_EQ(climb_robot::magneticForce(0,90,.012,.04),90);
 EXPECT_DOUBLE_EQ(climb_robot::magneticForce(.04,90,.012,.04),0);
 EXPECT_DOUBLE_EQ(climb_robot::magneticForce(.1,90,.012,.04),0);
 EXPECT_DOUBLE_EQ(climb_robot::magneticForce(-.1,90,.012,.04),0);
 EXPECT_LE(climb_robot::magneticForce(-.001,90,.012,.04),90);
}
TEST(Magnet, MonotonicAndContinuousCutoff) {
 double last=90;
 for(double gap=0;gap<.041;gap+=.00001) {
  const double f=climb_robot::magneticForce(gap,90,.012,.04);
  EXPECT_LE(f,last+1e-10);EXPECT_GE(f,0);last=f;
 }
 EXPECT_NEAR(climb_robot::magneticForce(.04-1e-7,90,.012,.04),0,1e-7);
}
TEST(Magnet, CylinderNormalSupport) {
 EXPECT_DOUBLE_EQ(climb_robot::cylinderSupport(.075,.045,0),.075);
 EXPECT_DOUBLE_EQ(climb_robot::cylinderSupport(.075,.045,1),.0225);
 EXPECT_DOUBLE_EQ(climb_robot::cylinderSupport(.075,.045,-1),.0225);
 EXPECT_GT(climb_robot::cylinderSupport(.075,.045,.1),.075);
}
