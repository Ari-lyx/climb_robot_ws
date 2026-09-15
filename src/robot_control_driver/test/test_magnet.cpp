// 磁力/几何模型的单元测试：只依赖 magnet_math.hpp，不需要启动 Gazebo。
// 运行：colcon test --packages-select robot_control_driver
#include <gtest/gtest.h>
#include "magnet_math.hpp"

// 接触与捕获边界：贴壁时为满吸力，达到/超过捕获距离后必须为零。
TEST(Magnet, ContactAndCapture) {
 EXPECT_DOUBLE_EQ(climb_robot::magneticForce(0,90,.012,.04),90);      // g=0 -> 满吸力
 EXPECT_DOUBLE_EQ(climb_robot::magneticForce(.04,90,.012,.04),0);     // g=range -> 0
 EXPECT_DOUBLE_EQ(climb_robot::magneticForce(.1,90,.012,.04),0);      // 远场 -> 0
 EXPECT_DOUBLE_EQ(climb_robot::magneticForce(-.1,90,.012,.04),0);     // 异常穿透 -> 0
 EXPECT_LE(climb_robot::magneticForce(-.001,90,.012,.04),90);         // 轻微穿透不放大吸力
}

// 单调性与截止连续性：力随间隙单调不增，且在捕获距离末端连续收敛到 0
// （若不连续，物理求解器会在边界处出现力的阶跃，导致机器人抖动/弹跳）。
TEST(Magnet, MonotonicAndContinuousCutoff) {
 double last=90;
 for(double gap=0;gap<.041;gap+=.00001) {
  const double f=climb_robot::magneticForce(gap,90,.012,.04);
  EXPECT_LE(f,last+1e-10);EXPECT_GE(f,0);last=f;
 }
 EXPECT_NEAR(climb_robot::magneticForce(.04-1e-7,90,.012,.04),0,1e-7);
}

// 圆柱轮支撑长度：轮轴平行壁面时支撑=半径，垂直时半宽，倾斜时介于两者之间。
TEST(Magnet, CylinderNormalSupport) {
 EXPECT_DOUBLE_EQ(climb_robot::cylinderSupport(.075,.045,0),.075);
 EXPECT_DOUBLE_EQ(climb_robot::cylinderSupport(.075,.045,1),.0225);
 EXPECT_DOUBLE_EQ(climb_robot::cylinderSupport(.075,.045,-1),.0225);
 EXPECT_GT(climb_robot::cylinderSupport(.075,.045,.1),.075);
}
