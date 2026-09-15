#pragma once
// 磁吸与几何支撑的纯函数实现，单独放在头文件里，便于 gtest 直接包含、不依赖 Gazebo。
#include <algorithm>
#include <cmath>
namespace climb_robot {

// 圆柱轮沿“壁面法线”方向的支撑长度 h。
//
// 为什么需要它？轮子不是质点，而是一个半径 radius、宽 width 的圆柱。
// 球壳是曲面，轮胎与壁面的最短距离不只看轮心到壁面的距离：
//   - 轮轴平行于壁面 (axle_dot_normal = 0) 时，轮胎最外缘比轮心近 radius；
//   - 轮轴垂直于壁面 (|axle_dot_normal| = 1) 时，是轮宽端面先接触，只有 width/2。
// 一般姿态下这两者叠加，得到解析式：
//     h = R·sqrt(1-c²) + (W/2)·c,  c = |轮轴单位向量 · 壁面法线|
// 轮面间隙 g = R_tank - |p_wheel - center| - h，见 magnetic_drive.cpp。
inline double cylinderSupport(double radius, double width, double axle_dot_normal) {
  const double c=std::clamp(std::abs(axle_dot_normal),0.0,1.0);
  return radius*std::sqrt(1.0-c*c)+0.5*width*c;
}

// 近场磁力模型：F(g) = F_contact · taper(g) / (1 + g/decay)²
//
// 设计取舍（不是磁场有限元，见 docs/requirements.md）：
//   * g >= range 直接为 0 —— 磁吸是“近场”，没有远距离吸引；
//   * g < -0.015 视为异常穿透，返回 0，避免求解器抖动时产生巨大冲击；
//   * 分母 (1+g/decay)² 让吸力随间隙快速但连续地衰减；
//   * 在最后 20% 捕获距离内叠加 smoothstep 锥度 taper，使 g=range 处
//     力平滑收敛到 0（否则会在边界处力突变，引起抖动）。
inline double magneticForce(double gap, double contact_force, double decay, double range) {
  if (gap>=range || gap < -0.015) return 0.0;
  const double g=std::max(gap,0.0);
  double taper=1.0;
  if (g>0.8*range) {
    const double t=(range-g)/(0.2*range);   // t 从 1 线性降到 0
    taper=t*t*(3.0-2.0*t);                  // smoothstep：一阶导在两端为 0
  }
  return contact_force*taper/std::pow(1.0+g/decay,2);
}
}  // namespace climb_robot
