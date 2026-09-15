#pragma once
#include <algorithm>
#include <cmath>
namespace climb_robot {
inline double cylinderSupport(double radius, double width, double axle_dot_normal) {
  const double c=std::clamp(std::abs(axle_dot_normal),0.0,1.0);
  return radius*std::sqrt(1.0-c*c)+0.5*width*c;
}
inline double magneticForce(double gap, double contact_force, double decay, double range) {
  if (gap>=range || gap < -0.015) return 0.0;
  const double g=std::max(gap,0.0);
  double taper=1.0;
  if (g>0.8*range) {
    const double t=(range-g)/(0.2*range);
    taper=t*t*(3.0-2.0*t);
  }
  return contact_force*taper/std::pow(1.0+g/decay,2);
}
}  // namespace climb_robot
