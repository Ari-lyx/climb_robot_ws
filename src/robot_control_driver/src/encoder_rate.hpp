#pragma once
#include <cmath>
#include <cstdint>

namespace climb_robot {
// Backward difference of encoder angle, not ODE's independent hinge-rate state.
class EncoderRate {
 public:
  void reset() { initialized_ = false; }
  double update(double angle, int64_t stamp) {
    if (initialized_ && stamp == stamp_) return velocity_;
    const double dt = (stamp - stamp_) * 1e-9;
    velocity_ = initialized_ && dt > 0.0 && dt <= 0.1 ?
        std::remainder(angle - angle_, 6.283185307179586) / dt : 0.0;
    angle_ = angle;
    stamp_ = stamp;
    initialized_ = true;
    return velocity_;
  }
 private:
  double angle_ = 0.0, velocity_ = 0.0;
  int64_t stamp_ = 0;
  bool initialized_ = false;
};
}
