#pragma once
#include "vector2.h"

namespace se306p1 {
class Pose {
 public:
  Vector2 position_;
  double theta_;  // Theta is defined as CCW from East.

  inline Pose(Vector2 position = Vector2(0.0, 0.0), double theta = 0.0)
      : position_(position),
        theta_(theta) {
  }

  inline Pose(const Pose &from)
      : position_(from.position_),
        theta_(from.theta_) {
  }

  inline bool operator==(const Pose &pose) const {
    return this->position_ == pose.theta_ && this->theta_ == pose.theta_;
  }

  inline bool operator!=(const Pose &pose) const {
    return this->position_ != pose.theta_ || this->theta_ != pose.theta_;
  }
};
}
