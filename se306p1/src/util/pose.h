#pragma once
#include "vector2.h"

namespace se306p1 {
  class Pose {
  public:
    Vector2 position_;
    double theta_;

    inline Pose(Vector2 position, double theta) :
      position_(position), theta_(theta) { }

    inline Pose(const Pose &from) :
      position_(from.position_), theta_(from.theta_) { }
  };
}
