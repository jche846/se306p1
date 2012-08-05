#pragma once
#include "vector2.h"

namespace se306p1 {
  class Pose {
  public:
    Vector2 position_;
    double theta_;

    inline Pose(Vector2 position=Vector2(0.0, 0.0), double theta=0.0) :
      position_(position), theta_(theta) { }

    inline Pose(const Pose &from) :
      position_(from.position_), theta_(from.theta_) { }
  };
}
