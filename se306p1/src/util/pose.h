#include "vector2.h"

namespace se306p1 {
  class Pose {
  public:
    const Vector2 position_;
    const double theta_;

    inline Pose(Vector2 position, double theta) :
      position_(position), theta_(theta) { }

    inline Pose(Pose &from) :
      position_(from.position), theta_(from.theta) { }
  };
}
