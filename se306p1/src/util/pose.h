#pragma once
#include "vector2.h"

namespace se306p1 {
/**
 * A structure representing the combination of a position and an orientation.
 */
class Pose {
 public:
  /// Robot position.
  Vector2 position_;

  /// Robot orientation, in degrees CCW from East.
  double theta_;

  /**
   * Construct a pose from a vector and a theta value.
   */
  explicit inline Pose(Vector2 position = Vector2(0.0, 0.0), double theta = 0.0)
      : position_(position),
        theta_(theta) {
  }

  /**
   * Copy constructor.
   */
  inline Pose(const Pose &from)
      : position_(from.position_),
        theta_(from.theta_) {
  }

  /**
   * Check the equality of two poses.
   */
  inline bool operator==(const Pose &pose) const {
    return this->position_ == pose.theta_ && this->theta_ == pose.theta_;
  }

  /**
   * Check the inequality of two poses.
   */
  inline bool operator!=(const Pose &pose) const {
    return this->position_ != pose.theta_ || this->theta_ != pose.theta_;
  }
};
}
