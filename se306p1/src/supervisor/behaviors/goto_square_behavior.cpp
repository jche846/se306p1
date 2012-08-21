#include "goto_square_behavior.h"

#include "../../util/pose.h"
#include "../../util/trig.h"
#include "../supervisor.h"

#define DEFAULT_MOVE_SPEED 1

#define DEFAULT_SQUARE_X -4
#define DEFAULT_SQUARE_Y -4
#define DEFAULT_SQUARE_THETA 0
#define DEFAULT_SQUARE_DIAMETER 20
#define DEFAULT_SQUARE_SIDES 4

namespace se306p1 {
std::vector<Pose> SquarePolicy::FindRobotPoses(Supervisor &sup) {
  return se306p1::FindRobotPoses(
    Vector2(DEFAULT_SQUARE_X, DEFAULT_SQUARE_Y),
    DEFAULT_SQUARE_THETA,
    DEFAULT_SQUARE_DIAMETER,
    sup.robots_.size(),
    DEFAULT_SQUARE_SIDES
  );
}
}
