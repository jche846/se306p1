#include "goto_circle_behavior.h"

#include "../../util/pose.h"
#include "../../util/trig.h"
#include "../supervisor.h"

#define DEFAULT_MOVE_SPEED 1
#define DEFAULT_CIRCLE_X 4
#define DEFAULT_CIRCLE_Y 4
#define DEFAULT_CIRCLE_THETA 0
#define DEFAULT_CIRCLE_DIAMETER 100.0

namespace se306p1 {
std::vector<Vector2> CirclePolicy::FindRobotPositions(Supervisor &sup) {
  return se306p1::FindRobotPositions(
    Vector2(DEFAULT_CIRCLE_X, DEFAULT_CIRCLE_Y),
    DEFAULT_CIRCLE_THETA,
    DEFAULT_CIRCLE_DIAMETER,
    sup.robots_.size(),
    sup.robots_.size()
  );
}
}
