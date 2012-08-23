#include "goto_octagon_behavior.h"

#include "../../util/pose.h"
#include "../../util/trig.h"
#include "../supervisor.h"

#define DEFAULT_MOVE_SPEED 1

#define DEFAULT_X -10
#define DEFAULT_Y -10
#define DEFAULT_THETA 0
#define DEFAULT_DIAMETER 20

namespace se306p1 {
std::vector<Pose> OctagonPolicy::FindRobotPoses(Supervisor &sup) {
  return se306p1::FindRobotPoses(
    Vector2(DEFAULT_X, DEFAULT_Y),
    DEFAULT_THETA,
    DEFAULT_DIAMETER,
    sup.robots_.size(),
    8
  );
}
}
