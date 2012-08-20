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
GotoSquareBehavior::GotoSquareBehavior(Supervisor &sup) : Behavior(sup) {
  ROS_INFO("Initialized goto square behavior.");
}

GotoSquareBehavior::~GotoSquareBehavior() {
}

void GotoSquareBehavior::Execute() {
  ROS_INFO("Going to a square.");

  std::vector<Vector2> positions = FindRobotPositions(
    Vector2(DEFAULT_SQUARE_X, DEFAULT_SQUARE_Y),
    DEFAULT_SQUARE_THETA,
    DEFAULT_SQUARE_DIAMETER,
    this->supervisor_.robots_.size(),
    DEFAULT_SQUARE_SIDES
  );

  std::vector<Pose> poses;
  for (size_t i = 0; i < positions.size(); ++i) {
    poses.push_back(Pose(positions[i], 0));
  }

  std::vector<std::shared_ptr<Robot>> nodes;
  nodes.push_back(this->supervisor_.clusterHead_);

  for(auto &robot : this->supervisor_.nonHeadRobots_) {
    nodes.push_back(robot);
  }

  this->supervisor_.MoveNodesToDests(nodes, poses);
}
}
