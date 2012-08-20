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
GotoCircleBehavior::GotoCircleBehavior(Supervisor &sup) : Behavior(sup) {
  ROS_INFO("Initialized goto circle behavior.");
}

GotoCircleBehavior::~GotoCircleBehavior() {
}

void GotoCircleBehavior::Execute() {
  ROS_INFO("Going to a circle.");

  std::vector<Vector2> positions = FindRobotPositions(
    Vector2(DEFAULT_CIRCLE_X, DEFAULT_CIRCLE_Y),
    DEFAULT_CIRCLE_THETA,
    DEFAULT_CIRCLE_DIAMETER,
    this->supervisor_.robots_.size(),
    this->supervisor_.robots_.size()
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
