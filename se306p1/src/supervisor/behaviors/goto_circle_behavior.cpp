#include "goto_circle_behavior.h"

#include "../../util/pose.h"
#include "../../util/trig.h"
#include "../supervisor.h"

#define DEFAULT_MOVE_SPEED 1
#define DEFAULT_CIRCLE_X 4
#define DEFAULT_CIRCLE_Y 4
#define DEFAULT_CIRCLE_DIAMETER 100.0

namespace se306p1 {
GotoCircleBehavior::GotoCircleBehavior(Supervisor &sup) : Behavior(sup) {
  ROS_INFO("Initialized goto circle behavior.");
  this->completed_ = false;
}

GotoCircleBehavior::~GotoCircleBehavior() {
}

void GotoCircleBehavior::Tick() {
  if (this->completed_) return;
  ROS_INFO("Going to a circle.");

  std::vector<Vector2> positions = FindCirclePositions(
    Vector2(DEFAULT_CIRCLE_X, DEFAULT_CIRCLE_Y),
    DEFAULT_CIRCLE_DIAMETER,
    this->supervisor_.robots_.size()
  );

  for (size_t i = 0; i < this->supervisor_.robots_.size(); ++i) {
    Robot &robot = *this->supervisor_.robots_[i];
    robot.Go(Pose(positions[i], 0), false);
    ROS_INFO("Robot %" PRId64 " is being sent to (%f, %f)",
      robot.id_,
      positions[i].x_,
      positions[i].y_
    );
  }
  this->completed_ = true;
}
}
