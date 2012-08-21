#include "rotate_behavior.h"

#include "../supervisor.h"

#define DEFAULT_MOVE_SPEED 1

#define CIRCLE_AV 0.62
#define CIRCLE_LV 1.0

namespace se306p1 {
RotateBehavior::RotateBehavior(Supervisor &sup) : Behavior(sup) {
  ROS_INFO("Initialized rotate behavior.");
}

RotateBehavior::~RotateBehavior() {
}

void RotateBehavior::Execute() {
  ROS_INFO("Rotating.");

  Pose clusterHeadPose = this->supervisor_.clusterHeadPose_;
  clusterHeadPose.theta_ = 999;
  for (auto &cur_robot : this->supervisor_.robots_) {
    if (cur_robot.second->id_ != this->supervisor_.clusterHead_->id_) {
      // tell robot to move to cluster head pos
      cur_robot.second->Go(clusterHeadPose, false);
    }
    // enqueue rotate for after they have reached the cluster head pos
    cur_robot.second->Do(
        CIRCLE_LV, CIRCLE_AV,
        cur_robot.second->id_ != this->supervisor_.clusterHead_->id_);
  }
}
}
