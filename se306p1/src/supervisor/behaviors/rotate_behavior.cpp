#include "rotate_behavior.h"

#include "../supervisor.h"

#define DEFAULT_MOVE_SPEED 1

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

    int num_of_robots =  this->supervisor_.robots_.size();
    double radius = (num_of_robots * 6 * 0.35) / (M_PI * 2);

    double circle_lv = 4.0;
    double circle_av = circle_lv / (radius);

    cur_robot.second->Do(
        circle_lv, circle_av,
        cur_robot.second->id_ != this->supervisor_.clusterHead_->id_);
  }
}
}
