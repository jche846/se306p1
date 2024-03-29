#include "rotate_behavior.h"

#include <limits>

#include "../supervisor.h"

#define DEFAULT_LV 4.0

namespace se306p1 {
RotateBehavior::RotateBehavior(Supervisor &sup) : Behavior(sup) {
  ROS_INFO("Initialized rotate behavior.");
}

RotateBehavior::~RotateBehavior() {
}

void RotateBehavior::Execute() {
  ROS_INFO("Rotating.");

  double goTick = ros::Time::now().toSec();
  goTick += 10.0;
  ROS_INFO("RotateBehavior goTick: %f", goTick);

  Pose clusterHeadPose = this->supervisor_.clusterHeadPose_;

  for (auto &cur_robot : this->supervisor_.robots_) {
    cur_robot.second->Go(clusterHeadPose, false, 0.00001, 0.00001, goTick);

    int num_of_robots =  this->supervisor_.robots_.size();
    double radius = (num_of_robots * 6.0 * 0.35) / (M_PI * 2.0);

    cur_robot.second->Do(
    		DEFAULT_LV,
    		DEFAULT_LV / radius,
    		true);
  }

}
}
