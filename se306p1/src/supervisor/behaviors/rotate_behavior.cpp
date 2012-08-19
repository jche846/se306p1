#include "rotate_behavior.h"

#include "../supervisor.h"

#include "../../util/trig.h"

#define ROBOT_WIDTH 0.35
#define DEFAULT_MOVE_SPEED 1

#define CIRCLE_AV 0.62
#define CIRCLE_LV 1.0

namespace se306p1 {
RotateBehavior::RotateBehavior(Supervisor &sup) : Behavior(sup) {
}

RotateBehavior::~RotateBehavior() {
}

void RotateBehavior::Initialize() {
  ROS_INFO("Starting rotate behavior.");

  std::vector<Pose> dests = this->FindRobotDests();
  this->MoveNodesToDests(this->supervisor_.nonHeadRobots_, dests);

  ROS_INFO("MOVING AT lv:%f av:%f", CIRCLE_LV, CIRCLE_AV);
}

void RotateBehavior::Tick() {
  if (!this->rotating_) {
    bool exec_done = true;
    // iterate through all robots and see if they have finished executing
    for (auto &cur_robot : this->supervisor_.robots_) {
      if (cur_robot.second->executing_) {
        // if any of the robots are executing wait
        exec_done = false;
        break;
      }
    }

    /**
     * this block is only ever called once. It starts the robots moving in a circle,
     * then sets rotating to true so this block is never called again.
     */

    if (exec_done) {
      this->supervisor_.clusterHead_->pose_.theta_ = 999;
      for (auto &cur_robot : this->supervisor_.robots_) {
        if (cur_robot.second->id_ != this->supervisor_.clusterHead_->id_) {
          // tell robot to move to cluster head pos
          cur_robot.second->Go(this->supervisor_.clusterHead_->pose_, false);
        }
        // enqueue rotate for after they have reached the cluster head pos
        cur_robot.second->Do(
            CIRCLE_LV, CIRCLE_AV,
            cur_robot.second->id_ != this->supervisor_.clusterHead_->id_);
      }
      this->rotating_ = true;
    }
  }
}

std::vector<Pose> RotateBehavior::FindRobotDests() {
  std::vector<Pose> lineLocations;

  size_t numRobots = this->supervisor_.robots_.size();

  double goalTheta = AngleBetweenPoints(this->supervisor_.clusterHead_->pose_.position_,
                                        Vector2());
  ROS_INFO("Setting GoalTheta to %f", goalTheta);

  // set goal theta of cluster head
  Pose headPose = this->supervisor_.clusterHead_->pose_;
  headPose.theta_ = goalTheta;
  this->supervisor_.clusterHead_->Go(headPose, false);

  Vector2 lastLocation = this->supervisor_.clusterHead_->pose_.position_;
  Vector2 robotSep = lastLocation.Normalized() * ROBOT_WIDTH * 6;
  for (size_t i = 1; i < numRobots; i++) {
    lastLocation = lastLocation + robotSep;
    lineLocations.push_back(Pose(lastLocation, goalTheta));
  }

  return lineLocations;
}
}