#include "rotate_behavior.h"

#include "../supervisor.h"

#define DEFAULT_MOVE_SPEED 1

#define CIRCLE_AV 0.62
#define CIRCLE_LV 1.0

namespace se306p1 {
GotoSquareBehavior::GotoSquareBehavior(Supervisor &sup) : Behavior(sup) {
  this->behaving_ = false;
}

GotoSquareBehavior::~GotoSquareBehavior() {
}

void GotoSquareBehavior::Tick() {
  if (!behaving_) {
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
     * this block is only ever called once. It tells the robots to go to square,
     * then sets rotating to true so this block is never called again.
     */

    if (exec_done) {
      this->supervisor_.clusterHead_->pose_.theta_ = 999; //Sam Tony and Huggies are not sure what this line does.
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
      this->behaving_ = true;
    }
  }
}
}