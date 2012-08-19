#include "rotate_behavior.h"

#include "../supervisor.h"

#define DEFAULT_MOVE_SPEED 1

#define CIRCLE_AV 0.62
#define CIRCLE_LV 1.0

namespace se306p1 {
RotateBehavior::RotateBehavior(Supervisor &sup) : Behavior(sup) {
}

RotateBehavior::~RotateBehavior() {
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
}