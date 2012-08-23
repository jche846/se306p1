#pragma once

#include "ros/ros.h"

#include "behavior.h"
#include "../../util/pose.h"

namespace se306p1 {
/**
 * A policy-based behavior.
 */
class GotoPolicy { };

/**
 * A policy-based template class for implementing GotoPolicy-based behaviors.
 */
template<typename Policy>
class GotoPolicyBehavior : public Behavior {
public:
  /**
   * Create a policy-based goto behavior.
   */
  GotoPolicyBehavior(Supervisor &sup) : Behavior(sup) {
    ROS_INFO("Initialized goto %s behavior.", Policy::name().c_str());
  }

  virtual ~GotoPolicyBehavior() { }

  /**
   * Get the ID of the behavior.
   */
  inline static uint64_t id() { return Policy::id(); }

  virtual void Execute() {
      ROS_INFO("Going to a %s.", Policy::name().c_str());

      std::vector<Pose> poses = Policy::FindRobotPoses(this->supervisor_);

      std::vector<std::shared_ptr<Robot>> nodes;
      nodes.push_back(this->supervisor_.clusterHead_);

      for(auto &robot : this->supervisor_.nonHeadRobots_) {
        nodes.push_back(robot);
      }

      this->supervisor_.MoveNodesToDests(nodes, poses);
  }
};
}
