#pragma once

#include "ros/ros.h"

#include "behavior.h"
#include "../../util/pose.h"

namespace se306p1 {
template<typename Policy>
class GotoPolicyBehavior : public Behavior {
public:
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

      std::vector<Vector2> positions = Policy::FindRobotPositions(this->supervisor_);

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
};
}
