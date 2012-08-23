#pragma once

#include "ros/ros.h"
#include <vector>
#include <string>
#include "goto_policy_behavior.h"

namespace se306p1 {
class PentagonPolicy {
public:
  inline static std::string name() { return "pentagon"; }
  inline static uint64_t id() { return 5; }
  static std::vector<Pose> FindRobotPoses(Supervisor &sup);
};

typedef GotoPolicyBehavior<PentagonPolicy> GotoPentagonBehavior;
}
