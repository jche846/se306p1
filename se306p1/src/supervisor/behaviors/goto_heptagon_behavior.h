#pragma once

#include "ros/ros.h"
#include <vector>
#include <string>
#include "goto_policy_behavior.h"

namespace se306p1 {
class HeptagonPolicy {
public:
  inline static std::string name() { return "heptagon"; }
  inline static uint64_t id() { return 7; }
  static std::vector<Pose> FindRobotPoses(Supervisor &sup);
};

typedef GotoPolicyBehavior<HeptagonPolicy> GotoHeptagonBehavior;
}
