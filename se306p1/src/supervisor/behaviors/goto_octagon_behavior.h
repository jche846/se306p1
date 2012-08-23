#pragma once

#include "ros/ros.h"
#include <vector>
#include <string>
#include "goto_policy_behavior.h"

namespace se306p1 {
class OctagonPolicy {
public:
  inline static std::string name() { return "octagon"; }
  inline static uint64_t id() { return 8; }
  static std::vector<Pose> FindRobotPoses(Supervisor &sup);
};

typedef GotoPolicyBehavior<OctagonPolicy> GotoOctagonBehavior;
}
