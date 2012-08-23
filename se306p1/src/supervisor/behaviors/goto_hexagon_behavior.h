#pragma once

#include "ros/ros.h"
#include <vector>
#include <string>
#include "goto_policy_behavior.h"

namespace se306p1 {
class HexagonPolicy {
public:
  inline static std::string name() { return "hexagon"; }
  inline static uint64_t id() { return 6; }
  static std::vector<Pose> FindRobotPoses(Supervisor &sup);
};

typedef GotoPolicyBehavior<HexagonPolicy> GotoHexagonBehavior;
}
