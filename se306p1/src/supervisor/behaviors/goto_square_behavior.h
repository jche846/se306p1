#pragma once

#include "ros/ros.h"
#include <vector>
#include <string>
#include "goto_policy_behavior.h"

namespace se306p1 {
class SquarePolicy {
public:
  inline static std::string name() { return "square"; }
  inline static uint64_t id() { return 3; }
  static std::vector<Vector2> FindRobotPositions(Supervisor &sup);
};

typedef GotoPolicyBehavior<SquarePolicy> GotoSquareBehavior;
}
