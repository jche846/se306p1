#pragma once

#include "ros/ros.h"
#include <vector>
#include <string>
#include "goto_policy_behavior.h"

namespace se306p1 {
class CirclePolicy {
public:
  inline static std::string name() { return "circle"; }
  inline static uint64_t id() { return 2; }
  static std::vector<Pose> FindRobotPoses(Supervisor &sup);
};

typedef GotoPolicyBehavior<CirclePolicy> GotoCircleBehavior;
}
