#pragma once

#include "ros/ros.h"
#include <vector>
#include <string>
#include "goto_policy_behavior.h"

namespace se306p1 {
class TrianglePolicy {
public:
  inline static std::string name() { return "triangle"; }
  inline static uint64_t id() { return 3; }
  static std::vector<Pose> FindRobotPoses(Supervisor &sup);
};

typedef GotoPolicyBehavior<TrianglePolicy> GotoTriangleBehavior;
}
