#pragma once

#include "ros/ros.h"
#include <vector>
#include <string>
#include "goto_policy_behavior.h"

namespace se306p1 {
class SquarePolicy : public GotoPolicy {
public:
  /**
   * Behavior name.
   */
  inline static std::string name() { return "square"; }

  /**
   * Behavior ID.
   */
  inline static uint64_t id() { return 4; }

  /**
   * Hook to find robot poses.
   */
  static std::vector<Pose> FindRobotPoses(Supervisor &sup);
};

/**
 * Typedef to the GotoPolicyBehavior using the SquarePolicy.
 */
typedef GotoPolicyBehavior<SquarePolicy> GotoSquareBehavior;
}
