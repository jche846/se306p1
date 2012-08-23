#pragma once

#include "ros/ros.h"
#include <vector>
#include <string>
#include "goto_policy_behavior.h"

namespace se306p1 {
class HexagonPolicy : public GotoPolicy {
public:
  /**
   * Behavior name.
   */
  inline static std::string name() { return "hexagon"; }

  /**
   * Behavior ID.
   */
  inline static uint64_t id() { return 6; }

  /**
   * Hook to find robot poses.
   */
  static std::vector<Pose> FindRobotPoses(Supervisor &sup);
};

/**
 * Typedef to the GotoPolicyBehavior using the HexagonPolicy.
 */
typedef GotoPolicyBehavior<HexagonPolicy> GotoHexagonBehavior;
}
