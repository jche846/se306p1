#pragma once

#include "ros/ros.h"
#include <vector>
#include <string>
#include "goto_policy_behavior.h"

namespace se306p1 {
class OctagonPolicy : public GotoPolicy {
public:
  /**
   * Behavior name.
   */
  inline static std::string name() { return "octagon"; }

  /**
   * Behavior ID.
   */
  inline static uint64_t id() { return 8; }

  /**
   * Hook to find robot poses.
   */
  static std::vector<Pose> FindRobotPoses(Supervisor &sup);
};

/**
 * Typedef to the GotoPolicyBehavior using the OctagonPolicy.
 */
typedef GotoPolicyBehavior<OctagonPolicy> GotoOctagonBehavior;
}
