#pragma once

#include "ros/ros.h"

#include "behavior.h"
#include "../../util/pose.h"

namespace se306p1 {
class GotoCircleBehavior : public Behavior {
private:
  bool completed_;

public:
  GotoCircleBehavior(Supervisor &sup);
  virtual ~GotoCircleBehavior();

  /**
   * Get the ID of the behavior.
   */
  inline static uint64_t id() { return 2; }

  virtual void Tick();
};
}
