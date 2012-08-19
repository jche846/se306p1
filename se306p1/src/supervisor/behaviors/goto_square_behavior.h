#pragma once

#include "ros/ros.h"

#include "behavior.h"
#include "../../util/pose.h"

namespace se306p1 {
class GotoSquareBehavior : public Behavior {
private:
  bool completed_; //initally false;

public:
  GotoSquareBehavior(Supervisor &sup);
  virtual ~GotoSquareBehavior();

  inline static uint64_t id() { return 3; }

  virtual void Tick();
};
}
