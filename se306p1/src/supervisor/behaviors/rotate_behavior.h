#pragma once

#include "ros/ros.h"

#include "behavior.h"
#include "../../util/pose.h"

namespace se306p1 {
class RotateBehavior : public Behavior {
private:
  bool rotating_;  //initially false. Set explicitly in constructor.

public:
  RotateBehavior(Supervisor &sup);
  virtual ~RotateBehavior();

  inline static uint64_t id() { return 1; }

  virtual void Tick();
};
}
