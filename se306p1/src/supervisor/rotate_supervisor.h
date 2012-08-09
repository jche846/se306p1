#pragma once

#include "supervisor.h"
#include <vector>

namespace se306p1 {
  class RotateSupervisor : public Supervisor {
  private:
    void FindRobotDests();
    std::vector<Pose> lineLocations_;

  public:
    virtual ~RotateSupervisor();
    /**
    * Run the supervisor.
    */
    virtual void Run();
  };
}