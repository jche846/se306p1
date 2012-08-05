#pragma once

#include "supervisor.h"
#include <vector>
namespace se306p1 {
  class CircleSupervisor : public Supervisor {
  private:
    void FindRobotDests();
    std::shared_ptr<Robot> clusterHead_;
    std::vector<std::shared_ptr<Robot>> nonHeadRobots_;
    std::vector<Pose> lineLocations_;

  public:
    virtual ~CircleSupervisor();
    /**
    * Run the supervisor.
    */
    virtual void Run();

    void ElectHead();

  };
}