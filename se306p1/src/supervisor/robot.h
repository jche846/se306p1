#pragma once

#include "ros/ros.h"

#include <cstdint>

#define ROBOT_DO_TOPIC(ROBOT) ROBOT "/do"
#define ROBOT_GO_TOPIC(ROBOT) ROBOT "/go"

namespace se306p1 {
  class Robot {
  private:
    uint64_t _id;

    ros::Publisher _doPublisher;
    ros::Publisher _goPublisher;

    double _x; //< Last known x.
    double _y; //< Last known y.
    double _theta; //< Last known theta.

  public:
    Robot();
    virtual ~Robot();

    /**
     * Request the robot to go to a position via a child controller.
     */
    void go(double x, double y, double theta, double lv, bool enqueue);

    /**
     * Request the robot repeatedly does something.
     */
    void do_(double lv, double ev, bool enqueue);
  };
}
