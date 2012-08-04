#pragma once

#include "ros/ros.h"

#include <cstdint>

#include "../macros.h"

namespace se306p1 {
  class Robot {
  private:
    ros::NodeHandle _nh;
    ros::Publisher _doPublisher;
    ros::Publisher _goPublisher;

  public:
    Robot(uint64_t n);
    virtual ~Robot();

    uint64_t id;

    double x; //< Last known x.
    double y; //< Last known y.
    double theta; //< Last known theta.

    /**
     * Request the robot to go to a position via a child controller.
     */
    void go(double x, double y, double theta, double lv, bool enqueue);

    /**
     * Request the robot to stop immediately.
     */
    void stop();

    /**
     * Request the robot repeatedly does something.
     */
    void do_(double lv, double av, bool enqueue);
  };
}
