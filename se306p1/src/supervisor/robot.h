#pragma once

#include "ros/ros.h"

#include <cstdint>

#include "../macros.h"

namespace se306p1 {
  class Robot {
  private:
    ros::NodeHandle nh_;
    ros::Publisher doPublisher_;
    ros::Publisher goPublisher_;

  public:
    Robot(uint64_t n);
    virtual ~Robot();

    uint64_t id_;

    double x_; //< Last known x.
    double y_; //< Last known y.
    double theta_; //< Last known theta.

    /**
     * Request the robot to go to a position via a child controller.
     */
    void Go(double x, double y, double theta, double lv, bool enqueue);

    /**
     * Request the robot to stop immediately.
     */
    void Stop();

    /**
     * Request the robot repeatedly does something.
     */
    void Do(double lv, double av, bool enqueue);
  };
}
