#pragma once

#include "ros/ros.h"

#include <cstdint>

#include "../macros.h"

#include "../util/vector2.h"
#include "../util/pose.h"

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

    Vector2 position_;
    double theta_;

    /**
     * Request the robot to go to a position via a child controller.
     */
    void Go(const Pose &pos, double lv, bool enqueue);

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
