#pragma once

#include "ros/ros.h"
#include <vector>

#include "robot.h"

#include <se306p1/Position.h>

#define ASK_POS_TOPIC "/supervisor/ask_pos"
#define ANS_POS_TOPIC "/supervisor/ans_pos"

namespace se306p1 {
  class Supervisor {
  private:
    std::vector<Robot> _robots;

    ros::Subscriber _ansPosSubscriber;
    ros::Publisher _askPosPublisher;

    /**
     * The callback for the position answer, used to either:
     *
     * * Initially register the position during the population phase.
     *
     * * Accepting the position for queueing the next task.
     *
     * * Resetting in the case of a late robot ans_pos.
     */
    void ansPos_callback(Position msg);

  public:
    Supervisor();
    virtual ~Supervisor();

    /**
     * Request positions of all robots and populate _robots.
     */
    void populate();

    /**
     * Reset the state of the supervisor.
     *
     * Used in the case of a late robot ans_pos.
     */
    void reset();

    /**
     * Run the supervisor.
     */
    void run();
  };
}
