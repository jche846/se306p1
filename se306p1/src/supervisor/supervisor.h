#pragma once

#include "ros/ros.h"
#include <map>
#include <memory>

#include "robot.h"

#include <se306p1/Position.h>

#define ASK_POS_TOPIC "/supervisor/ask_pos"
#define ANS_POS_TOPIC "/supervisor/ans_pos"

namespace se306p1 {
  class Supervisor {
    enum State {
      DISCOVERY,
      CONTROLLING
    };

  private:
    State _state;
    std::map<uint64_t, std::shared_ptr<Robot> > _robots;

    ros::NodeHandle _nh;
    ros::Subscriber _ansPosSubscriber;
    ros::Publisher _askPosPublisher;

  protected:
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
     * Request positions of all robots and discover _robots.
     */
    void discover(int timeout);

    /**
     * Run the supervisor.
     */
    void run();
  };
}
