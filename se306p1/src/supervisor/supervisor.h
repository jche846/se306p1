#pragma once

#include "ros/ros.h"
#include <map>
#include <memory>

#include "robot.h"

#include <se306p1/Position.h>
#include "../util/pose.h"

#define ASSOCIATE_TOPIC "/supervisor/associate"
#define ASK_POS_TOPIC "/supervisor/ask_pos"
#define ANS_POS_TOPIC "/supervisor/ans_pos"

namespace se306p1 {
  class Supervisor {
    enum class State {
      DISCOVERY,
      WAITING,
      CONTROLLING,
    };

  private:
    State state_;

    std::map<uint64_t, std::shared_ptr<Robot>>::iterator dispatchIt_;

    ros::NodeHandle nh_;
    ros::Subscriber ansPosSubscriber_;
    ros::Publisher askPosPublisher_;

  protected:
    std::map<uint64_t, std::shared_ptr<Robot>> robots_;
    std::shared_ptr<Robot> clusterHead_;
    std::vector<std::shared_ptr<Robot>> nonHeadRobots_;

    /**
     * The callback for the position answer, used to either:
     *
     * * Initially register the position during the population phase.
     *
     * * Accepting the position for queuing the next task.
     */
    void ansPos_callback(Position msg);

    /**
     * Stuff for the supervisor to do. This is an abstract method.
     */
    virtual void Run() = 0;

    void DispatchMessages();

  public:
    Supervisor();
    virtual ~Supervisor();

    /**
     * Request positions of all robots to discover them.
     */
    void Discover(int timeout);
    void WaitForReady();

    /**
     * Elect a cluster head.
     */
    void ElectHead();

    /**
     * Start the supervisor.
     */
    void Start();

    /**
     *
     */
    void MoveNodesToDests(const std::vector<std::shared_ptr<Robot> > &nodes,
                          const std::vector<Pose> &poses);
  };
}
