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

    /**
     * Atempt to send a message to a robot Controller
     *
     * This method must be called regularly to ensure that messages are sent to the Robot Controllers  
     */
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
     * 
     * Will also create the nonHeadRobots_ lists
     */
    void ElectHead();

    /**
     * Start the supervisor.
     *
     * Will call Run() after finding all the RobotControllers and electing a head.
     */
    void Start();

    /**
     * Moves a list of robots to a list of position
     *
     * Will send robots to destinations in a manner that the minimum time is taken for all the robots to reach a destination 
     */
    void MoveNodesToDests(const std::vector<std::shared_ptr<Robot> > &nodes,
                          const std::vector<Pose> &poses);
  };
}
