#pragma once

#include "ros/ros.h"
#include <map>
#include <memory>

#include "robot.h"

#include <se306p1/Position.h>
#include <se306p1/ScanResult.h>
#include "../util/pose.h"

#include "behaviors/behavior.h"

#define ASSOCIATE_TOPIC "/supervisor/associate"
#define ASK_POS_TOPIC "/supervisor/ask_pos"
#define ANS_POS_TOPIC "/supervisor/ans_pos"

namespace se306p1 {
/**
 * The supervisor, which looks after all the robots.
 */
class Supervisor {
  /**
   * Describes the state of the supervisor.
   */
  enum class State {
    DISCOVERY,
    WAITING,
    SCANNING,
    LINEINGUP,
    CONTROLLING
  };

 private:
  State state_; ///< The state of the supervisor.


  /**
   * Find poses for robots when lining up.
   */
  std::vector<Pose> FindRobotDests();

  /// Iterator to keep track of the current robot that's sending messages.
  std::map<uint64_t, std::shared_ptr<Robot>>::iterator dispatchIt_;

  ros::NodeHandle nh_; ///< ROS node handle.
  ros::Subscriber ansPosSubscriber_; ///< Subscriber for Position.
  ros::Publisher askPosPublisher_; ///< Publisher for AskPosition.
  ros::Subscriber scanResultSubscriber_; ///< Subscriber for ScanResult.

  /// Registry of factories of behaviors.
  std::map<uint64_t, Behavior::BehaviorFactory *> behaviorFactories_;

  /**
   * The behavior that the supervisor is currently executing.
   */
  std::unique_ptr<Behavior> currentBehavior_;

  /**
   * The ID of the supervisor.
   */
  uint64_t sid_;

  /**
   * The lower end of the supervised range.
   */
  uint64_t rmin_;

  /**
   * The upper end of the supervised range.
   */
  uint64_t rmax_;

   /**
   * The callback for the position answer, used to either:
   *
   * * Initially register the position during the population phase.
   *
   * * Accepting the position for queuing the next task.
   */
  void ansPos_callback(Position msg);

  /**
   * The callbackfor scan results from the cluster head
   *
   */
  void scanResult_callback(ScanResult msg);

  /**
   * Atempt to send a message to a robot Controller
   *
   * This method must be called regularly to ensure that messages are sent to the Robot Controllers
   */
  void DispatchMessages();

  /**
   * Switch the behavior of the supervisor to one named by the given ID.
   */
  void SwitchBehavior(uint64_t id);

  /**
   * Regstier a behavior with the map of factories.
   */
  template<typename T>
  inline void RegisterBehavior() {
    this->behaviorFactories_[T::id()] = &Behavior::construct<T>;
  };

  /**
   * Register all behaviors with the map of factories. Implemented in
   * behaviors/all.h
   */
  void RegisterBehaviors();

 public:
  /**
   * The robots supervised in this cluster.
   */
  std::map<uint64_t, std::shared_ptr<Robot>> robots_;

  /**
   * The head of the supervised cluster.
   */
  std::shared_ptr<Robot> clusterHead_;

 /**
   * Location of cluster head in the line
   */
  Pose clusterHeadPose_;

  /**
   * The non-head robots of the cluster.
   */
  std::vector<std::shared_ptr<Robot>> nonHeadRobots_;

  /**
   * Create a supervisor attached to a given NodeHandle.
   */
  Supervisor(ros::NodeHandle &);

  /**
   * Request positions of all robots to discover them.
   */
  void Discover(int timeout);

  /**
   * Waits for robots to become ready
   *
   * This method will block until all robots in the swarm are ready,
   * It checks if there are robots not ready then calls ros::spinonce() and checks again
   */
  void WaitForReady();

  /**
   * Elect a cluster head.
   *
   * Will also create the nonHeadRobots_ lists
   */
  void ElectHead();

  /**
   * Moves Robots to most optimal location
   *
   * Moves robots to a location from the list such that all the robots reach there destination in the smallest amout of time.
   * This may mean robots move from valid positions to another position if that will make overall convergence faster.
   * The implementation is not perfect but is a best effort atempt without exploring all possible combinations
   */
  void MoveNodesToDests(const std::vector<std::shared_ptr<Robot> > &nodes,
                        const std::vector<Pose> &poses);

  /**
   * Start the supervisor.
   *
   * Will call Run() after finding all the RobotControllers and electing a head.
   */
  void Start();
};
}
