#pragma once

#define __STDC_FORMAT_MACROS
#include <inttypes.h>

#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <se306p1/AskPosition.h>
#include <se306p1/Position.h>
#include <se306p1/Do.h>
#include <se306p1/Go.h>
#include "../util/pose.h"
#include "../util/command.h"
#include <deque>

#define ASSOCIATE_TOPIC "/supervisor/associate"
#define ASK_POS_TOPIC "/supervisor/ask_pos"
#define ANS_POS_TOPIC "/supervisor/ans_pos"

/**
 * This header file defines the state variables and methods available to control
 * a robot.
 */
namespace se306p1 {
enum class RobotState {
  FINISHED,
  IDLE,
  DOING,
  GOING
};

enum class GoStep {
  AIMING,
  MOVING,
  ALIGNING
};

class RobotController {
 private:
  // Robot identification
  uint64_t robot_id_;

  // Position
  Pose pose_;

  // Where the robot is currently trying to end up after receiving a go.
  Pose goal_;

  // Movement fields
  double lv_;  // Linear velocity
  double av_;  // Angular velocity (counter clockwise)

  // Robot state
  RobotState state_;

  // Current step of Go command
  GoStep gostep_;

  // Command queue
  std::deque<Command> commands_;

  // ROS Node handler for pub/subbing to topics
  ros::NodeHandle nh_;

  // Supervisor pub/subs
  ros::Subscriber askPosSubscriber_;
  ros::Subscriber doSubscriber_;
  ros::Subscriber goSubscriber_;
  ros::Publisher ansPosPublisher_;

  // Stage pub/subs
  ros::Subscriber odom_;
  ros::Publisher twist_;

 public:
  RobotController(ros::NodeHandle &nh, uint64_t id);
  virtual ~RobotController();
  void go_callback(Go msg);
  void do_callback(Do msg);
  void askPosition_callback(AskPosition msg);
  void odom_callback(nav_msgs::Odometry msg);
  void AnswerPosition();
  void PublishVelocity();
  void UpdateVelocity();
  void MoveTowardsGoal();
  void SetGoing(Go msg);
  void SetDoing(Do msg);
  void ExecuteCommand(Command cmd);
  void DequeueCommand();
  void InterruptCommandQueue(Command cmd);
  void Run();
};
}
