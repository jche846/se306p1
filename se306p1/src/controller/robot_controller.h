#pragma once

#define __STDC_FORMAT_MACROS
#include <inttypes.h>

#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <rosgraph_msgs/Clock.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <se306p1/AskPosition.h>
#include <se306p1/Position.h>
#include <se306p1/ScanResult.h>
#include <se306p1/Scan.h>
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
  READY,
  DOING,
  GOING,
  SCANNING
};

enum class GoStep {
  AIMING,
  MOVING,
  ALIGNING
};

enum class ScanStep {
  INIT,
  SCANNING,
  FINISHED
};

class RobotController {
 private:
  // testing variables
  Vector2 prevpos_;
  ros::Time prevtime_;
  double prevtheta_;
  double prevlv_;
  double prevav_;

  // Robot identification
  uint64_t robot_id_;

  // Scan Result
  int scanResult_;

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
  GoStep goStep_;

  // Current step of Scan command
  ScanStep scanStep_;

  // Command queue
  std::deque<Command> commands_;

  // ROS Node handler for pub/subbing to topics
  ros::NodeHandle nh_;

  // Supervisor pub/subs
  ros::Subscriber askPosSubscriber_;
  ros::Subscriber scanSubscriber_;
  ros::Subscriber doSubscriber_;
  ros::Subscriber goSubscriber_;
  ros::Publisher ansPosPublisher_;
  ros::Publisher scanResultPublisher_;

  // Stage pub/subs
  ros::Subscriber clock_;
  ros::Subscriber odom_;
  ros::Subscriber baseScan_;
  ros::Publisher twist_;

  // Count down clock for Scanning
  int scanningDuration_;
  double scanningStart_;

 public:
  RobotController(ros::NodeHandle &nh, uint64_t id);
  virtual ~RobotController();
  void clock_callback(rosgraph_msgs::Clock msg);
  void odom_callback(nav_msgs::Odometry msg);
  void scan_callback(Scan msg);
  void go_callback(Go msg);
  void do_callback(Do msg);
  void askPosition_callback(AskPosition msg);
  void baseScan_callback(sensor_msgs::LaserScan msg);
  void AnswerPosition();
  void PublishVelocity();
  void WaitForScan();
  bool RotateInto(double theta);
  bool MoveTo(Vector2 point);
  void MoveTowardsGoal();
  void SetScanning(Scan msg);
  void SetGoing(Go msg);
  void SetDoing(Do msg);
  void ExecuteCommand(Command cmd);
  void ReceiveCommand(Command cmd);
  void DequeueCommand();
  void InterruptCommandQueue(Command cmd);
  void Run();
};
}
