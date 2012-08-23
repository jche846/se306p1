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

namespace se306p1 {
/**
 * The robot's state.
 */
enum class RobotState {
  READY,
  DOING,
  GOING,
  SCANNING
};

/**
 * The robot's go states.
 */
enum class GoStep {
  AIMING,
  MOVING,
  ALIGNING
};

/**
 * The robot's scan states.
 */
enum class ScanStep {
  INIT,
  SCANNING,
  FINISHED
};

/**
 * The controller class for robots on Stage.
 */
class RobotController {
 private:

  /// Robot identification.
  uint64_t robot_id_;

  /// Result of the last scan.
  int scanResult_;

  /// Position and orientation.
  Pose pose_;

  /// The previous direction the robot was facing.
  Vector2 prevDirection_;

  /// Where the robot is currently trying to end up after receiving a go.
  Pose goal_;

  // The current tolerances for the action.
  double errDist_; ///< Distance tolerance.
  double errTheta_; ///< Angular tolerance.

  // Movement fields
  double lv_;  ///< Linear velocity
  double av_;  ///< Angular velocity (counter clockwise)

  /// Robot state
  RobotState state_;

  /// Current step of Go command.
  GoStep goStep_;

  /// Current step of Scan command.
  ScanStep scanStep_;

  /// Command queue.
  std::deque<Command> commands_;

  /// ROS Node handler for pub/subbing to topics.
  ros::NodeHandle nh_;

  // Supervisor pub/subs
  ros::Subscriber askPosSubscriber_; ///< Subscriber for AskPosition.
  ros::Subscriber scanSubscriber_; ///< Subscriber for Scan.
  ros::Subscriber doSubscriber_; ///< Subscriber for Do.
  ros::Subscriber goSubscriber_; ///< Subscriber for Go.
  ros::Publisher ansPosPublisher_; ///< Publisher for AnswerPosition.
  ros::Publisher scanResultPublisher_; ///< Publisher for ScanResult.

  // Stage pub/subs
  ros::Subscriber clock_; ///< Subscriber for the clock.
  ros::Subscriber odom_; ///< Subscriber for the odometry.
  ros::Subscriber baseScan_; ///< Subscriber for the laser scanner.
  ros::Publisher twist_; ///< Publisher for moving the robot.

  // Count down clock for Scanning
  int scanningDuration_; ///< How long the robot is scanning for.
  double scanningStart_; ///< When the scanning started.

 public:
  /**
   * Construct a robot controller with a given node handle and ID.
   */
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
