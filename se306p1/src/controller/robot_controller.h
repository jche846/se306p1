/*
 * RobotController.h
 *
 *  Created on: Aug 4, 2012
 *      Author: ahug048
 */

#pragma once

#include "ros/ros.h"
#include <se306p1/AskPosition.h>
#include <se306p1/Position.h>
#include <se306p1/Do.h>
#include <se306p1/Go.h>
#include <cstdint>
#include <queue>

#define ASK_POS_TOPIC "/supervisor/ask_pos"
#define ANS_POS_TOPIC "/supervisor/ans_pos"
#define DO_TOPIC "/se306p1/Do"
#define GO_TOPIC "/se306p1/Go"

/**
 * This header file defines the state variables and methods available to control a robot
 */
namespace se306p1 {

class RobotController {

private:
  // Robot identification
  int64_t robot_id;

  // Position fields
  double x; // x coordinate
  double y; // y coordinate
  double theta; // direction that the robot is facing, in radians counter clockwise, measured from "east".

  // Movement fields
  double linear; // Linear velocity
  double angular; // Angular velocity (counter clockwise)

  // Command queue
  std::queue<int> commands;

  // ROS Node handler for pub/subbing to topics
  ros::NodeHandle nh_;

  //Topic pub/subs
  ros::Subscriber askPosSubscriber_;
  ros::Subscriber doSubscriber_;
  ros::Subscriber goSubscriber_;
  ros::Publisher ansPosPublisher_;

public:
  RobotController();
  virtual ~RobotController();
  void MoveTo(double x, double y, double theta);
  void ContinuousMove(double lv, double av);
  void go_callback(Go message);
  void do_callback(Do message);
  void askPosition_callback(AskPosition message);
  void AnswerPosition();
  void ResolveCollision();
};

}

