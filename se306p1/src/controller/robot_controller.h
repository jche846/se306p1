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

