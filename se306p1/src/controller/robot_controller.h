/*
 * RobotController.h
 *
 *  Created on: Aug 4, 2012
 *      Author: ahug048
 */

#include <cstdint>
#include <queue>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#pragma once

/**
 * This header file defines the state variables and methods available to control a robot
 */
namespace se306p1 {
  class RobotController {

  private:

    // Robot identification
    std::uint64_t robot_id;

    // Position fields
    double x; // x coordinate
    double y; // y coordinate
    double theta; // direction that the robot is facing, in radians counter clockwise, measured from "east".

    // Movement fields
    double linear; // Linear velocity
    double angular; // Angular velocity (counter clockwise)

    // Command queue
    std::queue commands;

  public:
    RobotController();
    virtual ~RobotController();
    void MoveTo(double x, double y, double theta);
    void ContinuousMove(double lv, double av);
    void GoCallback(message);
    void DoCallback(message);
    void AskPositionCallback(message);
    void AnswerPosition();
    void ResolveCollision();
  };
}

