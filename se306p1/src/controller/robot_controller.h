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
#include <se306p1/command.h>
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
    int64_t robot_id_;

    // Position fields
    double x_; // x coordinate
    double y_; // y coordinate
    double theta_; // direction that the robot is facing, in radians counter clockwise, measured from "east".

    // Movement fields
    double lv_; // Linear velocity
    double av_; // Angular velocity (counter clockwise)

    // Loop control variables
    bool moving_;
    bool dequeuing_;

    // Command queue
    std::queue<Command> commands_;

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
    void go_callback(Go msg);
    void do_callback(Do msg);
    void askPosition_callback(AskPosition msg);
    void AnswerPosition();
    void ResolveCollision();
  };
}

