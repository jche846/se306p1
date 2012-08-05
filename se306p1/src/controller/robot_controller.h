/*
 * RobotController.h
 *
 *  Created on: Aug 4, 2012
 *      Author: ahug048
 */

#pragma once

#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <se306p1/AskPosition.h>
#include <se306p1/Position.h>
#include <se306p1/Do.h>
#include <se306p1/Go.h>
#include <se306p1/command.h>
#include <cstdint>
#include <queue>

#define ASK_POS_TOPIC "/supervisor/ask_pos"
#define ANS_POS_TOPIC "/supervisor/ans_pos"

/**
 * This header file defines the state variables and methods available to control a robot
 */
namespace se306p1 {
  class RobotController {
     private:
      // Robot identification
      int64_t robot_id_;

      // Position
      Pose position_;

      // Where the robot is currently trying to end up after receiving a go.
      Pose goal_;

      // Movement fields
      double lv_;  // Linear velocity
      double av_;  // Angular velocity (counter clockwise)

      // Loop control variables
      bool moving_;
      bool rotating_;
      bool dequeuing_;

      // Command queue
      std::deque<Command> commands_;

      // ROS Node handler for pub/subbing to topics
      ros::NodeHandle nh_;

      //Topic pub/subs
      ros::Subscriber askPosSubscriber_;
      ros::Subscriber doSubscriber_;
      ros::Subscriber goSubscriber_;
      ros::Publisher ansPosPublisher_;

     public:
      RobotController(int64_t id, Pose pose);
      virtual ~RobotController();
      void Move();
      void MoveTo(const Pose &pose, double lv);
      void Rotate();
      void go_callback(Go msg);
      void do_callback(Do msg);
      void askPosition_callback(AskPosition msg);
      void AnswerPosition();
      void ResolveCollision();
      void DequeueCommand();
      void Run();
  };
}
