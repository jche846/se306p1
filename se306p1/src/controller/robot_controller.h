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
#include "../util/pose.h"
#include "../util/command.h"
#include <deque>

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
      Pose pose_;

      // Where the robot is currently trying to end up after receiving a go.
      Pose goal_;

      // Movement fields
      double lv_;  // Linear velocity
      double av_;  // Angular velocity (counter clockwise)

      // Loop control variables
      bool doing_;
      bool going_;
      bool moving_;
      bool aiming_;

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
      RobotController(ros::NodeHandle &nh, int64_t id);
      virtual ~RobotController();
      double AngleToGoal();
      void go_callback(Go msg);
      void do_callback(Do msg);
      void askPosition_callback(AskPosition msg);
      void AnswerPosition();
      void odom_callback(nav_msgs::Odometry msg);
      void Move();
      void SetGoing(Go msg);
      void SetDoing(Do msg);
      void ExecuteCommand(Command cmd);
      void DequeCommand();
      void InterruptCommandQueue(Command cmd);
      void Run();
  };
}
