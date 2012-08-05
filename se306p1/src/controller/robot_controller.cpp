/*
 * RobotController.cpp
 *
 *  Created on: Aug 4, 2012
 *      Author: ahug048
 */

#include "robot_controller.h"
#include <cmath>
#include "../util/trig.h"
#include "../util/vector2.h"
#define FREQUENCY 10 // The number of ticks per second the robot will execute.
namespace se306p1 {
RobotController::RobotController(int64_t id, Pose pose) {
  // Initialise the robot as stationary with a given pose and ID.
  this->robot_id_ = id;
  this->position_ = pose;
  this->lv_ = 0;
  this->av_ = 0;

  // Initialise as stationary.
  this->moving_ = false;
  this->rotating_ = false;

  // Initially the robot is attempting to execute it's command queue.
  this->dequeuing_ = true;

  // We need to listen for the supervisor asking for our position.
  this->askPosSubscriber_ = nh_.subscribe<AskPosition>(
      ASK_POS_TOPIC, 1000, &RobotController::askPosition_callback, this,
      ros::TransportHints.reliable());

  // Subscribe to do messages in order to know when to move.
  this->doSubscriber_ = nh_.subscribe<Do>(DO_TOPIC, 1000,
                                          &RobotController::go_callback, this,
                                          ros::TransportHints.reliable());

  // Subscribe to go messages in order to know when to move.
  this->goSubscriber_ = nh_.subscribe<Go>(GO_TOPIC, 1000,
                                          &RobotController::do_callback, this,
                                          ros::TransportHints.reliable());

  // Publish our position when asked by the supervisor.
  this->ansPosPublisher_ = nh_.advertise<Position>(ANS_POS_TOPIC, 1000);

  /** ROS sub/pubs from Chandan
   //advertise() function will tell ROS that you want to publish on a given topic_
   //for other robots
   ros::Publisher RobotNode_pub = n.advertise<Project2Sample::R_ID>("Robot0_msg",1000);
   //to stage
   ros::Publisher RobotNode_stage_pub = n.advertise<geometry_msgs::Twist>("/robot_0/cmd_vel",1000);

   //subscribe to listen to messages of other robots
   ros::Subscriber RobotNode1_sub = n.subscribe<Project2Sample::R_ID>("Robot1_msg",1000, RobotNode1_callback);

   //subscribe to listen to messages coming from stage
   ros::Subscriber StageOdo_sub = n.subscribe<nav_msgs::Odometry>("Robot0_odo",1000, StageOdom_callback);
   ros::Subscriber StageLaser_sub = n.subscribe<sensor_msgs::LaserScan>("Robot0_laser",1000,StageLaser_callback);
   **/
}

RobotController::~RobotController() {
  // TODO Auto-generated destructor stub
}

/**
 * Moves the robot every ROS spin according to it's linear velocity until it is in the desired position.
 */
void RobotController::Move() {
  // Move linearly by lv / FREQUENCY.

  // If the goal is reached, stop moving.
  if (this->position_.position_ == this->goal_.position_) {
    this->moving_ = false;
  }

  // If there is no goal, keep moving.
}

/**
 * Rotates the robot every ROS spin according to it's angular velocity until it is facing the desired direction.
 */
void RobotController::Rotate() {
  // Rotate angularly by av / FREQUENCY.

  // If the right theta is reached, stop rotating.
  if (this->position_.theta_ == this->goal_.theta_) {
    this->rotating_ = false;
  }
}

/**
 * Moves the robot to a given position and aligns it so that it is facing the required direction
 * Parameters: x - x coordinate of where we want the robot to move to
 *             y - y coordinate of where we want the robot to move to
 *             theta - direction that we want the robot to face after movement
 */
void RobotController::MoveTo(const Pose &pose, double lv) {
  // First get to position
  double dx = pose.position_.x_;
  double dy = pose.position_.y_;
  double phi;

  this->goal_ = *pose;

  while (this->position_ != this->goal_) {
    // continue to move
  }

  double a_tan = DegATan(dy / dx);

  if (dx >= 0 && dy >= 0) {
    phi = a_tan;
  } else if (dx < 0 && dy >= 0) {
    phi = 180.0 - a_tan;
  } else if (dx < 0 && dy < 0) {
    phi = 180 + a_tan;
  } else {
    phi = 360 - a_tan;
  }
}

void RobotController::go_callback(Go msg) {
  if (msg.enqueue == true) {
    // If the message is for the command queue, queue it.
    this->commands_.push_back(Command(msg));
  } else {
    // Otherwise, interrupt the current action and execute the command.
    this->dequeuing_ = false;
    this->moving_ = false;

  }
}

void RobotController::do_callback(Do msg) {
  if (msg.enqueue == true) {
    // If the message is for the command queue, queue it.
    this->commands_.push_back(Command(msg));
  } else {
    // Otherwise, interrupt the current action and execute the command.

  }
}

void RobotController::askPosition_callback(AskPosition message) {
  this->AnswerPosition();
}

void RobotController::AnswerPosition() {
}

void RobotController::ResolveCollision() {
}

void RobotController::DequeueCommand() {
  // Get the next command from the queue.

  // If the command is a do, set the goal to blank

  // If the command is a go, set the goal.
}

void RobotController::Run() {
  ros::Rate r(FREQUENCY);  // Run FREQUENCY times a second

  while (ros::ok()) {
    // Move each tick
    if (this->moving_) {
      this->Move();
    }

    if (this->rotating_) {
      this->Rotate();
    }

    if (this->position_ == this->goal_) {
      // dequeue a new command and change the goal.
      this->DequeueCommand();
    }

    r.sleep();
    ros::spinOnce();
  }
}
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "robot_controller");
  se306p1::RobotController rc;
  rc.Run();
  return 0;
}
