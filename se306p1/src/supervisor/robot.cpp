#include "robot.h"

#include <string>
#include <iostream>

#include <se306p1/Do.h>
#include <se306p1/Go.h>

namespace se306p1 {
Robot::Robot(uint64_t n) {
  this->id_ = n;
  this->readiness_ = Readiness::NOT_READY;

  ros::SubscriberStatusCallback statusCb =
      [this](const ros::SingleSubscriberPublisher& pub) -> void {
        switch(this->readiness_) {
          case Readiness::NOT_READY:
          this->readiness_ = Readiness::HALF_READY;
          return;
          default:
          this->readiness_ = Readiness::READY;
          ROS_INFO("Controller for robot %" PRId64 " ready.", this->id_);
          return;
        }
      };

  // create publishers and subscribers
  std::stringstream doss;
  doss << "/robot_" << n << "/do";
  doPublisher_ = nh_.advertise<se306p1::Do>(doss.str(), 1000, statusCb);

  std::stringstream goss;
  goss << "/robot_" << n << "/go";
  goPublisher_ = nh_.advertise<se306p1::Go>(goss.str(), 1000, statusCb);
}

Robot::~Robot() {
}

void Robot::EnqueueCommand(Command c) {
  this->commands_.push_back(c);
}

void Robot::DispatchCommand() {
  if (this->commands_.empty())
    return;

  Command c = this->commands_.front();
  this->commands_.pop_front();

  if (c.type == CommandType::DO) {
    se306p1::Do msg;
    msg.lv = c.lv;
    msg.av = c.av;
    msg.enqueue = c.enqueue;

    this->doPublisher_.publish(msg);
  } else if (c.type == CommandType::GO) {
    se306p1::Go msg;
    msg.x = c.x;
    msg.y = c.y;
    msg.theta = c.theta;
    msg.enqueue = c.enqueue;

    this->goPublisher_.publish(msg);
  }
}

void Robot::Go(const Pose &pos, bool enqueue) {
  Command c;
  c.enqueue = enqueue;
  c.type = CommandType::GO;
  c.x = pos.position_.x_;
  c.y = pos.position_.y_;
  c.theta = pos.theta_;
  this->EnqueueCommand(c);

  this->executing_ = true;
}

void Robot::Stop() {
  this->Do(0, 0, false);
}

void Robot::Do(double lv, double av, bool enqueue) {
  Command c;
  c.enqueue = enqueue;
  c.type = CommandType::DO;
  c.lv = lv;
  c.av = av;
  this->EnqueueCommand(c);

  this->executing_ = true;
}
}
