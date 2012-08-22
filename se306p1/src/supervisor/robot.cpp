#include "robot.h"

#include <string>
#include <iostream>

#include <se306p1/Do.h>
#include <se306p1/Go.h>
#include <se306p1/Scan.h>

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

  std::stringstream scanss;
  scanss << "/robot_" << n << "/scan";
  scanPublisher_ = nh_.advertise<se306p1::Scan>(scanss.str(), 1000, statusCb);
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

    if (c.enqueue)
      ROS_INFO("R%" PRIu64 " | DISPATCHING DO, true", this->id_);
    else
      ROS_INFO("R%" PRIu64 " | DISPATCHING DO, false", this->id_);

    this->doPublisher_.publish(msg);
  } else if (c.type == CommandType::GO) {
    se306p1::Go msg;
    msg.x = c.x;
    msg.y = c.y;
    msg.theta = c.theta;
    msg.enqueue = c.enqueue;
    msg.errDist = c.errDist;
    msg.errTheta = c.errTheta;

    if (c.enqueue)
      ROS_INFO("R%" PRIu64 " | DISPATCHING GO, true", this->id_);
    else
      ROS_INFO("R%" PRIu64 " | DISPATCHING GO, false", this->id_);

    this->goPublisher_.publish(msg);
  } else if(c.type == CommandType::SCAN) {
    se306p1::Scan msg;
    msg.duration = c.duration;
    msg.enqueue = c.enqueue;
    this->scanPublisher_.publish(msg);
    if (c.enqueue)
      ROS_INFO("R%" PRIu64 " | DISPATCHING SCAN, true", this->id_);
    else
      ROS_INFO("R%" PRIu64 " | DISPATCHING SCAN, false", this->id_);
  }
}

void Robot::ScanBarcode(int time, bool enqueue) {
  Scan msg;
  msg.duration = time;
  msg.enqueue = enqueue;
  this->EnqueueCommand(Command(msg));

  this->executing_ = true;
}

void Robot::Go(const Pose &pos, bool enqueue, double errDist, double errTheta) {
  Command c;
  c.enqueue = enqueue;
  c.type = CommandType::GO;
  c.x = pos.position_.x_;
  c.y = pos.position_.y_;
  c.theta = pos.theta_;
  c.errDist = errDist;
  c.errTheta = errTheta;
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
