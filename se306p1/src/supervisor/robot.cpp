#include "robot.h"

#include <string>
#include <iostream>

#include <se306p1/Do.h>
#include <se306p1/Go.h>

namespace se306p1 {
  Robot::Robot(uint64_t n) {
    this->id_ = n;

    // create publishers and subscribers
    std::stringstream doss;
    doss << "/robot_" << n << "/do";

    doPublisher_ = nh_.advertise<se306p1::Do>(doss.str(), 1000);

    std::stringstream goss;
    goss << "/robot_" << n << "/go";
    goPublisher_ = nh_.advertise<se306p1::Go>(goss.str(), 1000);
  }

  Robot::~Robot() {}

  void Robot::Go(const Pose &pos, bool enqueue) {
    se306p1::Go msg;
    msg.x = pos.position_.x_;
    msg.y = pos.position_.y_;
    msg.theta = pos.theta_;
    msg.enqueue = enqueue;

    goPublisher_.publish(msg);
  }

  void Robot::Stop() {
    this->Do(0, 0, false);
  }

  void Robot::Do(double lv, double av, bool enqueue) {
    se306p1::Do msg;
    msg.lv = lv;
    msg.av = av;
    msg.enqueue = enqueue;

    doPublisher_.publish(msg);
  }
}
