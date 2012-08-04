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

  void Robot::Go(const Vector2 &position, double theta, double lv, bool enqueue) {
    se306p1::Go msg;
    msg.x = position.x_;
    msg.y = position.y_;
    msg.theta = theta;
    msg.lv = lv;
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
