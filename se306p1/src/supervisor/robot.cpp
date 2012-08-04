#include "robot.h"

#include <string>
#include <iostream>

#include <se306p1/Do.h>
#include <se306p1/Go.h>

namespace se306p1 {
  Robot::Robot(uint64_t n) {
    this->id = n;

    // create publishers and subscribers
    std::stringstream doss;
    doss << "/robot_" << n << "/do";

    _doPublisher = _nh.advertise<Do>(doss.str(), 1000);

    std::stringstream goss;
    goss << "/robot_" << n << "/go";
    _goPublisher = _nh.advertise<Go>(goss.str(), 1000);
  }

  Robot::~Robot() {}

  void Robot::go(double x, double y, double theta, double lv, bool enqueue) {
    Go msg;
    msg.x = x;
    msg.y = y;
    msg.theta = theta;
    msg.lv = lv;
    msg.enqueue = enqueue;

    _goPublisher.publish(msg);
  }

  void Robot::stop() {
    this->do_(0, 0, false);
  }

  void Robot::do_(double lv, double av, bool enqueue) {
    Do msg;
    msg.lv = lv;
    msg.av = av;
    msg.enqueue = enqueue;

    _doPublisher.publish(msg);
  }
}
