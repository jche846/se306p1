/*
 * RobotController.cpp
 *
 *  Created on: Aug 4, 2012
 *      Author: ahug048
 */

#include "robot_controller.h"
#include <math.h>

namespace se306p1 {
RobotController::RobotController() {
  this->x = 0;
  this->y = 0;
  this->linear = 0;
  this->theta = 0;
  this->angular = 0;
}

RobotController::~RobotController() {
  // TODO Auto-generated destructor stub
}

void RobotController::MoveTo(double x, double y, double theta) {
  // First get to position
  double dx = this->x - x;
  double dy = this->y - y;

  double a_tan = atan(dy / dx);
  if (dx >= 0 && dy >= 0) {
    // find theta
  } else if (dx < 0 && dy >= 0) {

  } else if (dx < 0 && dy < 0) {

  } else {

  }
  //Rotate so that we are facing the right way
}

void RobotController::ContinuousMove(double lv, double av) {
}

void RobotController::go_callback(Go message) {
}
void RobotController::do_callback(Do message) {
}
void RobotController::askPosition_callback(AskPosition message) {
}

void RobotController::AnswerPosition() {
}

void RobotController::ResolveCollision() {
}
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "robot_conroller");

  return 0;
}

