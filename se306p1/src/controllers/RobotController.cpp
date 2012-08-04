/*
 * RobotController.cpp
 *
 *  Created on: Aug 4, 2012
 *      Author: ahug048
 */

#include "RobotController.h"
#include <math.h>

RobotController::RobotController() {

}

RobotController::~RobotController() {
  // TODO Auto-generated destructor stub
}

RobotController::MoveTo(double x, double y, double theta) {
  // First get to position
  double dx = this.x - x;
  double dy = this.y - y;

  double a_tan = atan(dy/dx);
  if (dx >= 0 && dy >= 0) {
    // find theta
  } else if (dx < 0 && dy >= 0) {

  } else if (dx < 0 && dy < 0) {

  } else {

  }
  //Rotate so that we are facing the right way

}
