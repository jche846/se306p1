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

    this->x_ = 0;
    this->y_ = 0;
    this->theta_ = 0;
    this->lv_ = 0;
    this->av_ = 0;

    this->moving_ = false;
    this->dequeuing_ = true;
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

  void RobotController::go_callback(Go msg) {
  }
  void RobotController::do_callback(Do msg) {
    this->lv_ = msg->lv;
    this->av_ = msg->av;

    if (msg->enqueue == true) {
      this->commands_.
    }

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

    while (ros::ok()) {
      //do stuff
      ros::spinOnce();
    }

    return 0;
}

