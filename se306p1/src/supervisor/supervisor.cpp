#include "supervisor.h"

#include <se306p1/AskPosition.h>
#include <se306p1/Position.h>

#include "robot.h"

namespace se306p1 {
  Supervisor::Supervisor() {
    // create publishers and subscribers
    _ansPosSubscriber = _nh.subscribe<Position>(ANS_POS_TOPIC, 1000, &Supervisor::ansPos_callback, this, ros::TransportHints().reliable());
    _askPosPublisher = _nh.advertise<AskPosition>(ASK_POS_TOPIC, 1000);
  }

  Supervisor::~Supervisor() { }

  void Supervisor::ansPos_callback(Position msg) {
    if (this->_state == DISCOVERY) {
      std::shared_ptr<Robot> robot_ptr;

      if (_robots.find(msg.R_ID) != _robots.end()) {
        robot_ptr = _robots[msg.R_ID];

        if (robot_ptr-> x != msg.x || robot_ptr->y != msg.y ||
           robot_ptr->theta != msg.theta) {
          ROS_WARN("Robot %ld changed its position during discovery.",
                   _robots[msg.R_ID]->id);
        } else {
          return;
        }
      } else {
        _robots[msg.R_ID] = std::shared_ptr<Robot>(new Robot(msg.R_ID));
        robot_ptr = _robots[msg.R_ID];
        ROS_INFO("Hello robot %ld!", _robots[msg.R_ID]->id);
      }

      robot_ptr->x = msg.x;
      robot_ptr->y = msg.y;
      robot_ptr->theta = msg.theta;
      robot_ptr->stop();
    }
  }

  void Supervisor::discover(int timeout) {
    ROS_INFO("Discovering robots for %d seconds.", timeout);
    this->_state = DISCOVERY;

    ros::Time end = ros::Time::now() + ros::Duration(timeout, 0);

    while (ros::ok() && ros::Time::now() <= end) {
      this->_askPosPublisher.publish(AskPosition());
      ros::spinOnce();
    }

    ROS_INFO("Discovered %zd robots.", _robots.size());
  }

  void Supervisor::run() {
    while(ros::Time::now().isZero());

    this->discover(10);
    this->_state = CONTROLLING;
    while (ros::ok()) {
      ros::spinOnce();
    }
  }
}
