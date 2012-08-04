#include "supervisor.h"

#include <se306p1/AskPosition.h>
#include <se306p1/Position.h>

#include "robot.h"

namespace se306p1 {
  Supervisor::Supervisor() {
    // create publishers and subscribers
    ansPosSubscriber_ = nh_.subscribe<Position>(ANS_POS_TOPIC, 1000, &Supervisor::ansPos_callback, this, ros::TransportHints().reliable());
    askPosPublisher_ = nh_.advertise<AskPosition>(ASK_POS_TOPIC, 1000);
  }

  Supervisor::~Supervisor() { }

  void Supervisor::ansPos_callback(Position msg) {
    if (this->state_ == DISCOVERY) {
      std::shared_ptr<Robot> robot_ptr;

      if (robots_.find(msg.R_ID) != robots_.end()) {
        robot_ptr = robots_[msg.R_ID];

        if (robot_ptr->position_.x_ != msg.x ||
            robot_ptr->position_.y_ != msg.y ||
            robot_ptr->theta_ != msg.theta) {
          ROS_WARN("Robot %ld changed its position during discovery.",
                   robots_[msg.R_ID]->id_);
        } else {
          return;
        }
      } else {
        robots_[msg.R_ID] = std::shared_ptr<Robot>(new Robot(msg.R_ID));
        robot_ptr = robots_[msg.R_ID];
        ROS_INFO("Hello robot %ld!", robots_[msg.R_ID]->id_);
      }

      robot_ptr->position_ = Vector2(msg.x, msg.y);
      robot_ptr->theta_ = msg.theta;
      robot_ptr->Stop();
    }
  }

  void Supervisor::Discover(int timeout) {
    ROS_INFO("Discovering robots for %d seconds.", timeout);
    this->state_ = DISCOVERY;

    ros::Time end = ros::Time::now() + ros::Duration(timeout, 0);

    while (ros::ok() && ros::Time::now() <= end) {
      this->askPosPublisher_.publish(AskPosition());
      ros::spinOnce();
    }

    ROS_INFO("Discovered %zd robots.", robots_.size());
  }

  void Supervisor::Run() {
    while (ros::Time::now().isZero());

    this->Discover(10);
    this->state_ = CONTROLLING;
    while (ros::ok()) {
      ros::spinOnce();
    }
  }
}
