#include <limits>

#include "supervisor.h"
#include "robot.h"

#include <se306p1/AskPosition.h>
#include <se306p1/Position.h>

#include "../util/pose.h"

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

        if (robot_ptr->pose_.position_.x_ != msg.x ||
            robot_ptr->pose_.position_.y_ != msg.y ||
            robot_ptr->pose_.theta_ != msg.theta) {
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

      robot_ptr->pose_ = Pose(Vector2(msg.x, msg.y), msg.theta);
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

  void Supervisor::MoveNodesToDests(const std::vector<std::shared_ptr<Robot> > &nodesIn,
                                    const std::vector<Pose> &posesIn) {
    std::vector<std::shared_ptr<Robot> > nodes = nodesIn;
    std::vector<Pose> poses = posesIn;

    while (poses.size()) {
      double longestDist = -1;
      int longestNodeIndex = 0;
      int longestPoseIndex = 0;

      for (size_t nodeIndex = 0; nodeIndex < nodes.size(); nodeIndex++) {
        double dist = std::numeric_limits<double>::max();
        int destIndex = 0;
        std::shared_ptr<Robot> node = nodes[nodeIndex];
        for(size_t posesIndex = 0; posesIndex < poses.size(); posesIndex++){
          double testDist = abs((node->pose_.position_ - poses[posesIndex].position_).LengthSquared());
          if(testDist < dist){
            dist = testDist;
            destIndex = posesIndex;
          }
        }
        if (dist > longestDist) {
          longestDist = dist;
          longestNodeIndex = nodeIndex;
          longestPoseIndex = destIndex;
        }
      }
      nodes[longestNodeIndex]->Go(poses[longestPoseIndex], false);
      nodes.erase(nodes.begin() + longestNodeIndex);
      poses.erase(poses.begin() + longestPoseIndex);
    }
  }
}
