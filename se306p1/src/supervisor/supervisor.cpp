#include <limits>

#include "supervisor.h"
#include "robot.h"

#include <se306p1/Associate.h>
#include <se306p1/AskPosition.h>
#include <se306p1/Position.h>

#include "../util/pose.h"

#define FREQUENCY 1

namespace se306p1 {
  Supervisor::Supervisor() {
    // create publishers and subscribers
    ansPosSubscriber_ = nh_.subscribe<Position>(ANS_POS_TOPIC, 1000,
                                                &Supervisor::ansPos_callback, this,
                                                ros::TransportHints().reliable());
    askPosPublisher_ = nh_.advertise<AskPosition>(ASK_POS_TOPIC, 1000, true);
    assocPublisher_ = nh_.advertise<Associate>(ASSOCIATE_TOPIC, 1000, true);
  }

  Supervisor::~Supervisor() { }

  void Supervisor::ansPos_callback(Position msg) {
    std::shared_ptr<Robot> robot_ptr;
    if (robots_.find(msg.R_ID) != robots_.end()) {
      robot_ptr = robots_[msg.R_ID];
    } else {
      robot_ptr = NULL;
    }

    if (this->state_ == State::DISCOVERY) {
      if (robot_ptr != NULL) {
        if (robot_ptr->pose_.position_.x_ != msg.x ||
            robot_ptr->pose_.position_.y_ != msg.y ||
            robot_ptr->pose_.theta_ != msg.theta) {
          ROS_WARN("Robot %" PRId64 " changed its position during discovery.",
                   robots_[msg.R_ID]->id_);
        } else {
          return;
        }
      } else {
        robots_[msg.R_ID] = std::shared_ptr<Robot>(new Robot(msg.R_ID));
        robot_ptr = robots_[msg.R_ID];
        this->AssociateRobot(*robot_ptr);
      }
    }

    if(robot_ptr == NULL){
      return; // must not be in discovery mode so don't accept new robots
    }

    robot_ptr->Stop();
    robot_ptr->pose_ = Pose(Vector2(msg.x, msg.y), msg.theta);
    robot_ptr->executing_ = false;
  }

  void Supervisor::Discover(int timeout) {
    ROS_INFO("Discovering robots for %d seconds.", timeout);
    ros::Rate r(FREQUENCY);

    this->state_ = State::DISCOVERY;

    ros::Time end = ros::Time::now() + ros::Duration(timeout, 0);

    while (ros::ok() && ros::Time::now() <= end) {
      this->askPosPublisher_.publish(AskPosition());
      r.sleep();
      ros::spinOnce();
    }

    ROS_INFO("Discovered %zd robots.", robots_.size());
  }

  void Supervisor::AssociateRobot(const Robot &robot) {
    ROS_INFO("Supervisor associating with robot %" PRId64 ".", robot.id_);
    Associate msg;
    msg.R_ID = robot.id_;
    this->assocPublisher_.publish(msg);
  }

  void Supervisor::Start() {
    while (ros::Time::now().isZero());

    this->Discover(5);
    this->state_ = State::CONTROLLING;

    if (!this->robots_.size()) {
      ROS_ERROR("No robots discovered.");
      return;
    }

    this->ElectHead();

    this->Run();
  }

  void Supervisor::ElectHead() {
    std::map<uint64_t, std::shared_ptr<Robot> >::iterator it;

    double clusterHeadDist = -1;

    for (it = this->robots_.begin(); it != this->robots_.end(); it++) {
      std::shared_ptr<Robot> robot_ptr = (*it).second;
      if (clusterHeadDist == -1) {
        this->clusterHead_ = robot_ptr;
      } else {
        double distToOrig = robot_ptr->pose_.position_.Length();
        if (distToOrig != 0 && distToOrig < clusterHeadDist) {

          this->nonHeadRobots_.push_back(this->clusterHead_);

          this->clusterHead_ = robot_ptr;
          clusterHeadDist = distToOrig;
        } else {
          this->nonHeadRobots_.push_back(robot_ptr);
        }
      }
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
