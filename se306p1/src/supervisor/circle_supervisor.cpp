#include "circle_supervisor.h"

#include "../util/vector2.h"
#define ROBOT_WIDTH 0.35
#define DEFAULT_MOVE_SPEED 1

namespace se306p1 {
  CircleSupervisor::~CircleSupervisor() {  }

  void CircleSupervisor::Run() {

    ROS_INFO("Starting Circle Supervisor.");

    this->Discover(10);

    this->ElectHead();

    this->MoveNodesToDests(this->nonHeadRobots_, this->lineLocations_, DEFAULT_MOVE_SPEED);

  }

  void CircleSupervisor::ElectHead() {
    std::map<uint64_t, std::shared_ptr<Robot> >::iterator it;

    double clusterHeadDist = -1;
    for (it=this->robots_.begin(); it != this->robots_.end(); it++) {
      std::shared_ptr<Robot> robot_ptr = (*it).second;
      if (clusterHeadDist == -1) {
        this->clusterHead_ = robot_ptr;
      } else {
        double distToOrig = robot_ptr->position_.Length();
        if (distToOrig != 0 && distToOrig < clusterHeadDist) {

          this->nonHeadRobots_.push_back(this->clusterHead_);

          this->clusterHead_ = robot_ptr;
          clusterHeadDist = distToOrig;
        }else{
          this->nonHeadRobots_.push_back(robot_ptr);
        }
      }
    }

  } 

  void CircleSupervisor::FindRobotDests() {
    size_t numRobots = this->robots_.size();

    double theta = 0.5; //TODO: work out theta

    Vector2 lastLocation = this->clusterHead_->position_;
    Vector2 robotSep = lastLocation.Normalized() * ROBOT_WIDTH * 6;
    for(uint i = 1; i < numRobots; i++) {

      lastLocation = lastLocation + robotSep;

      this->lineLocations_.push_back(Pose(lastLocation, theta));
    }
  }
}