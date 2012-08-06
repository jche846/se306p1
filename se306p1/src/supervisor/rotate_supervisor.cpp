#include "rotate_supervisor.h"

#include "../util/vector2.h"
#define ROBOT_WIDTH 0.35
#define DEFAULT_MOVE_SPEED 1

namespace se306p1 {
  RotateSupervisor::~RotateSupervisor() {  }

  void RotateSupervisor::Run() {
    ROS_INFO("Starting Rotate Supervisor.");
    this->Discover(10);
    this->ElectHead();
    this->MoveNodesToDests(this->nonHeadRobots_, this->lineLocations_);
  }

  void RotateSupervisor::ElectHead() {
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

  void RotateSupervisor::FindRobotDests() {
    size_t numRobots = this->robots_.size();

    double theta = 0.5; //TODO: work out theta

    Vector2 lastLocation = this->clusterHead_->pose_.position_;
    Vector2 robotSep = lastLocation.Normalized() * ROBOT_WIDTH * 6;
    for(size_t i = 1; i < numRobots; i++) {
      lastLocation = lastLocation + robotSep;
      this->lineLocations_.push_back(Pose(lastLocation, theta));
    }
  }
}

#ifdef ROTATE_SUPERVISOR_MAIN
int main(int argc, char *argv[]) {
  ros::init(argc, argv, "rotate_supervisor");
  se306p1::RotateSupervisor s;
  s.Run();
  return 0;
}
#endif
