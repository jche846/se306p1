#include "rotate_supervisor.h"

#include "../util/vector2.h"
#include "../util/trig.h"

#define ROBOT_WIDTH 0.35
#define DEFAULT_MOVE_SPEED 1

namespace se306p1 {
  RotateSupervisor::~RotateSupervisor() {  }

  void RotateSupervisor::Run() {
    ROS_INFO("Starting rotate supervisor.");
    ros::Rate r(100);

    // TODO: remove this code!
    for(std::pair<const uint64_t, std::shared_ptr<Robot>> &pair : this->robots_) {
      pair.second->Go(Pose(Vector2(5.0, 5.0), 180), false);
    }

    while (ros::ok()) {
      this->DispatchMessages();
      r.sleep();
      ros::spinOnce();
    }

    //this->FindRobotDests();
    //this->MoveNodesToDests(this->nonHeadRobots_, this->lineLocations_);

    //wait for robots to arrive in line

    // start cluster head moving

    // Go other 5 to cluster head

    // Queue Do on other 5 to move in circle
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
  s.Start();
  return 0;
}
#endif
