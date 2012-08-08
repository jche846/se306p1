#include "rotate_supervisor.h"

#include "../util/vector2.h"
#include "../util/trig.h"

#define ROBOT_WIDTH 0.35
#define DEFAULT_MOVE_SPEED 1
#define CIRCLE_LV 2
#define CIRCLE_AV 2

namespace se306p1 {
  RotateSupervisor::~RotateSupervisor() {
  }

  void RotateSupervisor::Run() {
    ROS_INFO("Starting rotate supervisor.");
    ros::Rate r(100);

    this->FindRobotDests();
    this->MoveNodesToDests(this->nonHeadRobots_, this->lineLocations_);

    bool rotating = false;

    while (ros::ok()) {
      this->DispatchMessages();

      if (!rotating) {
        // iterate through all robots and see if they have finished executing
        bool allFinishedExecuting = true;
        //if any robot is executing, set allFinishedExecuting to false
        for (auto &cur_robot: this->robots_) {
          if (cur_robot.second->executing) {
            allFinishedExecuting = false;
            break;
          }
        }

        // tell all robots to move to clusterHeadPosition
        // and enqueue them moving in a circle.
        if (allFinishedExecuting) {
          for (auto &cur_robot: this->robots_) {
            cur_robot.second->Go(this->clusterHead_->pose_, false); //moveTocluster head position
            cur_robot.second->Do(CIRCLE_LV, CIRCLE_AV, true); //queue circling
          }
          rotating = true;
        }
      }

      r.sleep();
      ros::spinOnce();

    }
  }

  void RotateSupervisor::FindRobotDests() {
    size_t numRobots = this->robots_.size();

    double theta = 0.5; //TODO: work out theta

    Vector2 lastLocation = this->clusterHead_->pose_.position_;
    Vector2 robotSep = lastLocation.Normalized() * ROBOT_WIDTH * 6;
    for (size_t i = 1; i < numRobots; i++) {
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
