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

    // rotating ensures that the robots are only told to rotate once
    bool rotating = false;

    while (ros::ok()) {
      this->DispatchMessages();

      if (!rotating) {
        bool exec_done = true;
        // iterate through all robots and see if they have finished executing
        for (auto &cur_robot: this->robots_) {
          if (cur_robot.second->executing) {
            // if any of the robots are executing wait
            exec_done = false;
            break;
          }
        }

        if (exec_done) {
          for (auto &cur_robot: this->robots_) {
            // tell robot to move to cluster head pos
            cur_robot.second->Go(this->clusterHead_->pose_, false);
            // enqueue rotate for after they have reached the cluster head pos
            cur_robot.second->Do(CIRCLE_LV, CIRCLE_AV, true);
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
