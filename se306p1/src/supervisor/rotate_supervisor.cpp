#include "rotate_supervisor.h"

#include "../util/vector2.h"
#include "../util/trig.h"

#define ROBOT_WIDTH 0.35
#define DEFAULT_MOVE_SPEED 1

namespace se306p1 {
  RotateSupervisor::~RotateSupervisor() {
  }

  /**
   * Runs the Alpha version behaviour in the robots.
   *
   * @author Sam Grace, Andrew Luey
   */
  void RotateSupervisor::Run() {
    ROS_INFO("Starting rotate supervisor.");
    ros::Rate r(100);

    this->FindRobotDests();
    this->MoveNodesToDests(this->nonHeadRobots_, this->lineLocations_);

    // rotating ensures that the robots are only told to rotate once
    bool rotating = false;

    double circle_av = 0.62;
    double CIRCLE_LV = 1.0;

    ROS_INFO("MOVING AT lv:%f av:%f", CIRCLE_LV, circle_av);
    //The while loop that repeatedly runs to make the robots move to new positions.

    while (ros::ok()) {
      this->DispatchMessages();

      if (!rotating) {
        bool exec_done = true;
        // iterate through all robots and see if they have finished executing
        for (auto &cur_robot : this->robots_) {
          if (cur_robot.second->executing_) {
            // if any of the robots are executing wait
            exec_done = false;
            break;
          }
        }

        /**
         * this block is only ever called once. It starts the robots moving in a circle,
         * then sets rotating to true so this block is never called again.
         */

        if (exec_done) {
          this->clusterHead_->pose_.theta_ = 999;
          for (auto &cur_robot : this->robots_) {
            if (cur_robot.second->id_ != this->clusterHead_->id_) {
              // tell robot to move to cluster head pos
              cur_robot.second->Go(this->clusterHead_->pose_, false);
            }
            // enqueue rotate for after they have reached the cluster head pos
            cur_robot.second->Do(
                CIRCLE_LV, circle_av,
                cur_robot.second->id_ != this->clusterHead_->id_);
          }
          rotating = true;
        }
      }

      r.sleep();
      ros::spinOnce();

    }
  }

  /**
   * Sets the pose for all robots in this->lineLocations for all robots except
   * the head to arrange in a a radial line out from the (0,0) origin through
   * the cluster head position, and spaced evenly at 5 robot diameters apart.
   *
   * Sets the cluster head to align towards the origin.
   *
   * @author scott goodhew
   */
  void RotateSupervisor::FindRobotDests() {
    size_t numRobots = this->robots_.size();

    double goalTheta = AngleBetweenPoints(this->clusterHead_->pose_.position_,
                                          Vector2());
    ROS_INFO("Setting GoalTheta to %f", goalTheta);

    // set goal theta of cluster head
    Pose headPose = this->clusterHead_->pose_;
    headPose.theta_ = goalTheta;
    this->clusterHead_->Go(headPose, false);

    Vector2 lastLocation = this->clusterHead_->pose_.position_;
    Vector2 robotSep = lastLocation.Normalized() * ROBOT_WIDTH * 6;
    for (size_t i = 1; i < numRobots; i++) {
      lastLocation = lastLocation + robotSep;
      this->lineLocations_.push_back(Pose(lastLocation, goalTheta));
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
