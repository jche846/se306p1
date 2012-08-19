#include <limits>
#include <memory>

#include "supervisor.h"
#include "robot.h"

#include <se306p1/AskPosition.h>
#include <se306p1/Position.h>

#include "../util/pose.h"
#include "../util/trig.h"

#include "behaviors/all.h"

#define FREQUENCY 1
#define ROBOT_WIDTH 0.35

namespace se306p1 {
Supervisor::Supervisor(ros::NodeHandle &nh) : nh_(nh) {
  // create publishers and subscribers
  ansPosSubscriber_ = nh_.subscribe<Position>(ANS_POS_TOPIC, 1000,
                                              &Supervisor::ansPos_callback,
                                              this,
                                              ros::TransportHints().reliable());
  askPosPublisher_ = nh_.advertise<AskPosition>(ASK_POS_TOPIC, 1000);

  int sid;
  nh_.getParam("sid", sid);

  this->sid_ = static_cast<uint64_t>(sid);

  this->RegisterBehaviors();

  // TODO: remove this, it's hardcoded!
  this->SwitchBehavior(GotoSquareBehavior::id());
}

void Supervisor::ansPos_callback(Position msg) {
  if (msg.R_ID < this->rmin_ || msg.R_ID > this->rmax_) return;

  // attempt to find the robot in the list of found robots
  std::shared_ptr<Robot> robot_ptr;
  if (robots_.find(msg.R_ID) != robots_.end()) {
    robot_ptr = robots_[msg.R_ID];
  } else {
    robot_ptr = nullptr;
  }

  // Discovery state
  if (this->state_ == State::DISCOVERY) {
    // robot has already been found so check its not moving.
    if (robot_ptr != nullptr) {
      if (robot_ptr->pose_.position_.x_ != msg.x
          || robot_ptr->pose_.position_.y_ != msg.y
          || robot_ptr->pose_.theta_ != msg.theta) {
        ROS_WARN(
            "Supervisor %" PRId64 ": Robot %" PRId64 " changed its position during discovery.", this->sid_, robots_[msg.R_ID]->id_);
      } else {
        return;  // robot wasn't moving
      }
    } else {  // need to create a new robot and put it in the robot map
      robots_[msg.R_ID] = std::shared_ptr<Robot>(new Robot(msg.R_ID));
      robot_ptr = robots_[msg.R_ID];
      ROS_INFO("Supervisor %" PRId64 ": Associating with robot %" PRId64 ".", this->sid_, msg.R_ID);
    }
  }

  if (robot_ptr == nullptr) {
    return;  // if the pointer is still null then we we are not in discovery mode and are not accepting new robots so ignore it.
  }
  // ensure the robot is stopped
//    robot_ptr->Stop();
  // record its location
  robot_ptr->pose_ = Pose(Vector2(msg.x, msg.y), msg.theta);
  robot_ptr->executing_ = false;
}

void Supervisor::Discover(int timeout) {
  int rmin, rmax;

  nh_.getParam("rmin", rmin);
  nh_.getParam("rmax", rmax);

  this->rmin_ = static_cast<uint64_t>(rmin);
  this->rmax_ = static_cast<uint64_t>(rmax);

  ROS_INFO("Supervisor %" PRId64 ": Discovering robots from %d to %d for %d seconds.", this->sid_, rmin, rmax, timeout);
  ros::Rate r(FREQUENCY);

  // set state for the ansPos_callback
  this->state_ = State::DISCOVERY;

  ros::Time end = ros::Time::now() + ros::Duration(timeout, 0);

  // blast askPos messages for timeout
  while (ros::ok() && ros::Time::now() <= end) {
    this->askPosPublisher_.publish(AskPosition());
    r.sleep();
    ros::spinOnce();
  }

  ROS_INFO("Supervisor %" PRId64 ": Discovered %zd robots.", this->sid_, robots_.size());
}

void Supervisor::WaitForReady() {
  ROS_INFO("Supervisor %" PRId64 ": Waiting for robots to become ready.", this->sid_);
  ros::Rate r(FREQUENCY);

  this->state_ = State::WAITING;

  while (ros::ok()) {
    bool ready = true;
    for (auto &pair : this->robots_) {
      if (pair.second->readiness_ != Robot::Readiness::READY) {
        ready = false;
        break;
      }
    }
    if (!ready) {
      r.sleep();
      ros::spinOnce();
      continue;
    }
    break;
  }

  ROS_INFO("Supervisor %" PRId64 ": Robots ready.", this->sid_);
}

void Supervisor::Start() {
  while (ros::Time::now().isZero());

  this->Discover(5);
  this->state_ = State::CONTROLLING;

  if (!this->robots_.size()) {
    ROS_ERROR("Supervisor %" PRId64 ": No robots discovered.", this->sid_);
    return;
  }

  this->WaitForReady();

  this->dispatchIt_ = robots_.begin();

  this->ElectHead();

  ros::Rate r(100);

  this->MoveNodesToDests(this->nonHeadRobots_, this->FindRobotDests());

  ROS_INFO("Supervisor %" PRId64 ": Spinning up.", this->sid_);

  while (ros::ok()) {
    this->DispatchMessages();

    bool execDone = true;
    for (auto &robot : this->robots_) {
      if (robot.second->executing_) {
        execDone = false;
        break;
      }
    }

    if (execDone) this->currentBehavior_->Tick();

    r.sleep();
    ros::spinOnce();
  }
}

void Supervisor::ElectHead() {
  std::map<uint64_t, std::shared_ptr<Robot> >::iterator it;

  double clusterHeadDist = -1;

  for (auto &pair : this->robots_) {
    double distToOrig = pair.second->pose_.position_.Length();
    if (clusterHeadDist == -1 && distToOrig != 0) {
      this->clusterHead_ = pair.second;
      clusterHeadDist = distToOrig;
    } else {
      if (distToOrig != 0 && distToOrig < clusterHeadDist) {

        this->nonHeadRobots_.push_back(this->clusterHead_);

        this->clusterHead_ = pair.second;
        clusterHeadDist = distToOrig;
      } else {
        this->nonHeadRobots_.push_back(pair.second);
      }
    }
  }
  ROS_INFO("Elected R%ld as Cluster Head", this->clusterHead_->id_);
}

void Supervisor::DispatchMessages() {
  dispatchIt_++;
  if (dispatchIt_ == robots_.end())
    dispatchIt_ = robots_.begin();
  dispatchIt_->second->DispatchCommand();
//    ROS_INFO("Dispatched command for robot %" PRId64 ".", dispatchIt_->first);
}

std::vector<Pose> Supervisor::FindRobotDests() {
  std::vector<Pose> lineLocations;

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
    lineLocations.push_back(Pose(lastLocation, goalTheta));
  }

  return lineLocations;
}

void Supervisor::MoveNodesToDests(
    const std::vector<std::shared_ptr<Robot> > &nodesIn,
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
      for (size_t posesIndex = 0; posesIndex < poses.size(); posesIndex++) {
        double testDist =
            abs((node->pose_.position_ - poses[posesIndex].position_)
                .LengthSquared());
        if (testDist < dist) {
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

void Supervisor::SwitchBehavior(uint64_t id) {
  this->currentBehavior_ = this->behaviorFactories_[id](*this);
}
}

#ifdef SUPERVISOR_MAIN
int main(int argc, char *argv[]) {
  ros::init(argc, argv, "supervisor", ros::init_options::AnonymousName);
  ros::NodeHandle nh("~");

  se306p1::Supervisor s(nh);
  s.Start();
  return 0;
}
#endif
