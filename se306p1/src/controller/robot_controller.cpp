#include "robot_controller.h"
#include "../util/trig.h"
#include <string>

#define FREQUENCY 100 // The number of ticks per second the robot will execute.
#define DEFAULT_LV 1.0
#define DEFAULT_AV 2.0
#define SCAN_TIME 5

namespace se306p1 {
RobotController::RobotController(ros::NodeHandle &nh, uint64_t id = 0) {
  // Node handler to talk to ROS.
  this->nh_ = nh;

  // Initialise the robot as stationary with a given ID.
  this->robot_id_ = id;
  this->lv_ = 0;
  this->av_ = 0;

  // Initialise as waiting for new commands.
  this->state_ = RobotState::READY;

  // We need to listen for the supervisor asking for our position.
  this->askPosSubscriber_ = nh_.subscribe<AskPosition>(
      ASK_POS_TOPIC, 1000, &RobotController::askPosition_callback, this,
      ros::TransportHints().reliable());

  // Publish our position when asked by the supervisor.
  this->ansPosPublisher_ = nh_.advertise<Position>(ANS_POS_TOPIC, 1000);

  // Subscribe to clock messages from Stage
  this->clock_ = nh_.subscribe<rosgraph_msgs::Clock>(
      "/clock", 1000, &RobotController::clock_callback, this,
      ros::TransportHints().reliable());

  // Subscribe to pose messages from Stage
  std::stringstream odomss;
  odomss << "/robot_" << this->robot_id_ << "/odom";
  this->odom_ = nh_.subscribe<nav_msgs::Odometry>(
      odomss.str(), 1000, &RobotController::odom_callback, this,
      ros::TransportHints().reliable());

  // Subscribe to laser messages from Stage
  std::stringstream laserss;
  laserss << "/robot_" << this->robot_id_ << "/base_scan";
  this->baseScan_ = nh_.subscribe<sensor_msgs::LaserScan>(
      laserss.str(), 1000, &RobotController::baseScan_callback, this,
      ros::TransportHints().reliable());

  // Publish ScanResults to the supervisor on completion
  std::stringstream scanResultss;
  scanResultss << "/robot_" << this->robot_id_ << "/scan_result";
  this->scanResultPublisher_ = nh_.advertise<ScanResult>(scanResultss.str(),
                                                         1000);

  // Subscribe to Do messages in order to know when to move.
  std::stringstream doss;
  doss << "/robot_" << this->robot_id_ << "/do";
  this->doSubscriber_ = nh_.subscribe<Do>(doss.str(), 1000,
                                          &RobotController::do_callback, this,
                                          ros::TransportHints().reliable());

  // Subscribe to Go messages in order to know when to move.
  std::stringstream goss;
  goss << "/robot_" << this->robot_id_ << "/go";
  this->goSubscriber_ = nh_.subscribe<Go>(goss.str(), 1000,
                                          &RobotController::go_callback, this,
                                          ros::TransportHints().reliable());

  // Tell stage that we are going to advertise commands to our robot.
  std::stringstream twistss;
  twistss << "/robot_" << this->robot_id_ << "/cmd_vel";
  this->twist_ = nh_.advertise<geometry_msgs::Twist>(twistss.str(), 1000);
}

RobotController::~RobotController() {
  // TODO Auto-generated destructor stub
}

/**
 * On each clock callback, execute the robot behaviour according to its
 * internal state.
 *
 * @param msg The current time since Stage was launched.
 */
void RobotController::clock_callback(rosgraph_msgs::Clock msg) {
  if (this->state_ == RobotState::GOING) {  // If the robot is going somewhere, keep trying to go there
    this->MoveTowardsGoal();
  } else if (this->state_ == RobotState::DOING) {  // If the robot is doing, keep doing.
    this->PublishVelocity();
  } else if (this->state_ == RobotState::SCANNING) {
    this->Scan();
  } else if (this->state_ == RobotState::READY) {  // Get the next command.
    this->DequeueCommand();
  }
}

/**
 * Called when stage publishes an odometry message. Updates the controller's
 * knowledge of the robot's current position, and triggers the controller to
 * update the linear and angular velocity to achieve the robot's goal.
 *
 * @param msg The odometry message containing position data for this robot.
 */
void RobotController::odom_callback(nav_msgs::Odometry msg) {
  double roll;
  double pitch;
  double yaw;

  // Set the position in 2D space.
  this->pose_.position_.x_ = -1 * msg.pose.pose.position.y;
  this->pose_.position_.y_ = msg.pose.pose.position.x;

  // Set the rotation.
  QuaternionMsgToRPY(msg.pose.pose.orientation, roll, pitch, yaw);
  this->pose_.theta_ = RadiansToDegrees(yaw);

//  ROS_INFO(
//      "R%" PRIu64 " ODOM | x=%f, y=%f, theta=%f, lv=%f, av=%f", this->robot_id_, this->pose_.position_.x_, this->pose_.position_.y_, this->pose_.theta_, this->lv_, this->av_);
}

/**
 * Called when the supervisor publishes a Scan message for this robot. If the
 * message is to be enqueued, the message is put on the queue. Otherwise, the
 * command queue is interrupted and thrown away and the Scan message is executed
 * immediately.
 *
 * @param msg A Scan message telling the robot to scan.
 */
void RobotController::scan_callback(se306p1::Scan msg) {
  this->ReceiveCommand(Command(msg));
  ROS_INFO( "R%" PRIu64 " SCAN", this->robot_id_);
}

/**
 * Called when the supervisor publishes a Go message for this robot. If the
 * message is to be enqueued, the message is put on the queue. Otherwise, the
 * command queue is interrupted and thrown away and the Go message is executed
 * immediately.
 *
 * @param msg A Go message containing a location for the robot to move to.
 */
void RobotController::go_callback(Go msg) {
  this->ReceiveCommand(Command(msg));
  ROS_INFO(
      "R%" PRIu64 " GO | x=%f, y=%f, theta=%f", this->robot_id_, msg.x, msg.y, msg.theta);
}

/**
 * Called when the supervisor publishes a Do message for this robot. If the
 * message is to be enqueued, the message is put on the queue. Otherwise, the
 * command queue is interrupted and thrown away and the Do message is executed
 * immediately.
 *
 * @param msg A Do message containing linear and angular velocity information.
 */
void RobotController::do_callback(Do msg) {
  this->ReceiveCommand(Command(msg));
  ROS_INFO( "R%" PRIu64 " DO | lv=%f, av=%f", this->robot_id_, msg.lv, msg.av);
}

/**
 * Called when the supervisor publishes a request for robots to call home.
 * Calls the AnswerPosition() method in response.
 *
 * @param msg A blank message.
 */
void RobotController::askPosition_callback(AskPosition msg) {
  this->AnswerPosition();
}

/**
 * If the robot state is scanning, the scanResult will be set to the number of
 * obstacles in the laser range.
 *
 * @param msg The LaserScan message containing laser scan data for this robot.
 */
void RobotController::baseScan_callback(sensor_msgs::LaserScan msg) {
  if (this->state_ == RobotState::SCANNING) {
    int barCount = 0;

    for (int i = 0; i < 180; i++) {
      if (msg.ranges[i] != 5.0) {
        barCount++;

        // Run over the current bar
        while (msg.ranges[i] != 5.0 && i < 180) {
          i++;
        }
      }
    }

//    ROS_INFO( "R%" PRIu64 " SCANNED %d", this->robot_id_, barCount);

    this->scanResult_ = barCount;
  }
}

/**
 * Publish this robot's position to the ans_position topic.
 */
void RobotController::AnswerPosition() {
  se306p1::Position msg;

  msg.R_ID = this->robot_id_;
  msg.x = this->pose_.position_.x_;
  msg.y = this->pose_.position_.y_;
  msg.theta = this->pose_.theta_;

  this->ansPosPublisher_.publish(msg);
}

/**
 * Tell stage to drive the robot at the current linear and angular velocity.
 */
void RobotController::PublishVelocity() {
  geometry_msgs::Twist msg;
  msg.angular.z = this->av_;
  msg.linear.x = this->lv_;
  this->twist_.publish(msg);
}

/**
 * Execute the next tick of movement when attempting to reach a goal position
 * while executing a Go command. When executing a Go command, the robot will
 * first rotate to point at it's destination. Next, the robot will begin to
 * move towards it's destination. Once it has reached the destination, the
 * robot will align to the specified angle.
 */
void RobotController::MoveTowardsGoal() {
// If the robot is already at the position, we can skip aiming at it and
// moving to it.
  if ((this->goal_.position_ - this->pose_.position_).Length() < 0.01) {
    this->goStep_ = GoStep::ALIGNING;
  }

// Aim at the goal position.
  if (this->goStep_ == GoStep::AIMING) {
    if (this->goal_.theta_ == 999) {
      this->goStep_ = GoStep::MOVING;
    } else {
      // Figure out the angle that the robot will need to be at to be facing
      // the goal position.
      double angle_to_goal = AngleBetweenPoints(this->pose_.position_,
                                                this->goal_.position_);

      // Figure out how far away from that angle the robot currently is.
      double diff = AngleDiff(this->pose_.theta_, angle_to_goal);

      // If the difference is small, we have finished aiming.
      if (-0.001 < diff && diff < 0.001) {
//        ROS_INFO(
//            "R%" PRIu64 " aimed | x=%f, y=%f, gx =%f, gy=%f, theta=%f, gtheta=%f, lv=%f, av=%f", this->robot_id_, this->pose_.position_.x_, this->pose_.position_.y_, this->goal_.position_.x_, this->goal_.position_.y_, (this->pose_.theta_), (this->goal_.theta_), this->lv_, this->av_);

        this->goStep_ = GoStep::MOVING;
      } else {
        // We are not moving forward, so 0 lv. Setting av to the diff each tick
        // ensures we don't overshoot.
        this->lv_ = 0.0;
        this->av_ = DegreesToRadians(diff);
      }
    }
  }

// Move towards the goal position
  if (this->goStep_ == GoStep::MOVING) {
    // Figure out the distance from the goal the robot currently is.
    double distance_to_goal = (this->goal_.position_ - this->pose_.position_)
        .Length();

    // If the distance is small, we have reached the goal position.
    if (distance_to_goal < 0.1) {
//      ROS_INFO(
//          "R%" PRIu64 " moved | x=%f, y=%f, gx =%f, gy=%f, theta=%f, gtheta=%f, lv=%f, av=%f", this->robot_id_, this->pose_.position_.x_, this->pose_.position_.y_, this->goal_.position_.x_, this->goal_.position_.y_, (this->pose_.theta_), (this->goal_.theta_), this->lv_, this->av_);

      this->goStep_ = GoStep::ALIGNING;
    } else {
      // The robot is not rotating, so 0 av. The lv is constant.
      this->av_ = 0.0;
      this->lv_ = DEFAULT_LV;
    }
  }

// Rotate into the given angle.
  if (this->goStep_ == GoStep::ALIGNING) {
    if (this->goal_.theta_ == 999) {
      this->lv_ = 0.0;
      this->av_ = 0.0;

      // Report back to the supervisor.
      this->AnswerPosition();

      this->state_ = RobotState::READY;
    } else {

      // Figure out how far from the angle the robot currently is.
      double diff = AngleDiff(this->pose_.theta_, this->goal_.theta_);

      // If the difference is small, the robot is aligned and the Go is
      // finished.
      if (-0.001 < diff && diff < 0.001) {
//        ROS_INFO(
//            "R%" PRIu64 " alned | x=%f, y=%f, gx =%f, gy=%f, theta=%f, gtheta=%f, lv=%f, av=%f", this->robot_id_, this->pose_.position_.x_, this->pose_.position_.y_, this->goal_.position_.x_, this->goal_.position_.y_, (this->pose_.theta_), (this->goal_.theta_), this->lv_, this->av_);
//        ROS_INFO("R%" PRIu64 " FINISHED GO", this->robot_id_);

        // Stop the robot moving.
        this->lv_ = 0.0;
        this->av_ = 0.0;

        // Report back to the supervisor.
        this->AnswerPosition();

        this->state_ = RobotState::READY;
      } else {
        // We are not moving forward, so 0 lv. Setting av to the diff each tick
        // ensures we don't overshoot.
        this->lv_ = 0.0;
        this->av_ = DegreesToRadians(diff);
      }
    }
  }

// Update the lv and the av on Stage.
  this->PublishVelocity();
}

/**
 * Perform a scan for items. The scan will continue for SCAN_TIME. If nothing is
 * found, the scan will restart from the beginning.
 */
void RobotController::Scan() {
  if (this->scanStep_ == ScanStep::INIT) {
    this->scanningStart_ = ros::Time::now().toSec();
    this->scanStep_ = ScanStep::SCANNING;
  }

  if (this->scanStep_ == ScanStep::SCANNING) {
    if (ros::Time::now().toSec() - this->scanningStart_ >= SCAN_TIME) {
      this->scanStep_ = ScanStep::FINISHED;
    }
  }

  if (this->scanStep_ == ScanStep::FINISHED) {
    if (this->scanResult_ == 0) {
      ROS_WARN("R%" PRIu64 " Scanned nothing", this->robot_id_);
      this->scanStep_ = ScanStep::INIT;
    } else {
      ROS_INFO("R%" PRIu64 " Scanned %d", this->robot_id_, this->scanResult_);

      se306p1::ScanResult msg;
      msg.scanResult = this->scanResult_;
      this->scanResultPublisher_.publish(msg);

      // Report back to the supervisor.
      this->AnswerPosition();
      this->state_ = RobotState::READY;
    }
  }
}

/**
 * Change the state of the robot to scanning, and initialise the scanning state
 * to init.
 *
 * @param msg
 */
void RobotController::SetScanning(se306p1::Scan msg) {
  this->state_ = RobotState::SCANNING;
  this->scanStep_ = ScanStep::INIT;
}

/**
 * Set the robot to Go as per to Go message.
 *
 * @param msg The message containing the x, y, and theta values the robot
 * should try and reach.
 */
void RobotController::SetGoing(Go msg) {
// Set the robot state to going.
  this->state_ = RobotState::GOING;

// The go starts by pointing at the destination.
  this->goStep_ = GoStep::AIMING;

  Pose p;

  p.position_.x_ = msg.x;
  p.position_.y_ = msg.y;
  p.theta_ = msg.theta;

  this->goal_ = p;
}

/**
 * Set the robot to Do as per the Do message. If the Do msg contains lv and av
 * 0, the robot state is set to ready.
 *
 * @param msg The message containing the linear and angular velocity values
 * that the robot should continuously move at.
 */
void RobotController::SetDoing(Do msg) {
  if (msg.lv == 0.0 && msg.av == 0.0) {
    this->state_ = RobotState::READY;
  } else {
    this->state_ = RobotState::DOING;
  }

  this->lv_ = msg.lv;
  this->av_ = msg.av;

//  ROS_INFO(
//      "R%" PRIu64 " STARTING DO: lv: %f av: %f", this->robot_id_, msg.lv, msg.av);

  this->PublishVelocity();
}

void RobotController::ReceiveCommand(Command cmd) {
  if (cmd.enqueue) {
    this->commands_.push_back(cmd);
  } else {
    this->InterruptCommandQueue(cmd);
  }
}

/**
 * Execute a Do, Go or Scan command.
 *
 * @param cmd The Do, Go or Scan command to be executed.
 */
void RobotController::ExecuteCommand(Command cmd) {
  if (cmd.type == CommandType::DO) {
    ROS_INFO("R%" PRIu64 " EXECUTING A DO", this->robot_id_);
    Do msg;

    msg.lv = cmd.lv;
    msg.av = cmd.av;

    this->SetDoing(msg);
  } else if (cmd.type == CommandType::GO) {
    ROS_INFO("R%" PRIu64 " EXECUTING A GO", this->robot_id_);
    Go msg;

    msg.x = cmd.x;
    msg.y = cmd.y;
    msg.theta = cmd.theta;

    this->SetGoing(msg);
  } else if (cmd.type == CommandType::SCAN) {
    ROS_INFO("R%" PRIu64 " EXECUTING A SCAN", this->robot_id_);
    se306p1::Scan msg;
    this->SetScanning(msg);
  }
}

/**
 * Take the next command from the front of the command queue and execute it.
 * If there are no more commands to execute, execute a Do with 0 linear and
 * angular velocity.
 */
void RobotController::DequeueCommand() {
  if (this->commands_.empty()) {  // Do nothing if there are no more commands.
    Do msg;
    msg.lv = 0;
    msg.av = 0;
    SetDoing(msg);
  } else {
    Command cmd = this->commands_.front();
    this->commands_.pop_front();
    this->ExecuteCommand(cmd);
  }
}

/**
 * Clears the current command queue and executes the given command.
 *
 * @param cmd The command to interrupt the command queue with.
 */
void RobotController::InterruptCommandQueue(Command cmd) {
// Clear the current command queue.
  this->commands_.clear();

// Stop Going and Doing.
  this->state_ = RobotState::READY;

// Execute the interrupting command.
  this->ExecuteCommand(cmd);
}

/**
 * Begins running the ros loop.
 */
void RobotController::Run() {
  ros::spin();
}
}

#ifdef ROBOT_CONTROLLER_MAIN
int main(int argc, char *argv[]) {
  ros::init(argc, argv, "rotate_supervisor", ros::init_options::AnonymousName);

  ros::NodeHandle nh("~");

  int r_id;
  nh.getParam("rid", r_id);

  ROS_INFO("Creating robot %s with id %d", argv[0], r_id);

  se306p1::RobotController rc(nh, r_id);
  rc.Run();

  return 0;
}
#endif
