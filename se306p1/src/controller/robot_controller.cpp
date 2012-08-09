/*
 * RobotController.cpp
 *
 *  Created on: Aug 4, 2012
 *      Author: ahug048
 */

#include "robot_controller.h"
#include "../util/trig.h"
#include <string>

#define FREQUENCY 100 // The number of ticks per second the robot will execute.
#define DEFAULT_LV 2.0
#define DEFAULT_AV 2.0

namespace se306p1 {
  RobotController::RobotController(ros::NodeHandle &nh, uint64_t id = 0) {
    // Node handler ti talk to ROS.
    this->nh_ = nh;

    // Initialise the robot as stationary with a given ID.
    this->robot_id_ = id;
    this->lv_ = 0;
    this->av_ = 0;

    // Initialise as doing nothing.
    this->state_ = RobotState::IDLE;

    // We need to listen for the supervisor asking for our position.
    this->askPosSubscriber_ = nh_.subscribe<AskPosition>(
        ASK_POS_TOPIC, 1000, &RobotController::askPosition_callback, this,
        ros::TransportHints().reliable());

    // Publish our position when asked by the supervisor.
    this->ansPosPublisher_ = nh_.advertise<Position>(ANS_POS_TOPIC, 1000);

    // Subscribe to pose messages from Stage
    std::stringstream odomss;
    odomss << "/robot_" << this->robot_id_ << "/base_pose_ground_truth";
    this->odom_ = nh_.subscribe<nav_msgs::Odometry>(
        odomss.str(), 1000, &RobotController::odom_callback, this,
        ros::TransportHints().reliable());

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
   * Called when the supervisor publishes a Go message for this robot. If the
   * message is to be enqueued, the message is put on the queue. Otherwise, the
   * command queue is interrupted and thrown away and the Go message is executed
   * immediately.
   *
   * @param msg A Go message containing a location for the robot to move to.
   */
  void RobotController::go_callback(Go msg) {
    if (msg.enqueue) {
      this->commands_.push_back(Command(msg));
    } else {
      this->InterruptCommandQueue(Command(msg));
    }
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
    if (msg.enqueue) {
      this->commands_.push_back(Command(msg));
    } else {
      this->InterruptCommandQueue(Command(msg));
    }
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
   * Called when stage publishes an odometry message. Updates the controller's
   * knowledge of the robot's current position.
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

    // Diagnostic information
//    ROS_INFO("Current x position is: %f", msg.pose.pose.position.x);
//    ROS_INFO("Current y position is: %f", msg.pose.pose.position.y);
//    ROS_INFO("Current theta is: %f", this->pose_.theta_);
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
  void RobotController::Move() {
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
   * robot will rotate to the specified angle.
   */
  void RobotController::MoveTowardsGoal() {
    if (this->gostep_ == GoStep::AIMING) {
      this->av_ = DEFAULT_AV;
      this->lv_ = 0.0;

      double angle_to_goal = AngleBetweenPoints(this->pose_.position_,
                                                this->goal_.position_);
      double diff = AbsAngleDiff(this->pose_.theta_, angle_to_goal);

      if (DegreesToRadians(diff) < this->av_) {
        this->av_ = DegreesToRadians(diff);
      }

//      ROS_INFO(
//          "Robot %ld aiming: av=%f theta=%f goaltheta=%f a2g=%f diff=%f", this->robot_id_, this->av_, DegreesToRadians(this->pose_.theta_), DegreesToRadians(this->goal_.theta_), angle_to_goal, diff);

      if (359.999 < diff || diff < 0.001) {
        this->gostep_ = GoStep::MOVING;
      } else {
        this->Move();
      }
    } else if (this->gostep_ == GoStep::MOVING) {
      this->av_ = 0.0;
      this->lv_ = DEFAULT_LV;

      double distance_to_goal = (this->goal_.position_ - this->pose_.position_)
          .Length();

      if (distance_to_goal < lv_) {
        this->lv_ = distance_to_goal;
      }

      ROS_INFO(
          "Robot %ld: dist=%f lv=%f x=%f y=%f", this->robot_id_, distance_to_goal, lv_, pose_.position_.x_, pose_.position_.y_);

      if (distance_to_goal < 0.01) {
        this->gostep_ = GoStep::ROTATING;
      } else {
        this->Move();
      }
    } else if (this->gostep_ == GoStep::ROTATING) {
      this->av_ = DEFAULT_AV;
      this->lv_ = 0.0;

      double diff = AbsAngleDiff(this->pose_.theta_, this->goal_.theta_);

      if (DegreesToRadians(diff) < this->av_) {
        this->av_ = DegreesToRadians(diff);
      }

//      ROS_INFO(
//          "Robot %ld : av=%f theta=%f goaltheta=%f diff=%f", this->robot_id_, this->av_, this->pose_.theta_, this->goal_.theta_, diff);

      if (359.999 < diff || diff < 0.001) {
        this->state_ = RobotState::FINISHED;
      } else {
        this->Move();
      }
    }
  }

  /**
   * Set the robot to Go as per to Go message. Resets the robot lv and av to
   * their default values.
   *
   * @param msg The message containing the x, y, and theta values the robot
   * should try and reach.
   */
  void RobotController::SetGoing(Go msg) {
    // Set the robot state to going.
    this->state_ = RobotState::GOING;

    // The go starts by pointing at the destination.
    this->gostep_ = GoStep::AIMING;

    Pose p;

    p.position_.x_ = msg.x;
    p.position_.y_ = msg.y;
    p.theta_ = msg.theta;

    this->goal_ = p;
  }

  /**
   * Set the robot to Do as per the Do message. If the Do msg contains lv and av
   * 0, the robot state is set to idle.
   *
   * @param msg The message containing the linear and angular velocity values
   * that the robot should continuously move at.
   */
  void RobotController::SetDoing(Do msg) {
    if (msg.lv == 0.0 && msg.av == 0.0) {
      this->state_ = RobotState::IDLE;
    } else {
      this->state_ = RobotState::DOING;
    }

    this->lv_ = msg.lv;
    this->av_ = msg.av;
  }

  /**
   * Execute a Do or Go command.
   *
   * @param cmd The Do or Go command to be executed.
   */
  void RobotController::ExecuteCommand(Command cmd) {
    if (cmd.isDo) {
      Do msg;

      msg.lv = cmd.lv;
      msg.av = cmd.av;

      this->SetDoing(msg);
    } else {
      Go msg;

      msg.x = cmd.x;
      msg.y = cmd.y;
      msg.theta = cmd.theta;

      this->SetGoing(msg);
    }
  }

  /**
   * Take the next command from the front of the command queue and execute it.
   * If there are no more commands to execute, execute a Do with 0 linear and
   * angular velocity.
   */
  void RobotController::DequeCommand() {
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
    this->state_ = RobotState::IDLE;

    // Execute the interrupting command.
    this->ExecuteCommand(cmd);
  }

  void RobotController::Run() {
    ros::Rate r(FREQUENCY);  // Run FREQUENCY times a second

    while (ros::ok()) {
      if (this->state_ == RobotState::GOING) {  // If the robot is going somewhere, keep trying to go there
        this->MoveTowardsGoal();
      } else if (this->state_ == RobotState::DOING) {  // If the robot is doing, keep doing.
        this->Move();
      } else if (this->state_ == RobotState::FINISHED) {  // Get the next command.
        this->DequeCommand();
        this->AnswerPosition();
      }

      r.sleep();
      ros::spinOnce();
    }
  }
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, argv[0]);

  ros::NodeHandle nh("~");

  int r_id;
  nh.getParam("rid", r_id);

  ROS_INFO("Creating robot %s with id %d", argv[0], r_id);

  se306p1::RobotController rc(nh, r_id);
  rc.Run();

  return 0;
}
