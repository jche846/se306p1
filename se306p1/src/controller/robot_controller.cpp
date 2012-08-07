/*
 * RobotController.cpp
 *
 *  Created on: Aug 4, 2012
 *      Author: ahug048
 */

#include "robot_controller.h"
#include "../util/trig.h"
#define FREQUENCY 20 // The number of ticks per second the robot will execute.
namespace se306p1 {
  RobotController::RobotController(ros::NodeHandle &nh, int64_t id = 0) {
    this->nh_ = nh;

    // Initialise the robot as stationary with a given ID.
    this->robot_id_ = id;
    this->lv_ = 0;
    this->av_ = 0;

    // Initialise as stationary.
    this->doing_ = false;
    this->going_ = false;

    // We need to listen for the supervisor asking for our position.
    this->askPosSubscriber_ = nh_.subscribe<AskPosition>(
        ASK_POS_TOPIC, 1000, &RobotController::askPosition_callback, this,
        ros::TransportHints().reliable());

    // Subscribe to do messages in order to know when to move.
    std::stringstream doss;
    doss << "/robot" << this->robot_id_ << "/do";
    this->doSubscriber_ = nh_.subscribe<Do>(doss.str(), 1000,
                                            &RobotController::do_callback, this,
                                            ros::TransportHints().reliable());

    // Subscribe to go messages in order to know when to move.
    std::stringstream goss;
    goss << "/robot" << this->robot_id_ << "/go";
    this->goSubscriber_ = nh_.subscribe<Go>(goss.str(), 1000,
                                            &RobotController::go_callback, this,
                                            ros::TransportHints().reliable());

    // Publish our position when asked by the supervisor.
    this->ansPosPublisher_ = nh_.advertise<Position>(ANS_POS_TOPIC, 1000);

    // Subscribe to pose messages from Stage
    std::stringstream odomss;
    odomss << "/robot_" << this->robot_id_ << "/base_pose_ground_truth";
    this->odom_ = nh_.subscribe<nav_msgs::Odometry>(
        odomss.str(), 1000, &RobotController::odom_callback, this);

    std::stringstream twistss;
    twistss << "/robot_" << this->robot_id_ << "/cmd_vel";
    this->twist_ = nh_.advertise<geometry_msgs::Twist>(twistss.str(), 1000);

    /** ROS sub/pubs from Chandan
     //advertise() function will tell ROS that you want to publish on a given topic_
     //for other robots
     ros::Publisher RobotNode_pub = n.advertise<Project2Sample::R_ID>("Robot0_msg",1000);
     //to stage
     ros::Publisher RobotNode_stage_pub = n.advertise<geometry_msgs::Twist>("/robot_0/cmd_vel",1000);

     //subscribe to listen to messages of other robots
     ros::Subscriber RobotNode1_sub = n.subscribe<Project2Sample::R_ID>("Robot1_msg",1000, RobotNode1_callback);

     //subscribe to listen to messages coming from stage
     ros::Subscriber StageLaser_sub = n.subscribe<sensor_msgs::LaserScan>("Robot0_laser",1000,StageLaser_callback);
     **/
  }

  RobotController::~RobotController() {
    // TODO Auto-generated destructor stub
  }

  /**
   * Work out the angle in degrees counter clockwise from North that the robot
   * needs to turn in order to point at it's goal coordinates.
   *
   * @return Degrees representing the angle between the robot's current theta
   * and the goal coordinates.
   */
  double RobotController::AngleToGoal() {

    double dx = this->goal_.position_.x_ - this->position_.position_.x_;
    double dy = this->goal_.position_.y_ - this->position_.position_.y_;
    double phi;

    double a_tan = DegATan(dy / dx);

    if (dx >= 0 && dy >= 0) {
      phi = a_tan;
    } else if (dx < 0 && dy >= 0) {
      phi = 180.0 - a_tan;
    } else if (dx < 0 && dy < 0) {
      phi = 180 + a_tan;
    } else {
      phi = 360 - a_tan;
    }

    return phi;
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
   * Publish this robot's position to the ans_position topic.
   */
  void RobotController::AnswerPosition() {
    se306p1::Position msg;

    msg.R_ID = this->robot_id_;
    msg.x = this->position_.position_.x_;
    msg.y = this->position_.position_.y_;
    msg.theta = this->position_.theta_;

    this->ansPosPublisher_.publish(msg);
  }

  /**
   * Called when stage publishes an odometry message.
   *
   * @param msg The odometry message containing position data for this robot.
   */
  void RobotController::odom_callback(nav_msgs::Odometry msg) {

    double roll;
    double pitch;
    double yaw;

    // Set the position in 2D space
    this->position_.position_.x_ = msg.pose.pose.position.x;
    this->position_.position_.y_ = msg.pose.pose.position.y;

    // Set the rotation
    QuaternionMsgToRPY(msg.pose.pose.orientation, roll, pitch, yaw);
    this->position_.theta_ = yaw;

    //  ROS_INFO("Current x position is: %f", msg.pose.pose.position.x);
    //  ROS_INFO("Current y position is: %f", msg.pose.pose.position.y);
    ROS_INFO("Current theta is: %f", this->position_.theta_);
  }

  /**
   * Tell stage to drive the robot at the given linear and angular velocity.
   *
   * @param lv Linear velocity
   * @param av Angular velocity
   */
  void RobotController::Twist(double lv = 0, double av = 0) {
    geometry_msgs::Twist msg;
    msg.angular.z = av;
    msg.linear.x = lv;
    this->twist_.publish(msg);
  }

  /**
   * Set the robot to Go as per to Go message.
   *
   * @param msg The message containing the x, y, and theta values the robot
   * should try and reach.
   */
  void RobotController::SetGoing(Go msg) {
    this->doing_ = false;
    this->going_ = true;

    Pose p;

    p.position_.x_ = msg.x;
    p.position_.y_ = msg.y;
    p.theta_ = msg.theta;

    this->goal_ = p;
  }

  /**
   * Set the robot to Do as per the Do message.
   *
   * @param msg The message containing the linear and angular velocity values
   * that the robot should continuously move at.
   */
  void RobotController::SetDoing(Do msg) {
    this->going_ = false;
    this->doing_ = true;

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
    if (this->commands_.empty()) {
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
    this->ExecuteCommand(cmd);
  }

  void RobotController::Run() {
    ros::Rate r(FREQUENCY);  // Run FREQUENCY times a second
    bool rotating = true;
    this->goal_.position_.x_ = 2.0;
    this->goal_.position_.y_ = 2.0;
    this->goal_.theta_ = 45.0;
    this->lv_ = 1.0;
    this->av_ = 1.0;

    while (ros::ok()) {
      // If the robot is not at it's goal position, go to the position.
      if (this->goal_ != this->position_) {
        if (rotating) {
          double angle_to_goal = this->AngleToGoal();

          if (angle_to_goal < this->av_) {
            this->av_ = angle_to_goal;
          }

          this->Twist(0.0, this->av_);

          if (this->position_.theta_ == this->goal_.theta_) {
            rotating = false;
          }
        }

        if (!rotating) {
          double distance_to_goal = (this->goal_.position_
              - this->position_.position_).Length();

          if (!(distance_to_goal >= (lv_ / FREQUENCY))) {
            lv_ = distance_to_goal;
          }

          this->Twist(this->lv_, 0.0);

          if ((this->goal_.position_ - this->position_.position_).Length()
              == 0.0) {
            // do nothing
          }
        }
      } else {
        this->DequeCommand();
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
