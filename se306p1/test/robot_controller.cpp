#include <gtest/gtest.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <se306p1/Do.h>
#include <se306p1/Go.h>
#include "../src/util/command.h"
#include "../src/util/pose.h"

#define private public
#include "../src/controller/robot_controller.h"
#undef private

// The fixture for testing class Command.
using namespace se306p1;
namespace {
class RobotControllerTest : public testing::Test {
   
   public:
    
    ros::NodeHandle nh;
    RobotController rc;
    
    RobotControllerTest() : nh("~"), rc(nh,1) {
      
    }

    virtual ~RobotControllerTest() {
      
    }

    virtual void SetUp() {
      //rc = new RobotController(nh,1);
    }

    virtual void TearDown() {
      
    }
    void setGoal(double x, double y);
    void setPose(double x, double y, double theta);
  };

  /**
  * Tests that do_callback runs correctly when enqueue is true
  */
  TEST_F(RobotControllerTest, test_do_callback_enqueue) {
    Do msg_do;
    msg_do.lv = 10;
    msg_do.av = 30;
    msg_do.enqueue = true;
    rc.do_callback(msg_do);
    Command cmd = rc.commands_.back();
    ASSERT_EQ(msg_do.lv, cmd.lv);
    ASSERT_EQ(msg_do.av, cmd.av);
    ASSERT_EQ(msg_do.enqueue, cmd.enqueue);
    //proves the deque hasnt been cleared, i.e InterruptQueue hasnt been called
    ASSERT_TRUE(rc.commands_.size()>0);
  }

  /**
  * Tests that go_callback runs correctly when enqueue is true
  */
  TEST_F(RobotControllerTest, test_go_callback_enqueue) {
    Go msg_go;
    msg_go.x = 7;
    msg_go.y = 15;
    msg_go.theta = 33;
    msg_go.enqueue = true;
    rc.go_callback(msg_go);
    Command cmd = rc.commands_.back();
    ASSERT_EQ(msg_go.x, cmd.x);
    ASSERT_EQ(msg_go.y, cmd.y);
    ASSERT_EQ(msg_go.theta, cmd.theta);
    ASSERT_EQ(msg_go.enqueue, cmd.enqueue);
    //proves the deque hasnt been cleared, i.e InterruptQueue hasnt been called
    ASSERT_TRUE(rc.commands_.size()>0);
  }

  /**
  * Tests that InterruptCommandQueue is called correctly.
  */
  TEST_F(RobotControllerTest, test_InterruptCommandQueue){
    Go msg_go1;
    Go msg_go2;
    Go msg_go3;
    msg_go1.x = 7;
    msg_go1.y = 15;
    msg_go1.theta = 33;
    msg_go1.enqueue = false;
    Command cmd1 = Command(msg_go1);
    msg_go2.x = 9;
    msg_go2.y = 5;
    msg_go2.theta = 12;
    msg_go2.enqueue = false;
    Command cmd2 = Command(msg_go2);
    rc.commands_.push_back(cmd1);
    rc.commands_.push_back(cmd2);
    //check commands are added
    ASSERT_TRUE(rc.commands_.size()>1);
    msg_go3.x = 2;
    msg_go3.y = 3;
    msg_go3.theta = 4;
    msg_go3.enqueue = false;
    Command interrupt = Command(msg_go3);
    rc.InterruptCommandQueue(interrupt);
    //checks deque is cleared
    ASSERT_EQ(rc.commands_.size(),0);
    
    Vector2 position = Vector2(2.0,3.0);
    double theta = 4.0;
    Pose pose = Pose (position,theta);
    //prove SetGoing has been called
    ASSERT_TRUE(rc.goal_.theta_==theta);
    ASSERT_TRUE(rc.goal_.position_.x_==position.x_);
    ASSERT_TRUE(rc.goal_.position_.y_==position.y_);
  }

  /**
  * Tests that do_callback calls InterruptCommandQueue when enqueue is false
  */
  TEST_F(RobotControllerTest, test_do_callback_no_enqueue) {
    Do msg_do;
    msg_do.lv = 10;
    msg_do.av = 30;
    msg_do.enqueue = false;
    rc.do_callback(msg_do);
    ASSERT_TRUE(rc.commands_.size()==0);
    //checking that the commands_ deque is cleared proves that InterruptCommandQueue()
    //was called.
    ASSERT_TRUE(rc.commands_.size()==0);
  }

  /**
  * Tests that go_callback calls InterruptCommandQueue when enqueue is false
  */
  TEST_F(RobotControllerTest, test_go_callback_no_enqueue) {
    Go msg_go;
    msg_go.x = 5;
    msg_go.y = 10;
    msg_go.theta = 30;
    msg_go.enqueue = false;
    Vector2 position = Vector2(5.0,10.0);
    double theta = 30.0;
    rc.go_callback(msg_go);
    ASSERT_TRUE(rc.commands_.size()==0);
    //checking that the commands_ deque is cleared proves that InterruptCommandQueue()
    //was called.
    ASSERT_TRUE(rc.goal_.theta_==theta);
    ASSERT_TRUE(rc.goal_.position_.x_==position.x_);
    ASSERT_TRUE(rc.goal_.position_.y_==position.y_);
  }

  /**
  * Tests that setGoing runs correctly.
  */
  TEST_F(RobotControllerTest, testSetGoing){
    Go msg_go;
    msg_go.x = 5;
    msg_go.y = 10;
    msg_go.theta = 30;
    msg_go.enqueue = true;
    Vector2 position = Vector2(5.0,10.0);
    double theta = 30.0;
    rc.SetGoing(msg_go);
    //check state
    ASSERT_EQ(rc.state_,RobotState::GOING);
    //check aiming set
    ASSERT_EQ(rc.gostep_,GoStep::AIMING);

    //prove SetGoing has been called
    ASSERT_TRUE(rc.goal_.theta_==theta);
    ASSERT_TRUE(rc.goal_.position_.x_==position.x_);
    ASSERT_TRUE(rc.goal_.position_.y_==position.y_);
  }

  /**
  * Tests that setDoing runs correctly when passed none zero values
  */
  TEST_F(RobotControllerTest, testSetDoing_hasGoal){
    Do msg_do;
    msg_do.av = 5;
    msg_do.lv = 10;
    msg_do.enqueue = true;

    rc.SetDoing(msg_do);
    //check state
    ASSERT_EQ(rc.state_,RobotState::DOING);
    //check lv is set for the robot
    ASSERT_EQ(rc.lv_,msg_do.lv);
    //check av is set for the robot
    ASSERT_EQ(rc.av_,msg_do.av);
  }

  /**
  * Tests that setDoing runs correctly when passed zero values
  */
  TEST_F(RobotControllerTest, testSetDoing_noGoal){
    Do msg_do;
    msg_do.av = 0;
    msg_do.lv = 0;
    msg_do.enqueue = true;

    rc.SetDoing(msg_do);
    //check state
    ASSERT_EQ(rc.state_,RobotState::IDLE);
    //check lv is set for the robot
    ASSERT_EQ(rc.lv_,msg_do.lv);
    //check av is set for the robot
    ASSERT_EQ(rc.av_,msg_do.av);
  }

  /**
  * tests DequeueCommand when commands_ is empty
  */
  TEST_F(RobotControllerTest, testDequeueCommands_emptyDeque){
    //check the deque is clear
    rc.commands_.clear();
    rc.DequeueCommand();
    //check state
    ASSERT_EQ(rc.state_,RobotState::IDLE);
    //check lv is set for the robot has been set to 0
    ASSERT_EQ(rc.lv_,0);
    //check av is set for the robot has been set to 0
    ASSERT_EQ(rc.av_,0);
  }

  /**
  * Tests DequeueCommand when commands_ is not empty.
  */
  TEST_F(RobotControllerTest, testDequeueCommands_nonEmptyDeque){
    //check the deque is clear
    rc.commands_.clear();
    //add an element to the queue
    //create a command
    Go msg_go;
    msg_go.x = 5;
    msg_go.y = 10;
    msg_go.theta = 30;
    msg_go.enqueue = true;
    
    Command cmd = Command(msg_go);
    rc.commands_.push_back(cmd);
    rc.DequeueCommand();
    //there is one command in the deque. Running deque should remove it
    //commands_ should be empty
    ASSERT_TRUE(rc.commands_.size()==0);
    //initialise a pose with the msg_go values
    Vector2 position = Vector2(msg_go.x,msg_go.y);
    double theta = msg_go.theta;
    Pose pose = Pose (position,theta);
    //the command is executed so the robots pose should be set to msg_go's values.
    ASSERT_TRUE(rc.goal_.theta_==theta);
    ASSERT_TRUE(rc.goal_.position_.x_==position.x_);
    ASSERT_TRUE(rc.goal_.position_.y_==position.y_);
  }

  /**
  * Tests that the ExecuteCommand() method executes a Go command correctly
  */
  TEST_F(RobotControllerTest, testExecuteGoCommand){
    //create a Go command
    Go msg_go;
    msg_go.x = 5;
    msg_go.y = 10;
    msg_go.theta = 30;
    msg_go.enqueue = true;
    Command cmd = Command(msg_go);
    //create a pose
    Vector2 position = Vector2(msg_go.x,msg_go.y);
    double theta = msg_go.theta;
    Pose pose = Pose (position,theta);
    rc.ExecuteCommand(cmd);
    //the robots go should be set to the pose.
    ASSERT_TRUE(rc.goal_.theta_==theta);
    ASSERT_TRUE(rc.goal_.position_.x_==position.x_);
    ASSERT_TRUE(rc.goal_.position_.y_==position.y_);
  }

  /**
  * Tests that the ExecuteCommand() method executes a Do command correctly
  */
  TEST_F(RobotControllerTest, testExecuteDoCommand){
    //create a Do msg
    Do msg_do;
    msg_do.av = 5;
    msg_do.lv = 10;
    msg_do.enqueue = true;
    Command cmd = Command(msg_do);
    rc.ExecuteCommand(cmd);
    //The robots angular and linear velocity should be set to that of the message.
    ASSERT_EQ(rc.lv_,msg_do.lv);
    ASSERT_EQ(rc.av_,msg_do.av);
  }

  /**
  * Directly sets the fields for the position of the robot
  */
  
//  void RobotControllerTest::setPose(double x, double y, double theta){
//    rc.pose_.position_.x_ = x;
//    rc.pose_.position_.y_ = y;
//    rc.pose_.theta_ = theta;
//  }
  /**
  * Directly sets the fields for the goal of the robot
  */
  
//  void RobotControllerTest::setGoal(double x, double y){
//    rc.goal_.position_.x_ = x;
//    rc.goal_.position_.y_ = y;
//  }
  
  // The following tests may be deprecated or need to be moved to another test suite
  /**
  * Test for a goal in sector 1
  */
//  TEST_F(RobotControllerTest, testAngleToGoal_Sector1 ){
//    setPose(3,-3,0);
//    setGoal(7,1);
//   ASSERT_EQ(rc.AngleToGoal(),-45.0);
//  }
  /**
  * Test for a goal in sector 2
  */
//  TEST_F(RobotControllerTest, testAngleToGoal_Sector1 ){
//    setPose(3,-3,0);
//    setGoal(-1,1);
//    ASSERT_EQ(rc.AngleToGoal(),45.0);
//  }
  /**
  * Test for a goal in sector 3
  */
//  TEST_F(RobotControllerTest, testAngleToGoal_Sector1 ){
//    setPose(3,-3,0);
//    setGoal(-1,-7);
//    ASSERT_EQ(rc.AngleToGoal(),135.0);
//  }
  /**
  * Test for a goal in sector 4
  */
//  TEST_F(RobotControllerTest, testAngleToGoal_Sector1 ){
//    setPose(3,-3,0);
//    setGoal(7,-7);
//    ASSERT_EQ(rc.AngleToGoal(),-135.0);
//  }
}//namespace

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_robot_controller");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
