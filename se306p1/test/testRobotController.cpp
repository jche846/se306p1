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
    RobotController rc;
    RobotControllerTest() {
      
    }

    virtual ~RobotControllerTest() {
      
    }

    virtual void SetUp() {
      ros::NodeHandle nh("~");
      rc = new RobotController(nh,1);
    }

    virtual void TearDown() {
      
    }
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
    //proves the deque hasnt been called, i.e InterruptQueue hasnt been called
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
    ASSERT_EQ(msg_do.lv, cmd.lv);
    ASSERT_EQ(msg_do.av, cmd.av);
    //proves the deque hasnt been called, i.e InterruptQueue hasnt been called
    ASSERT_TRUE(rc.commands_.size()>0);
  }
  /**
  * Tests that InterruptCommandQueue is called correctly.
  */
  TEST_F(RobotControllerTest, test_InterruptCommandQueue){
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
    ASSERT_TRUE(commands_.size()>1);
    
    msg_go3.x = 2;
    msg_go3.y = 3;
    msg_go3.theta = 4;
    msg_go3.enqueue = false;
    Command interrupt = Command(msg_go3);
    rc.InterruptCommandQueue(interrupt);
    //checks deque is cleared
    ASSERT_TRUE(commands_.size()==0);
    //check idle is set
    ASSERT_TRUE(rc.state_==RobotState::IDLE);
    
    Vector2 position = Vector2(2.0,3.0);
    double theta = 4.0;
    Pose pose = Pose (position,theta);
    //prove SetGoing has been called
    ASSERT_TRUE(rc.goal_==pose);
  }
  /**
  * Tests that do_callback calls InterruptCommandQueue when enqueue is false
  */
  TEST_F(RobotControllerTest, test_do_callback_no_enqueue) {
    Do msg_do;
    msg_do.lv = 10;
    msg_do.av = 30;
    msg_do.enqueue = false;
    ASSERT_TRUE(rc.commands_.size()>0);
    rc.do_callback(msg_do);
    //checking that the command_ deque is cleared proves that InterruptCommandQueue()
    //was called.
    ASSERT_TRUE(rc.commands_.size()==0);
  }
  /**
  * Tests that go_callback calls InterruptCommandQueue when enqueue is false
  */
  TEST_F(RobotControllerTest, test_go_callback_no_enqueue) {
    Do msg_do;
    msg_go.lv = 10;
    msg_go.av = 30;
    msg_go.enqueue = false;
    ASSERT_TRUE(rc.commands_.size()>0);
    rc.do_callback(msg_go);
    //checking that the command_ deque is cleared proves that InterruptCommandQueue()
    //was called.
    ASSERT_TRUE(rc.commands_.size()==0);
  }
  /**
  * Tests that setGoing runs correctly.
  */
  TEST_F(RobotControllerTest, testSetGoing){
    msg_go.x = 5;
    msg_go.y = 10;
    msg_go.theta = 30;
    msg_go.enqueue = true;
    Vector2 position = Vector2(5.0,10.0);
    double theta = 30.0;
    Pose pose = Pose (position,theta);
    rc.SetGoing(msg_go);
    //check state
    ASSERT_EQ(rc.state_,RobotState::GOING);
    //check aiming set
    ASSERT_EQ(rc.gostep_,GoStep::AIMING;);
    //check state is the correct pose.
    ASSERT_EQ(rc.goal_,pose);
  }
  /**
  * Tests that setDoing runs correctly when passed none zero values
  */
  TEST_F(RobotControllerTest, testSetDoing_hasGoal){
    msg_do.av = 5;
    msg_do.lv = 10;
    msg_do.enqueue = true;

    rc.SetDoing(msg_do);
    //check state
    ASSERT_EQ(rc.state_,RobotState::DOING);
    //check lv is set for the robot
    ASSERT_EQ(rc.lv_,msg_go.lv);
    //check av is set for the robot
    ASSERT_EQ(rc.av_,msg_go.av);
  }
  /**
  * Tests that setDoing runs correctly when passed zero values
  */
  TEST_F(RobotControllerTest, testSetDoing_noGoal){
    msg_do.av = 0;
    msg_do.lv = 0;
    msg_do.enqueue = true;

    rc.SetDoing(msg_do);
    //check state
    ASSERT_EQ(rc.state_,RobotState::IDLE);
    //check lv is set for the robot
    ASSERT_EQ(rc.lv_,msg_go.lv);
    //check av is set for the robot
    ASSERT_EQ(rc.av_,msg_go.av);
  }
  /**
  * tests DequeCommand when command_ is empty
  */
  TEST_F(RobotControllerTest, testDequeCommand_emptyDeque){
    //check the deque is clear
    rc.command_.clear();
    rc.DequeCommand();
    //check state
    ASSERT_EQ(rc.state_,RobotState::IDLE);
    //check lv is set for the robot has been set to 0
    ASSERT_EQ(rc.lv_,0);
    //check av is set for the robot has been set to 0
    ASSERT_EQ(rc.av_,0);
  }
}//namespace

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
