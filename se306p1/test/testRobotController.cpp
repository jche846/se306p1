#include <gtest/gtest.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <se306p1/Do.h>
#include <se306p1/Go.h>
#include "../src/util/command.h"
#include "../src/controller/robot_controller.h"
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

  TEST_F(RobotControllerTest, test_do_callback_enqueue) {
    Do msg_do;
    msg_do.lv = 10;
    msg_do.av = 30;
    msg_do.enqueue = true;
    rc.do_callback(msg_do);
    Command cmd = rc.commands_.back();
    ASSERT_EQ(msg_do.lv, cmd.lv);
    ASSERT_EQ(msg_do.av, cmd.av);
  }
  
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
  }

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
    ASSERT_TRUE(commands_.size()>1);
    
    msg_go3.x = 2;
    msg_go3.y = 2;
    msg_go3.theta = 4;
    msg_go3.enqueue = false;
    Command interrupt = Command(msg_go3);
    rc.InterruptCommandQueue(interrupt);
    ASSERT_TRUE(commands_.size()==0);
    ASSERT_TRUE(rc.state_==RobotState::IDLE);
  }

  TEST_F(RobotControllerTest, test_do_callback_no_enqueue) {
    Do msg_do;
    msg_do.lv = 10;
    msg_do.av = 30;
    msg_do.enqueue = false;
    rc.do_callback(msg_do);
    Command cmd = rc.commands_.back();
    ASSERT_EQ(msg_do.lv, cmd.lv);
    ASSERT_EQ(msg_do.av, cmd.av);
  }
}//namespace

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
