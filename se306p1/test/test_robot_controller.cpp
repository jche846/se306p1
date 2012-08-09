#include <gtest/gtest.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <se306p1/Do.h>
#include <se306p1/Go.h>
#include "../util/command.h"
// The fixture for testing class Command.
using namespace se306p1;
namespace {
class RobotControllerTest : public testing::Test {
   public:
    RobotControllerTest() {
      
    }

    virtual ~RobotControllerTest() {
      
    }

    virtual void SetUp() {
      ros::NodeHandle nh("~");
      RobotController::RobotController rc(nh,1);
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
    
  }

}//namespace

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
