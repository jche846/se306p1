#include <gtest/gtest.h>
#include"../src/supervisor/supervisor.h"
#include "../src/util/command.h"
#include "../src/util/vector2.h"
#include "../src/util/pose.h"

//Begin to implement tests in supervisor
using namespace se306p1;
namespace {
class SupervisorTest : public testing::Test {
   
   public:
    ros::NodeHandle nh;
    Supervisor super;
    std::shared_ptr<Robot> r0;
    std::shared_ptr<Robot> r1;
    std::shared_ptr<Robot> r2;
    std::shared_ptr<Robot> r3;
    std::shared_ptr<Robot> r4;
    
    SupervisorTest() : nh("~"), super(nh) {
      
    }

    virtual ~SupervisorTest() {
    }

    virtual void SetUp() {

    }

    virtual void TearDown() {
      
    }
  };

  /**
  * 
  */
  /*
  TEST_F(SupervisorTest, testSwitchBehaviour) {
    uint64_t id = 1;
    SwitchBehavior(id);
    super.currentBehavior_;
  }
  */
  
  /**
  * Check that for a selection of 5 robots the cluster head is selected correctly.
  */
  /*
  TEST_F(SupervisorTest, testElectHeadRandomPositions) {
    // we need to create a set of robots and check
    // that they get assigned to the correct supervisor fields
    //    i.e. to clusterHead and to nonClusterHeads
    r0 = std::shared_ptr<Robot>(new Robot(0));
    r1 = std::shared_ptr<Robot>(new Robot(1));
    r2 = std::shared_ptr<Robot>(new Robot(2));
    r3 = std::shared_ptr<Robot>(new Robot(3));
    r4 = std::shared_ptr<Robot>(new Robot(4));
    r0->pose_ = Pose(Vector2(10.0, 8.0),35.0);
    r1->pose_ = Pose(Vector2(2.0, 2.0),-45.0);
    r2->pose_ = Pose(Vector2(-2.0, 5.0),10.0);
    r3->pose_ = Pose(Vector2(-2.0, -5.0),135.0);
    r4->pose_ = Pose(Vector2(-6.0, 3.0),-160.0);

    // add robots to a map.
    super.robots_[0] = r0;
    super.robots_[1] = r1;
    super.robots_[2] = r2;
    super.robots_[3] = r3;
    super.robots_[4] = r4;
    
    //call elect head
    super.ElectHead();
    
    //check that the clust head is elected correctly.
    ASSERT_EQ(super.clusterHead_->id_,1);
    
    // Add asserts to check taht the nonHeads contain the correct robots
  }
  */
  /**
  * Check that a robot created at the origin is not made the cluster head.
  */
  TEST_F(SupervisorTest, testElectHeadWithARobotAtOrigin) {
    // we need to create a set of robots and check
    // that they get assigned to the correct supervisor fields
    //    i.e. to clusterHead and to nonClusterHeads
    
    r0 = std::shared_ptr<Robot>(new Robot(0));
    r1 = std::shared_ptr<Robot>(new Robot(1));
    r2 = std::shared_ptr<Robot>(new Robot(2));
    r3 = std::shared_ptr<Robot>(new Robot(3));
    r4 = std::shared_ptr<Robot>(new Robot(4));
    
    r0.pose_ = Pose(Vector2(0.0, 0.0),35.0);
    r1.pose_ = Pose(Vector2(2.0, 2.0),-45.0);
    r2.pose_ = Pose(Vector2(-2.0, 5.0),10.0);
    r3.pose_ = Pose(Vector2(-2.0, -5.0),135.0);
    r4.pose_ = Pose(Vector2(-6.0, 3.0),-160.0);

    // add robots to a map.
    super.robots_[0] = r0;
    super.robots_[1] = r1;
    super.robots_[2] = r2;
    super.robots_[3] = r3;
    super.robots_[4] = r4;
    
    //call elect head
    super.ElectHead();
    
    //check that the cluster head is not the robot that started at the origin.
    ASSERT_NE(super.clusterHead_->id_,0);
    
    
    ASSERT_EQ(super.clusterHead_->id_,1);
    // Add asserts to check taht the nonHeads contain the correct robots
    
  }
}//namespace
int main(int argc, char *argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
