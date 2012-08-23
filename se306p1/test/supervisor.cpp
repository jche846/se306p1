#include <gtest/gtest.h>

#define private public
#include"../src/supervisor/supervisor.h"
#include"../src/supervisor/robot.h"
#undef private

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
  * Check that for a selection of 5 robots the cluster head is selected correctly.
  */
  
  TEST_F(SupervisorTest, testElectHeadRandomPositions) {
    // we need to create a set of robots and check
    // that they get assigned to the correct supervisor fields
    //    i.e. to clusterHead and to nonClusterHeads
    for (int ID = 0 ; ID < 5 ; ID++ ) {
      super.robots_[ID] = std::shared_ptr<Robot>(new Robot(ID));
    }

    super.robots_[0]->pose_ = Pose(Vector2(2.0, 2.0),-45.0);
    super.robots_[1]->pose_ = Pose(Vector2(10.0, 8.0),35.0);
    super.robots_[2]->pose_ = Pose(Vector2(-2.0, 5.0),10.0);
    super.robots_[3]->pose_ = Pose(Vector2(-2.0, -5.0),135.0);
    super.robots_[4]->pose_ = Pose(Vector2(-6.0, 3.0),-160.0);

    //call elect head
    super.ElectHead();
    
    //check that the cluster head is the cluster head near the origin
    ASSERT_EQ(super.clusterHead_->id_,0);
    // Add asserts to check taht the nonHeads contain the correct robots
        for (int ID = 1 ; ID < 5 ; ID++ ) {
      std::map<uint64_t, std::shared_ptr<Robot>>::iterator it = super.robots_.find(2);
      if(it != super.robots_.end())
      {
        //if the robot is in the map create a pass
        ASSERT_EQ(1,1);
      } else {
        //create a failure
        ASSERT_EQ(0,1);
      }
    }
  }
  
  /**
  * Check that a robot created at the origin is not made the cluster head.
  */
  TEST_F(SupervisorTest, testElectHeadWithARobotAtOrigin) {
    // we need to create a set of robots and check
    // that they get assigned to the correct supervisor fields
    //    i.e. to clusterHead and to nonClusterHeads
    for (int ID = 0 ; ID < 5 ; ID++ ) {
      super.robots_[ID] = std::shared_ptr<Robot>(new Robot(ID));
    }
    super.robots_[0]->pose_ = Pose(Vector2(0.0, 0.0),35.0);
    super.robots_[1]->pose_ = Pose(Vector2(2.0, 2.0),-45.0);
    super.robots_[2]->pose_ = Pose(Vector2(-2.0, 5.0),10.0);
    super.robots_[3]->pose_ = Pose(Vector2(-2.0, -5.0),135.0);
    super.robots_[4]->pose_ = Pose(Vector2(-6.0, 3.0),-160.0);

    //call elect head
    super.ElectHead();
    
    //check that the cluster head is not the robot that started at the origin.
    ASSERT_NE(super.clusterHead_->id_,0);
    ASSERT_EQ(super.clusterHead_->id_,1);
    // Add asserts to check taht the nonHeads contain the correct robots
    for (int ID = 2 ; ID < 5 ; ID++ ) {
      std::map<uint64_t, std::shared_ptr<Robot>>::iterator it = super.robots_.find(2);
      if(it != super.robots_.end())
      {
        //if the robot is in the map create a pass
        ASSERT_EQ(1,1);
      } else {
        //create a failure
        ASSERT_EQ(0,1);
      }
    }
  }
  
  /**
  * Check that a for two robots that spawn close together, the corect head is chosen.
  */
  TEST_F(SupervisorTest, testElectHeadCloseHeads) {
    // we need to create a set of robots and check
    // that they get assigned to the correct supervisor fields
    //    i.e. to clusterHead and to nonClusterHeads
    for (int ID = 0 ; ID < 5 ; ID++ ) {
      super.robots_[ID] = std::shared_ptr<Robot>(new Robot(ID));
    }
    super.robots_[0]->pose_ = Pose(Vector2(1.8, 1.8),35.0);
    super.robots_[1]->pose_ = Pose(Vector2(2.0, 2.0),-45.0);
    super.robots_[2]->pose_ = Pose(Vector2(-2.0, 5.0),10.0);
    super.robots_[3]->pose_ = Pose(Vector2(-2.0, -5.0),135.0);
    super.robots_[4]->pose_ = Pose(Vector2(-6.0, 3.0),-160.0);

    //call elect head
    super.ElectHead();
    
    //check that the cluster head is not the robot that started at the origin.
    ASSERT_EQ(super.clusterHead_->id_,0);
    // Add asserts to check taht the nonHeads contain the correct robots
    for (int ID = 1 ; ID < 5 ; ID++ ) {
      std::map<uint64_t, std::shared_ptr<Robot>>::iterator it = super.robots_.find(2);
      if(it != super.robots_.end())
      {
        //if the robot is in the map create a pass
        ASSERT_EQ(1,1);
      } else {
        //create a failure
        ASSERT_EQ(0,1);
      }
    }
  }

  TEST_F(SupervisorTest, FindRobotDests) {
    // we need to create a set of robots and check
    // that they get assigned to the correct supervisor fields
    //    i.e. to clusterHead and to nonClusterHeads
    for (int ID = 0 ; ID < 5 ; ID++ ) {
      super.robots_[ID] = std::shared_ptr<Robot>(new Robot(ID));
    }
    super.robots_[0]->pose_ = Pose(Vector2(2.0, 2.0),35.0);
    super.robots_[1]->pose_ = Pose(Vector2(2.0, 3.0),-45.0);
    super.robots_[2]->pose_ = Pose(Vector2(-2.0, 5.0),10.0);
    super.robots_[3]->pose_ = Pose(Vector2(-2.0, -5.0),135.0);
    super.robots_[4]->pose_ = Pose(Vector2(-6.0, 3.0),-160.0);

    //call elect head
    super.ElectHead();
    //Call FindRobotDests
    std::vector<Pose> dests = super.FindRobotDests();
    std::vector<Pose> check;
    check.push_back(Pose(Vector2(2.0 + 1.48492424, 2.0 + 1.48492424),135.0));
    check.push_back(Pose(Vector2(2.0 + 1.48492424 * 2.0, 2.0 + 1.48492424 * 2.0),135.0));
    check.push_back(Pose(Vector2(2.0 + 1.48492424 * 3.0, 2.0 + 1.48492424 * 3.0),135.0));
    check.push_back(Pose(Vector2(2.0 + 1.48492424 * 4.0, 2.0 + 1.48492424 * 4.0),135.0));
    for(int ID = 0; ID<4; ID++){
      ASSERT_NEAR(dests.at(ID).position_.x_,check.at(ID).position_.x_,0.0001);
      ASSERT_NEAR(dests.at(ID).position_.y_,check.at(ID).position_.y_,0.0001);
      ASSERT_NEAR(dests.at(ID).theta_,check.at(ID).theta_,0.0001);
    }
  }
}//namespace
int main(int argc, char *argv[]) {
  ros::init(argc, argv, "test_supervisor");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
