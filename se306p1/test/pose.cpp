#include <gtest/gtest.h>
#include "../src/util/pose.h"
#include "../src/util/vector2.h"
// The fixture for testing class Command.
using namespace se306p1;
namespace {
class CommandPose : public testing::Test {
   public:
    CommandPose() {

    }

    virtual ~CommandPose() {

    }

    virtual void SetUp() {

    }

    virtual void TearDown() {
    }
  };

  TEST_F(CommandPose, testDefaultConstruct) {
        Pose pose; //this line is obviously way too self concious due to the amount of posing it does
        Vector2 position = Vector2(0.0,0.0);
        double theta = 0.0;
        ASSERT_EQ(position,pose.position_);
        ASSERT_EQ(theta,pose.theta_);
  }

  TEST_F(CommandPose, testPoseValues) {
        Vector2 position = Vector2(2.0,3.0);
        double theta = 1.0;
        Pose pose = Pose (position,theta);
        ASSERT_EQ(position,pose.position_);
        ASSERT_EQ(theta,pose.theta_);
  }
}//namespace

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
