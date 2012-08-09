#include <gtest/gtest.h>
#include "../src/util/pose.h"
#include "../src/util/vector2.h"
// The fixture for testing class Command.
using namespace se306p1;
namespace {
class PoseTest : public testing::Test {
   public:
    PoseTest() {

    }

    virtual ~PoseTest() {

    }

    virtual void SetUp() {

    }

    virtual void TearDown() {
    }
  };
  /**
  * Tests the default constructor
  */
  TEST_F(PoseTest, testDefaultConstruct) {
    Pose pose;
    Vector2 position = Vector2(0.0,0.0);
    double theta = 0.0;
    ASSERT_EQ(position,pose.position_);
    ASSERT_EQ(theta,pose.theta_);
  }
  /**
  * Tests when Pose is made with values
  */
  TEST_F(PoseTest, testPoseValues) {
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
