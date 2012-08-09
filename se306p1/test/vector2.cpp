#include <gtest/gtest.h>
#include "../src/util/vector2.h"

// The fixture for testing class Command.
using namespace se306p1;
namespace {
class CommandVector2 : public testing::Test {
   public:
    CommandTest() {

    }

    virtual ~CommandTest() {

    }

    virtual void SetUp() {

    }

    virtual void TearDown() {
    }
  };

  TEST_F(CommandVector2, testDefaultConstruct) {
    double x = 0.0
    double y = 0.0;
    ASSERT_EQ(x,vec.x_);
    ASSERT_EQ(x,vec.y_);
  }

  TEST_F(CommandVector2, testVectorValues) {
    double x = 3.0;
    double y = 4.0;
    vec = Vector2 (x,y);
    ASSERT_EQ(x,vec.x_);
    ASSERT_EQ(y,vec.y_);c
  }

  TEST_F(CommandVector2, testLength) {
    double x = 3.0;
    double y = 4.0;
    vec = Vector2 (x,y);
    ASSERT_EQ(5.0,vec.Length());
  }
  TEST_F(CommandVector2, testLengthSquared) {
    double x = 3.0;
    double y = 4.0;
    vec = Vector2 (x,y);
    ASSERT_EQ(25.0,vec.LengthSquared());
  }
  
}//namespace

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
