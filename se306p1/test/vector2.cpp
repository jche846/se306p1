#include <gtest/gtest.h>
#include "../src/util/vector2.h"

// The fixture for testing class Command.
using namespace se306p1;
namespace {
class Vector2Test : public testing::Test {
   public:
    Vector2Test() {

    }

    virtual ~Vector2Test() {

    }

    virtual void SetUp() {

    }

    virtual void TearDown() {
    }
  };

  TEST_F(Vector2Test, testDefaultConstruct) {
    double x = 0.0;
    double y = 0.0;
    Vector2 vec = Vector2 ();
    ASSERT_EQ(x,vec.x_);
    ASSERT_EQ(y,vec.y_);
  }

  TEST_F(Vector2Test, testVectorValues) {
    double x = 3.0;
    double y = 4.0;
    Vector2 vec = Vector2 (x,y);
    ASSERT_EQ(x,vec.x_);
    ASSERT_EQ(y,vec.y_);
  }

  TEST_F(Vector2Test, testLength) {
    double x = 3.0;
    double y = 4.0;
    Vector2 vec = Vector2 (x,y);
    ASSERT_EQ(5.0,vec.Length());
  }

  TEST_F(Vector2Test, testLengthSquared) {
    double x = 3.0;
    double y = 4.0;
    Vector2 vec = Vector2 (x,y);
    ASSERT_EQ(25.0,vec.LengthSquared());
  }
  
}//namespace

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
