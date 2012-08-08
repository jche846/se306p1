#include <gtest/gtest.h>
#include "../src/util/trig.h"
#include <cmath>

using namespace se306p1;
namespace {
class TrigTest : public testing::Test {
   public:

    TrigTest() {
    }

    virtual ~TrigTest() {
    }

    virtual void SetUp() {
    }

    virtual void TearDown() {
    }
  };

  TEST_F(TrigTest, testDegreesToRadians) {
    double result = M_PI/4;
    ASSERT_NEAR(result, DegreesToRadians(45), 0.0001);
  }

  TEST_F(TrigTest, testRadiansToDegrees) {
    double input = M_PI/10;
    double result = 18;
    ASSERT_NEAR(result, RadiansToDegrees(input), 0.0001);
  }
  TEST_F(TrigTest, testDegSin) {
    ASSERT_NEAR(0.5, DegSin(30), 0.0001);
  }
  TEST_F(TrigTest, testDegCos) {
    ASSERT_NEAR(0.0, DegCos(90), 0.0001);
  }
  TEST_F(TrigTest, testDegTan) {
    ASSERT_NEAR(1, DegTan(45), 0.0001);
  }
  TEST_F(TrigTest, testDegATan) {
    ASSERT_NEAR(45, DegATan(1), 0.0001);
  }
}//namespace

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
