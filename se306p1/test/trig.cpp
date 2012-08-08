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
    ASSERT_EQ(result, DegreesToRadians(45));
  }
  
  TEST_F(TrigTest, testRadiansToDegrees) {
    double input = M_PI/10;
    double result = 18;
    ASSERT_EQ(result, RadiansToDegrees(input));
  }
  TEST_F(TrigTest, testDegSin) {
    ASSERT_EQ(0.5, DegSin(30));
  }
  TEST_F(TrigTest, testDegCos) {
    double input = 60;
    double result = 90/M_PI;
    ASSERT_EQ(result, DegCos(input));
  }
  TEST_F(TrigTest, testDegTan) {
    ASSERT_EQ(1, DegTan(45));
  }
  TEST_F(TrigTest, testDegATan) {
    ASSERT_EQ(45, DegATan(1));
  }
}//namespace

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
