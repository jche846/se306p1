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

  // DEGREES TO RADIANS

  /**
   * Tests that a positive double converts from degrees to radians
   */
  TEST_F(TrigTest, testPosDegreesToRadians) {
    double result = M_PI / 16;
    ASSERT_NEAR(result, DegreesToRadians(11.25), 0.0001);
  }

  /**
   * Tests that a negative double converts from degrees to radians
   */
  TEST_F(TrigTest, testNegDegreesToRadians) {
    double result = -(M_PI / 16);
    ASSERT_NEAR(result, DegreesToRadians(-11.25), 0.0001);
  }

  /**
   * Tests that a zero converts from degrees to radians
   */
  TEST_F(TrigTest, testZeroDegreesToRadians) {
    ASSERT_EQ(0, DegreesToRadians(0));
  }

  // RADIANS TO DEGREES

  /**
   * Tests that a a positive double converts from radians to degrees
   */
  TEST_F(TrigTest, testPosRadiansToDegrees) {
    double input = M_PI / 16;
    double result = 11.25;
    ASSERT_NEAR(result, RadiansToDegrees(input), 0.0001);
  }

  /**
   * Tests that a negative double converts from radians to degrees
   */
  TEST_F(TrigTest, testNegRadiansToDegrees) {
    double input = -(M_PI / 16);
    double result = -11.25;
    ASSERT_NEAR(result, RadiansToDegrees(input), 0.0001);
  }



  /**
   * Tests that a zero converts from radians to degrees
   */
  TEST_F(TrigTest, testZeroRadiansToDegrees) {
    ASSERT_EQ(0, RadiansToDegrees(0));
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
} //namespace

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
