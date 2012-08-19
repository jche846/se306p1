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

  // SINE

  /**
   * Tests positive sine
   */
  TEST_F(TrigTest, testPosDegSin) {
    ASSERT_NEAR(0.5, DegSin(30), 0.0001);
  }

  /**
   * Tests negative sine
   */
  TEST_F(TrigTest, testNegDegSin) {
    ASSERT_NEAR(-0.5, DegSin(-30), 0.0001);
  }

  /**
   * Tests zero sine
   */
  TEST_F(TrigTest, testZeroDegSin) {
    ASSERT_NEAR(0, DegSin(0), 0.0001);
  }

  // COSINE

  /**
   * Tests positive cosine
   */
  TEST_F(TrigTest, testPosDegCos) {
    ASSERT_NEAR(-1.0, DegCos(180), 0.0001);
  }

  /**
   * Tests negative cosine
   */
  TEST_F(TrigTest, testNegDegCos) {
    ASSERT_NEAR(-1.0, DegCos(-180), 0.0001);
  }

  /**
   * Tests zero parameter cosine
   */
  TEST_F(TrigTest, testZeroParamDegCos) {
    ASSERT_NEAR(1.0, DegCos(0), 0.0001);
  }

  /**
   * Tests zero returned cosine
   */
  TEST_F(TrigTest, testZeroRetDegCos) {
    ASSERT_NEAR(0.0, DegCos(90), 0.0001);
  }

  // TAN

  /**
   * Tests positive tan
   */
  TEST_F(TrigTest, testPosDegTan) {
    ASSERT_NEAR(1, DegTan(45), 0.0001);
  }

  /**
   * Tests positive atan
   */
  TEST_F(TrigTest, testPosDegATan) {
    ASSERT_NEAR(45, DegATan(1), 0.0001);
  }

  /**
   * Tests negative tan
   */
  TEST_F(TrigTest, testNegDegTan) {
    ASSERT_NEAR(-1, DegTan(-45), 0.0001);
  }

  /**
   * Tests negative atan
   */
  TEST_F(TrigTest, testNegDegATan) {
    double result = -45.0;
    ASSERT_NEAR(result, DegATan(-1.0), 0.0001);
  }

  /**
   * Tests zero tan
   */
  TEST_F(TrigTest, testZeroDegTan) {
    ASSERT_NEAR(0, DegTan(0), 0.0001);
  }

  /**
   * Tests zero atan
   */
  TEST_F(TrigTest, testZeroDegATan) {
    ASSERT_NEAR(0, DegATan(0), 0.0001);
  }

  //TODO ANGLE DIFF



  //TODO ANGLE BETWEEN POINTS

  //TODO NORMALIZE ANGLE

  // FIND ROBOT POSITIONS

//  /**
//   * Tests zero tan
//   */
//  TEST_F(TrigTest, testFindRobotPos) {
//
//    // Inputs
//    Vector2 center = new Vector2();
//    double theta = 90;
//        double diameter = 10.0;
//        int numRobots
//        int numSides
//
//    ASSERT_NEAR(0, FindRobotPositions(), 0.0001);
//  }

  //TODO FIND POINT FROM THETA

  /**
  * Test for a goal in top left positive
  */
  TEST_F(TrigTest, testPosAngleDiff_TL ){
    ASSERT_EQ(AngleDiff(10, 55), 45.0);
  }

  /**
  * Test for a goal in top left negative
  */
  TEST_F(TrigTest, testNegAngleDiff_TL ){
    ASSERT_EQ(AngleDiff(55, 10), -45.0);
  }

  /**
  * Test for a goal in bottom left positive
  */
  TEST_F(TrigTest, testPosAngleDiff_BL ){
    ASSERT_EQ(AngleDiff(90,135),45.0);
  }

  /**
  * Test for a goal in bottom left negative
  */
  TEST_F(TrigTest, testNegAngleDiff_BL ){
    ASSERT_EQ(AngleDiff(135,90),-45.0);
  }

  /**
  * Test for a goal in top right positive
  */
  TEST_F(TrigTest, testPosAngleDiff_TR ){
    ASSERT_EQ(AngleDiff(-10, -55), -45.0);
  }

  /**
  * Test for a goal in top right negative
  */
  TEST_F(TrigTest, testNegAngleDiff_TR ){
    ASSERT_EQ(AngleDiff(-55, -10), 45.0);
  }

  /**
  * Test for a goal in bottom right positive
  */
  TEST_F(TrigTest, testPosAngleDiff_BR ){
    ASSERT_EQ(AngleDiff(-90, -135), -45.0);
  }

  /**
  * Test for a goal in bottom right negative
  */
  TEST_F(TrigTest, testNegAngleDiff_BR ){
    ASSERT_EQ(AngleDiff(-135, -90), 45.0);
  }

} //namespace

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
