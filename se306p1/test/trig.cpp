#include <gtest/gtest.h>
#include "../src/util/trig.h"
#include <cmath>
#include <iostream>

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

  //TODO QuaternionMsgToRPY

  // ANGLE DIFF

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
  * Tests NormalizeAngle for the case of greater than 180
  */
  TEST_F(TrigTest, testNormalizeAngle_positive) {
    ASSERT_NEAR(-160.0, NormalizeAngle(200.0), 0.0001);
  }

  /**
  * Tests NormalizeAngle for the case of less than -180
  */
  TEST_F(TrigTest, testNormalizeAngle_negative) {
    ASSERT_NEAR(160, NormalizeAngle(-200.0), 0.0001);
  }

  /**
  * Tests NormalizeAngle for the case of greater than 360
  */
  TEST_F(TrigTest, testNormalizeAngle_gt360) {
    ASSERT_NEAR(40, NormalizeAngle(400.0), 0.0001);
  }

  /**
  * Tests NormalizeAngle for the case of greater than 720, i.e. more that 2*360
  */
  TEST_F(TrigTest, testNormalizeAngle_gt720) {
    ASSERT_NEAR(50, NormalizeAngle(770.0), 0.0001);
  }

  /**
  * Tests NormalizeAngle for the case of less than -720, i.e. more that 2*360
  */
  TEST_F(TrigTest, testNormalizeAngle_ltNeg720) {
    ASSERT_NEAR(-60, NormalizeAngle(-1140.0), 0.0001);
  }

  /**
  * Tests NormalizeAngle for the case of less than -720 that will go positive,
  * i.e. more that 2*360
  */
  TEST_F(TrigTest, testNormalizeAngle_ltNeg720topos) {
    ASSERT_NEAR(130, NormalizeAngle(-2030.0), 0.0001);
  }

  /**
  * Tests FindPointFromTheta for the case of the topleft sector at 45 degs
  */
  TEST_F(TrigTest, testFindPointFromTheta_easytopleft) {
    ASSERT_NEAR(-0.70710678, FindPointFromTheta(Vector2(0.0, 0.0), 45.0, 1.0).x_, 0.0001);
  ASSERT_NEAR(0.70710678, FindPointFromTheta(Vector2(0.0, 0.0), 45.0, 1.0).y_, 0.0001);
  }

  /**
  * Tests FindPointFromTheta for the case of the topleft sector at 45 degs
  */
  TEST_F(TrigTest, testFindPointFromTheta_easybottomleft) {
    ASSERT_NEAR(-0.70710678, FindPointFromTheta(Vector2(0.0, 0.0), 135.0, 1.0).x_, 0.0001);
    ASSERT_NEAR(-0.70710678, FindPointFromTheta(Vector2(0.0, 0.0), 135.0, 1.0).y_, 0.0001);
  }

  /**
  * Tests FindPointFromTheta for the case of the topleft sector at 45 degs
  */
  TEST_F(TrigTest, testFindPointFromTheta_easytopright) {
    ASSERT_NEAR(0.70710678, FindPointFromTheta(Vector2(0.0, 0.0), -45.0, 1.0).x_, 0.0001);
    ASSERT_NEAR(0.70710678, FindPointFromTheta(Vector2(0.0, 0.0), -45.0, 1.0).y_, 0.0001);
  }

  /**
  * Tests FindPointFromTheta for the case of the topleft sector at 45 degs
  */
  TEST_F(TrigTest, testFindPointFromTheta_easybottomright) {
    ASSERT_NEAR(0.70710678, FindPointFromTheta(Vector2(0.0, 0.0), -135.0, 1.0).x_, 0.0001);
    ASSERT_NEAR(-0.70710678, FindPointFromTheta(Vector2(0.0, 0.0), -135.0, 1.0).y_, 0.0001);
  }
  
  /**
  * Dave's filthy test case
  */
  TEST_F(TrigTest, testFindRobotPositions_davesfilthytest) {
    std::vector<Vector2> v = FindRobotPositions(Vector2(4, 4), 0, 100.0, 5, 5);
    for (auto &x : v) {
      ROS_INFO("%f %f", x.x_, x.y_);
    }    
  }


  /** FIND ROBOT POSITIONS
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

  //TODO ANGLE BETWEEN POINTS

  //TODO NORMALIZE ANGLE

  //TODO FIND POINT FROM THETA

  //TODO FIND ROBOT POSITIONS

} //namespace

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
