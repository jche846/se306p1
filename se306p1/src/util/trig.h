#pragma once

#include <cmath>
#include <geometry_msgs/Quaternion.h>

#include <tf/transform_datatypes.h>
#include "vector2.h"

namespace se306p1 {
/**
 * Converts a given double from degrees to radians,
 * Parameters: degrees - a double representing an angle in degrees
 * Returns:  the same angle measured in radians
 */
inline double DegreesToRadians(double degrees) {
  return degrees * (M_PI / 180.0);
}

/**
 * Converts a given double from radians to degrees.
 * Paranmeters: radians - a double representing an angle in radians
 * Returns: the same angle measured in degrees
 */
inline double RadiansToDegrees(double radians) {
  return radians * (180.0 / M_PI);
}

/**
 * Calculates the Sine of an angle in degrees
 * Parameters: degrees - the angle whose sine is to be computed in degrees
 * Returns: the sine of that angle in degrees
 */
inline double DegSin(double degrees) {
  return sin(DegreesToRadians(degrees));
}

/**
 * Calculates the Cosine of an angle in degrees
 * Parameters: degrees - the angle whose Cosine is to be computed in degrees
 * Returns: the Cosine of that angle in degrees
 */
inline double DegCos(double degrees) {
  return cos(DegreesToRadians(degrees));
}

/**
 * Calculates the Tan of an angle in degrees
 * Parameters: degrees - the angle whose tan is to be computed in degrees
 * Returns: the tan of that angle in degrees
 */
inline double DegTan(double degrees) {
  return tan(DegreesToRadians(degrees));
}

/**
 * Calculates the Sine of an angle in degrees
 * Parameters: degrees - the angle whose sine is to be computed in degrees
 * Returns: the sine of that angle in degrees
 */
inline double DegATan(double x) {
  return (double) RadiansToDegrees(atan(x));
}

/**
 * Convert quaternion message to RPY (roll, pitch, yaw).
 */
inline void QuaternionMsgToRPY(const geometry_msgs::Quaternion &q, double &roll,
                               double &pitch, double &yaw) {
  btQuaternion tq;
  tf::quaternionMsgToTF(q, tq);
  btMatrix3x3(tq).getRPY(roll, pitch, yaw);
}

/**
 * Finds the difference from theta to phi in degrees, including direction.
 *
 * @param theta The angle we are moving from.
 * @param phi The angle we are moving to.
 * @return The angular distance from theta to phi.
 */
double AngleDiff(double theta, double phi) {
  double diff;

  if (theta < 0.0)
    theta = theta + 360.0;

  if (phi < 0.0)
    phi = phi + 360.0;

  diff = phi - theta;

  if (diff > 180.0)
    diff = diff - 360.0;

  if (diff < -180.0)
    diff = 360.0 + diff;

  return diff;
}

/**
 * Find the angle of the vector that points from one point to another using
 * the Stage coordinate system.
 *
 * @param p1 The point from which to find the angle of the vector that points
 * to p2.
 * @param p2 The point which the angle will point to.
 * @return The angle of the vector which points from p1 to p2.
 */
double AngleBetweenPoints(const Vector2 &p1, const Vector2 &p2) {
  double dx = p2.x_ - p1.x_;
  double dy = p2.y_ - p1.y_;
  double phi = 0.0;

  double a_tan = fabs(DegATan(dy / dx));

  if (dy == 0.0 && dx == 0.0) {
    phi = 0.0;
  } else if (dy >= 0.0 && dx >= 0.0) {
    //sector 1
    phi = -90.0 + a_tan;
  } else if (dy >= 0.0 && dx < 0.0) {
    //sector 2
    phi = 90.0 - a_tan;
  } else if (dy < 0.0 && dx < 0.0) {
    //sector 3
    phi = 90.0 + a_tan;
  } else if (dy < 0.0 && dx >= 0.0) {
    //sector 4
    phi = -90.0 - a_tan;
  }

  return (double) phi;
}

/**
* Normalizes an angle so that if it is outside the allowable range it fixes it.
* 
* @param theta The angle to be normalized
*
* @return the normalized angle
*/
double normalizeAngle (double theta) {
  // Ensures that the angle is within 360 and -360
  while (theta > 360.0) theta-=360.0; 
  while (theta < -360.0) theta += 360.0;
  // Fixes the sign and the angle amount so it is within 180 and -180
  if(theta > 180.0){
    theta = -360.0 + theta; 
  } else if (theta < -180.0){
    theta = 360.0 + theta;
  }
  return theta;
}

/**
* Finds a point that is located the distance that is diameter from the center at an angle of theta.
*
* @param center The center of the polygon
* @param theta The angle from the center of the polygon that point should be
* @param diameter The distance from the center of the polygon that the point should be
* @return point The point that the function calculates.
*/
Vector2 FindPointFromTheta (Vector2 center, double theta, double diameter) {
  double x; // x value of the point that we are calculating
  double y; // y value of the point that we are calculating
  if (theta < 0.0 && theta >= -90.0) {
    // Quad 1 (top right)
    theta = fabs(theta);
    x = center.x_ + (diameter * DegSin(theta));
    y = center.y_ + (diameter * DegCos(theta));
  } else if (theta < -90.0 && theta <= -180.0) {
    // Quad 2 (bottom right)
    theta = fabs(theta);
    x = center.x_ + (diameter * DegCos(theta));
    y = center.y_ - (diameter * DegSin(theta));
  } else if (theta > 0.0 && theta <= 90.0) {
    // Quad 3 (top left)
    x = center.x_ - (diameter * DegSin(theta));
    y = center.y_ + (diameter * DegCos(theta));
  } else if (theta > 90.0 && theta <= 180.0) {
    // Quad4 (bottom left) 
    x = center.x_ - (diameter * DegCos(theta));
    y = center.y_ - (diameter * DegSin(theta));
  } else {
    // Throw an error as the angle is invalid
    x = std::numeric_limits<double>::quiet_NaN();
    y = std::numeric_limits<double>::quiet_NaN();
  }
  return Vector2 (x, y);
}

/**
* Finds the points that robots should take in order to make the polygon with number of edges specified.
*
* Special cases to be wary of:
*   - If there are more sides than there are robots, the number of sides should be the same as the number
*     of robots so that there is a robot on every vertex
*   - If there are 4 sides (i.e. a square) then the shape should be rotated 45 degrees as to make
*     it look like a square rather than a diamond
*
* @param center A Vector2 of the position of the center of the polygon
* @param theta A double that determines which direction that the top vertex should be from the diameter
* @param diameter A double that determines the distance that vertexes of the polygon should be from the center
* @param numRobots The number of robots that should be in the polygon
* @param numSides The number of sides of the polygon
*
* @return A vector of Vector2s that determines the positions that the robots should take
*/
std::vector<Vector2> FindRobotPositions (Vector2 center, double theta, double diameter, int numRobots, int numSides) {
  // Handles special case of there being more sides than robots and 
  // makes sure that there will be a robot on every corner by decreasing 
  // the number of sides
  if (numSides > numRobots) {
    numSides = numRobots;
  }
  // Handles the special case for a square. rotate the square 45 degrees 
  // to make it a square as opposed to a diamond.
  if (numSides == 4) {
    theta += 45;
    theta = normalizeAngle(theta); 
  }
  // Set the size of the angle change 
  double angleStepSize = 360.0/numSides;
  std::vector<Vector2> positions;
  // Assign the positions of the vertexes of the polygon
  for (int i = 0 ; i < numSides ; i++) {
    positions.push_back(FindPointFromTheta(center, (theta + i*angleStepSize), diameter));
  }
  // We need to process the remaining robots that weren't used in vertices for the polygon
  numRobots -= numSides;
  // If there is 1 left over robot, either after the vertices or after placing equal number of extra robots 
  // on sides, then put it in the middle of the shape.
  if (numRobots%numSides == 1) {
    positions.push_back(center);
    numRobots--;  
  }

  // Process any left over robots to positions along each side of the shape. 

  // The maximum difference in the number of Robots on sides should be 1
  // Each side should have at minimum the integer division of numRobots by numSides
  // so the remainder is less than the number of sides. Then there should be 
  // and additional robot on n sides where n is the remainder of the integer division
  int baseNumberOfRobotsOnEachEdge = numRobots / numSides;
  int numSidesWithAdditionalRobot = numRobots % numSides;

  // Loop over each edge of the polygon adding the extra robots to it
  for (int i = 0; i < numSides ; i++) {

    // The two vertexes that we are positioning the robots between
    Vector2 vertex1 = positions.at(i);
    Vector2 vertex2 = positions.at((i+1) % numSides);

    // The difference between these two vertexes so that we know where to place the robots
    double dx = vertex1.x_ - vertex2.x_;
    double dy = vertex1.y_ - vertex2.y_;
    
    // Calculating the number of robots that will have to be added to the current edge
    int numRobotsToAdd = baseNumberOfRobotsOnEachEdge;
    if (i < numSidesWithAdditionalRobot){
      // Add basenumrobots + 1 to this edge
      numRobotsToAdd++;    
    }

    //booyakasha (when dave did some mad refactoring skillz)!

    // The size of the splits between each robot on the edge
    double xSplit = dx/(numRobotsToAdd+1);
    double ySplit = dy/(numRobotsToAdd+1);

    // Add the robots to the positions vector spaced equally along the edge
    for (int j = 0 ; j<numRobotsToAdd ; j++) {
      double x = vertex1.x_ + ((j+1) * xSplit);
      double y = vertex1.y_ + ((j+1) * ySplit);
      positions.push_back(Vector2(x,y));
    }
  }
  return positions;
}

}
