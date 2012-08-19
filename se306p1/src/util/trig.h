#pragma once

#include <cmath>
#include <complex>
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
double AngleDiff(double theta, double phi);

/**
 * Find the angle of the vector that points from one point to another using
 * the Stage coordinate system.
 *
 * @param p1 The point from which to find the angle of the vector that points
 * to p2.
 * @param p2 The point which the angle will point to.
 * @return The angle of the vector which points from p1 to p2.
 */
double AngleBetweenPoints(const Vector2 &p1, const Vector2 &p2);

/**
* Normalizes an angle so that if it is outside the allowable range it fixes it.
* 
* @param theta The angle to be normalized
*
* @return the normalized angle
*/
double NormalizeAngle (double theta);

/**
* Finds a point that is located the distance that is diameter from the center at an angle of theta.
*
* @param center The center of the polygon
* @param theta The angle from the center of the polygon that point should be
* @param diameter The distance from the center of the polygon that the point should be
* @return point The point that the function calculates.
*/
Vector2 FindPointFromTheta (Vector2 center, double theta, double diameter);

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
std::vector<Vector2> FindRobotPositions (Vector2 center, double theta, double diameter, int numRobots, int numSides);

/**
 * Find the points on a circle.
 */
std::vector<Vector2> FindCirclePositions (Vector2 center, double diameter, int numRobots);

}
