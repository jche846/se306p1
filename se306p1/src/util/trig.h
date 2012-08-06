#pragma once

#include <cmath>
#include <geometry_msgs/Quaternion.h>

#include <tf/transform_datatypes.h>

namespace se306p1
{
  /**
  * Converts a given double from degrees to radians,
  * Parameters: degrees - a double representing an angle in degrees
  * Returns:  the same angle measured in radians
  */
  inline double DegreesToRadians (double degrees) {
    return degrees * (M_PI / 180.0);
  }

  /**
  * Converts a given double from radians to degrees.
  * Paranmeters: radians - a double representing an angle in radians
  * Returns: the same angle measured in degress
  */
  inline double RadiansToDegrees (double radians) {
    return radians * (180.0 / M_PI);
  }

  /**
  * Calculates the Sine of an angle in degrees
  * Parameters: degrees - the angle whose sine is to be computed in degrees
  * Returns: the sine of that angle in degres
  */
  inline double DegSin (double degrees) {
    return RadiansToDegrees(sin(DegreesToRadians(degrees)));
  }

  /**
  * Calculates the Cosine of an angle in degrees
  * Parameters: degrees - the angle whose Cosine is to be computed in degrees
  * Returns: the Cosine of that angle in degres
  */
  inline double DegCos (double degrees) {
    return RadiansToDegrees(cos(DegreesToRadians(degrees)));
  }

  /**
  * Calculates the Tan of an angle in degrees
  * Parameters: degrees - the angle whose tan is to be computed in degrees
  * Returns: the tan of that angle in degres
  */
  inline double DegTan (double degrees) {
    return RadiansToDegrees(tan(DegreesToRadians(degrees)));
  }

  /**
  * Calculates the Sine of an angle in degrees
  * Parameters: degrees - the angle whose sine is to be computed in degrees
  * Returns: the sine of that angle in degres
  */
  inline double DegATan (double degrees) {
    return RadiansToDegrees(atan(DegreesToRadians(degrees)));
  }

  /**
  * Convert quaternion message to RPY (roll, pitch, yaw).
  */
  inline void QuaternionMsgToRPY(const geometry_msgs::Quaternion &q,
                                 double &roll, double &pitch, double &yaw) {
    btQuaternion tq;
    tf::quaternionMsgToTF(q, tq);
    btMatrix3x3(tq).getRPY(roll, pitch, yaw);
  }
}
