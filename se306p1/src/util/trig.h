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
    return sin(DegreesToRadians(degrees));
  }

  /**
  * Calculates the Cosine of an angle in degrees
  * Parameters: degrees - the angle whose Cosine is to be computed in degrees
  * Returns: the Cosine of that angle in degres
  */
  inline double DegCos (double degrees) {
    return cos(DegreesToRadians(degrees));
  }

  /**
  * Calculates the Tan of an angle in degrees
  * Parameters: degrees - the angle whose tan is to be computed in degrees
  * Returns: the tan of that angle in degres
  */
  inline double DegTan (double degrees) {
    return tan(DegreesToRadians(degrees));
  }

  /**
  * Calculates the Sine of an angle in degrees
  * Parameters: degrees - the angle whose sine is to be computed in degrees
  * Returns: the sine of that angle in degres
  */
  inline double DegATan (double x) {
    return (double) RadiansToDegrees(atan(x));
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

  double AbsAngleDiff(double theta, double phi) {
    double diff;

    if (theta == phi) {
      diff = 0.0;
    } else if (theta >= 0.0 && phi >= 0.0) {
      diff = fabs(theta - phi);
    } else if (theta <= 0.0 && phi <= 0.0) {
      diff = fabs(theta - phi);
    } else if (theta >= 0.0 && phi <= 0.0) {
      diff = 180.0 - theta + 180.0 - fabs(phi);
    } else if (theta <= 0.0 && phi >= 0.0) {
      diff = 180.0 - fabs(theta) + 180.0 - phi;
    } else {
      diff = 0.0;
    }

    return diff;
  }
}
