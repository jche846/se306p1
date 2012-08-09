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
   * Returns: the same angle measured in degress
   */
  inline double RadiansToDegrees(double radians) {
    return radians * (180.0 / M_PI);
  }

  /**
   * Calculates the Sine of an angle in degrees
   * Parameters: degrees - the angle whose sine is to be computed in degrees
   * Returns: the sine of that angle in degres
   */
  inline double DegSin(double degrees) {
    return sin(DegreesToRadians(degrees));
  }

  /**
   * Calculates the Cosine of an angle in degrees
   * Parameters: degrees - the angle whose Cosine is to be computed in degrees
   * Returns: the Cosine of that angle in degres
   */
  inline double DegCos(double degrees) {
    return cos(DegreesToRadians(degrees));
  }

  /**
   * Calculates the Tan of an angle in degrees
   * Parameters: degrees - the angle whose tan is to be computed in degrees
   * Returns: the tan of that angle in degres
   */
  inline double DegTan(double degrees) {
    return tan(DegreesToRadians(degrees));
  }

  /**
   * Calculates the Sine of an angle in degrees
   * Parameters: degrees - the angle whose sine is to be computed in degrees
   * Returns: the sine of that angle in degres
   */
  inline double DegATan(double x) {
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

  /**
   * Finds the absolute difference between two angles using the Stage coordinate
   * system.
   *
   * @param theta The angle to find the difference from.
   * @param phi The angle to find the difference to.
   * @return The absolute difference between theta and phi.
   */
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
}
