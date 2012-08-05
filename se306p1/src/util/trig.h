#pragma once

#include <cmath>

#define _USE_MATH_DEFINES

namespace se306p1 
{
  /**
  * Converts a given double from degrees to radians,
  * Parameters: degrees - a double representing an angle in degrees
  * Returns:  the same angle measured in radians
  */
  inline double DegreesToRadians (double degrees) {
    return radians = degrees * (M_PI / 180.0);
  }

  /**
  * Converts a given double from radians to degrees.
  * Paranmeters: radians - a double representing an angle in radians
  * Returns: the same angle measured in degress
  */
  inline double RadiansToDegrees (double radians) {
    return degrees = radians * (180.0 / M_PI);
  }

  /**
  * Calculates the Sine of an angle in degrees
  * Parameters: degrees - the angle whose sine is to be computed in degrees
  * Returns: the sine of that angle in degres
  */
  inline double DegSin (double degrees) {
    double radians = RadiansToDegrees(degrees);
    return RadiansToDegrees(sin(radians));
  }

  /**
  * Calculates the Cosine of an angle in degrees
  * Parameters: degrees - the angle whose Cosine is to be computed in degrees
  * Returns: the Cosine of that angle in degres
  */
  inline double DegCos (double degrees) {
    double radians = DegreesToRadians(degrees);
    return RadiansToDegrees(cos(radians));
  }

  /**
  * Calculates the Tan of an angle in degrees
  * Parameters: degrees - the angle whose tan is to be computed in degrees
  * Returns: the tan of that angle in degres
  */
  inline double DegTan (double degrees) {
    double radians = DegreesToRadians(degrees);
    return RadiansToDegrees(tan(radians));
  }

  /**
  * Calculates the Sine of an angle in degrees
  * Parameters: degrees - the angle whose sine is to be computed in degrees
  * Returns: the sine of that angle in degres
  */
  inline double DegATan (double degrees) {
    double radians = DegreesToRadians(degrees);
    return RadiansToDegrees(atan(radians));
  }

}