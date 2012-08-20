#include "trig.h"

namespace se306p1 {
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

double NormalizeAngle (double theta) {
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
    theta = NormalizeAngle(theta);
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
