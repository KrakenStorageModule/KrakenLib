#pragma once
#include <cmath>
/**
 * @brief Struct representing a point in 2D space with additional properties.
 */
struct Point {
  double x;            ///< X coordinate
  double y;            ///< Y coordinate
  double theta;        ///< Orientation in radians
  double velocity;     ///< Velocity of the point
  double acceleration; ///< Acceleration of the point

  /**
   * @brief Constructor to initialize a Point.
   * @param x X coordinate.
   * @param y Y coordinate.
   * @param theta Orientation in radians.
   * @param velocity Velocity of the point.
   * @param acceleration Acceleration of the point.
   */
  Point(double x, double y, double theta, double velocity, double acceleration)
      : x(x), y(y), theta(theta), velocity(velocity),
        acceleration(acceleration) {}
};
