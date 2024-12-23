// #ifndef MOTION_PROFILE_HPP
// #define MOTION_PROFILE_HPP

// #include <vector>
// #include "point.hpp" // Include your existing point structure
// #include "splineGen.hpp" // Include the spline generation header
// #include "main.h"
// #include "pros/misc.hpp"
// class MotionProfile {
// public:
//     MotionProfile(double max_velocity, double max_acceleration);

//     void addWaypoint(const Point& waypoint);
//     void generateMotionProfile();
//     const std::vector<Point>& getMotionProfile() const;

// private:
//     SplineGen splineGen; // Spline generator
//     std::vector<Point> motionProfile; // Generated motion profile
//     double max_velocity; // Maximum velocity
//     double max_acceleration; // Maximum acceleration

//     void calculateMotionProfile();
// };

// #endif // MOTION_PROFILE_HPP
#ifndef MOTION_PROFILE_HPP
#define MOTION_PROFILE_HPP

#include "KrakenLib/point.hpp"
#include "KrakenLib/splineGen.hpp"
#include <vector>

/**
 * @brief Class for generating 2D motion profiles with forward and backward
 * passes.
 */
/**
 * @brief Class for generating 2D motion profiles with forward and backward
 * passes.
 */
class MotionProfile {
public:
  /**
   * @brief Constructor for the MotionProfile class.
   *
   * @param max_velocity The maximum allowed velocity for the motion profile.
   * @param max_acceleration The maximum allowed acceleration for the motion
   * profile.
   */
  MotionProfile(double max_velocity, double max_acceleration, SplineGen &splineGen);

  /**
   * @brief Adds a waypoint to the motion profile.
   *
   * @param waypoint The waypoint to add to the motion profile.
   */
  void addWaypoint(const Point &waypoint);

  /**
   * @brief Generates the motion profile using the forward and backward passes.
   */
  void generateMotionProfile();

  /**
   * @brief Returns the generated motion profile.
   *
   * @return A const reference to the generated motion profile.
   */
  const std::vector<Point> &getMotionProfile() const;

private:
  //SplineGen splineGen;              // Spline generator
  std::vector<Point> motionProfile; // Generated motion profile
  double max_velocity;              // Maximum velocity
  double max_acceleration;          // Maximum acceleration

  /**
   * @brief Calculates the maximum allowed velocity for each segment of the
   * motion profile using the forward pass.
   */
  void calculateForwardPass();

  /**
   * @brief Calculates the maximum allowed acceleration for each segment of the
   * motion profile using the backward pass.
   */
  void calculateBackwardPass();
};

#endif // MOTION_PROFILE_HPP
