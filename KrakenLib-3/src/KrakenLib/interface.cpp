// You can create a function that takes a list of waypoints and executes the
// movement routine:

#include "interface.hpp"
#include "KrakenLib/2dmp.hpp"
#include "KrakenLib/ramsete.hpp"
#include "KrakenLib/splineGen.hpp"
#include "KrakenLib/point.hpp"
// Define a threshold for reaching the desired state
const double threshold = 0.1;
// Function to check if the robot has reached the desired state
bool reachedDesiredState(const Point &currentPose, const Point &desiredState) {
  double distance = std::hypot(currentPose.x - desiredState.x,
                               currentPose.y - desiredState.y);
  return distance <
         threshold; // Define a threshold for reaching the desired state
}


  /**
   * \brief Runs the Ramsete controller for the given motion profile
   *
   * This function takes a Ramsete controller, an odometry object, a motion
   * profile, a spline generator, and two boolean flags for reversing and
   * clockwise directions. It runs the Ramsete controller to follow the
   * motion profile. It does not return until the robot has reached the
   * desired state.
   *
   * \param ramseteController The Ramsete controller to use
   * \param odometry The odometry object to get the current pose
   * \param motionProfile The motion profile to follow
   * \param splineGen The spline generator to generate the motion profile
   * points
   * \param reverse Whether to reverse the motion profile points
   * \param clockwise Whether to move clockwise or counterclockwise
   */
void run(RamseteController &ramseteController, Odometry &odometry,
         MotionProfile motionProfile, SplineGen &splineGen, bool reverse,
         bool clockwise) {

  // Retrieve motion profile points
  auto motionProfilePoints = motionProfile.getMotionProfile();

  // Reverse waypoints if reverse is true
  if (reverse) {
    for (auto &waypoint : motionProfilePoints) {
      waypoint.x = -waypoint.x; // Reverse the x position
      waypoint.y = -waypoint.y; // Reverse the y position
      waypoint.theta += 180;    // Reverse the heading
    }
  }
  // Generate the motion profile
  motionProfile.generateMotionProfile();

  for (const auto &desiredState : motionProfilePoints) {
    while (true) {
      Point currentPose = Point(odometry.getX(), odometry.getY(),
                                odometry.getHeading(), 0.0, 0.0);

      // Update the Ramsete controller with the current pose and desired state
      ramseteController.update(currentPose, desiredState);

      // Apply motor voltages
      ramseteController.applyMotorVoltages();

      // Check if the robot has reached the desired state
      if (reachedDesiredState(currentPose, desiredState)) {
        break; // Exit the loop if the desired state is reached
      }

      pros::delay(20); // Delay for the next control cycle
    }
  }
}
