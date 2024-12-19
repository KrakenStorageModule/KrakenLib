// You can create a function that takes a list of waypoints and executes the
// movement routine:

#include "interface.hpp"
#include "KrakenLib/2dmp.hpp"

// Define a threshold for reaching the desired state
const double threshold = 0.1;
// Function to check if the robot has reached the desired state
bool reachedDesiredState(const Point &currentPose, const Point &desiredState) {
  double distance = std::hypot(currentPose.x - desiredState.x,
                               currentPose.y - desiredState.y);
  return distance <
         threshold; // Define a threshold for reaching the desired state
}

// You need to tune the max acceleration
void run(RamseteController &ramseteController, Odometry &odometry,
         MotionProfile motionProfile, bool reverse, bool clockwise) {

  SplineGen splineGen;
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
