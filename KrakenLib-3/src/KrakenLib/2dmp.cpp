// #include "KrakenLib/2dmp.hpp"
// #include <cmath>
// #include "pros/adi.hpp"
// MotionProfile::MotionProfile(double max_velocity, double max_acceleration)
//     : max_velocity(max_velocity), max_acceleration(max_acceleration) {}

// void MotionProfile::addWaypoint(const Point& waypoint) {
//     splineGen.addWaypoint(waypoint); // Add waypoint to spline generator
// }

// void MotionProfile::generateMotionProfile() {
//     splineGen.generateSpline(); // Generate spline from waypoints
//     calculateMotionProfile(); // Calculate motion profile based on the spline
// }

// void MotionProfile::calculateMotionProfile() {
//     const std::vector<Point>& splinePoints = splineGen.getSpline();
//     motionProfile.clear(); // Clear any existing profile

//     if (splinePoints.size() < 2) return; // Need at least two points for motion profile

//     // Initialize the first motion point
//     Point initialPoint = splinePoints[0];
//     initialPoint.velocity = 0.0; // Starting velocity
//     initialPoint.acceleration = 0.0; // Starting acceleration
//     motionProfile.push_back(initialPoint);


   
//     double startTime = pros::millis();

//     // Calculate velocity and acceleration for each point in the spline
//     for (size_t i = 1; i < splinePoints.size(); ++i) {
//         Point motionPoint = splinePoints[i];

//         double dx = motionPoint.x - splinePoints[i - 1].x;
//         double dy = motionPoint.y - splinePoints[i - 1].y;
//         double distance = std::hypot(dx, dy);

//         // Calculate the elapsed time since the start
//         double currentTime = pros::millis(); // Get the current time
//         double elapsedTime = currentTime - startTime; // Calculate elapsed time

//         // Calculate the desired time to travel this distance at max velocity
//         double timeToTravel = distance / max_velocity;

//         // Calculate desired velocity
//         motionPoint.velocity = std::min(max_velocity, distance / timeToTravel);

//         // Calculate acceleration
//         double previousVelocity = motionProfile.back().velocity;
//         motionPoint.acceleration = (motionPoint.velocity - previousVelocity) / elapsedTime;

//         // Ensure that acceleration does not exceed max_acceleration
//         if (std::abs(motionPoint.acceleration) > max_acceleration) {
//             motionPoint.acceleration = (motionPoint.acceleration > 0 ? max_acceleration : -max_acceleration);
//             // Adjust velocity based on the new acceleration
//             motionPoint.velocity = previousVelocity + motionPoint.acceleration * elapsedTime;
//         }

//         // Push the calculated motion point to the profile
//         motionProfile.push_back(motionPoint);
//     }
// }

// const std::vector<Point>& MotionProfile::getMotionProfile() const {
//     return motionProfile;
// }

#include "KrakenLib/2dmp.hpp"
#include <cmath>
#include "KrakenLib/splineGen.hpp"
/**
 * @brief Constructor for MotionProfile.
 * @param max_velocity The maximum velocity allowed in the motion profile.
 * @param max_acceleration The maximum acceleration allowed in the motion profile.
 */
MotionProfile::MotionProfile(double max_velocity, double max_acceleration)
    : max_velocity(max_velocity), max_acceleration(max_acceleration) {}

/**
 * @brief Add a waypoint to the motion profile.
 * @param waypoint The waypoint to add. The waypoint should have the position and possibly velocity and/or acceleration set.
 */
void MotionProfile::addWaypoint(const Point& waypoint) {
    splineGen.addWaypoint(waypoint);
}

/**
 * @brief Generates a motion profile by creating a spline from waypoints
 * and performing both forward and backward passes to calculate the profile.
 */
void MotionProfile::generateMotionProfile() {
    splineGen.generateSpline(); // Generate the spline
    calculateForwardPass();     // Perform forward pass
    calculateBackwardPass();    // Perform backward pass
}

/**
 * @brief Performs a forward pass of the motion profile calculation.
 *
 * This function starts with the initial point and iterates through the
 * spline points, calculating the maximum allowed velocity for each segment
 * and the resulting velocity and acceleration for each point. The
 * calculated points are then added to the motion profile.
 *
 * @return Nothing
 */
void MotionProfile::calculateForwardPass() {
    // Clear the profile and generate the spline
    splineGen.generateSpline();  // This generates the spline
    const std::vector<Point>& splinePoints = splineGen.getSpline();  // Get the spline points
    motionProfile.clear();  // Clear the motion profile

    if (splinePoints.empty()) return;

    // Start with the initial point
    Point initialPoint = splinePoints[0];
    initialPoint.velocity = 0.0;  // Initial velocity
    motionProfile.push_back(initialPoint);

    // Loop through the spline points to calculate velocity and acceleration
    for (size_t i = 1; i < splinePoints.size(); ++i) {
        const Point& prev = motionProfile.back();
        const Point& current = splinePoints[i];

        double dx = current.x - prev.x;
        double dy = current.y - prev.y;
        double distance = std::hypot(dx, dy);

        // Calculate maximum velocity for this segment
        double maxAllowedVelocity = std::sqrt(
            prev.velocity * prev.velocity + 2 * max_acceleration * distance
        );
        double velocity = std::min(max_velocity, maxAllowedVelocity);

        // Create a new motion point with calculated velocity and acceleration
        Point motionPoint = current;
        motionPoint.velocity = velocity;
        motionPoint.acceleration = (velocity - prev.velocity) / distance;
        motionProfile.push_back(motionPoint);
    }
}

void MotionProfile::calculateBackwardPass() {
    if (motionProfile.empty()) return;

    // Iterate backward through the motion profile to adjust velocities
    for (int i = motionProfile.size() - 2; i >= 0; --i) {
        Point& current = motionProfile[i];
        const Point& next = motionProfile[i + 1];

        double dx = next.x - current.x;
        double dy = next.y - current.y;
        double distance = std::hypot(dx, dy);

        // Adjust velocity based on the next point's velocity
        double maxAllowedVelocity = std::sqrt(
            next.velocity * next.velocity + 2 * max_acceleration * distance
        );
        current.velocity = std::min(current.velocity, maxAllowedVelocity);

        // Recalculate acceleration
        current.acceleration = (next.velocity - current.velocity) / distance;
    }
}


/**
 * @brief Retrieve the generated motion profile.
 *
 * @return A constant reference to the vector of Points in the motion profile
 */
const std::vector<Point>& MotionProfile::getMotionProfile() const {
    return motionProfile;
}
