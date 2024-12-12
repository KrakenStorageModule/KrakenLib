#include "KrakenLib/2dmp.hpp"
#include <cmath>

MotionProfile::MotionProfile(double max_velocity, double max_acceleration)
    : max_velocity(max_velocity), max_acceleration(max_acceleration) {}

void MotionProfile::addWaypoint(const Point& waypoint) {
    splineGen.addWaypoint(waypoint); // Add waypoint to spline generator
}

void MotionProfile::generateMotionProfile() {
    splineGen.generateSpline(); // Generate spline from waypoints
    calculateMotionProfile(); // Calculate motion profile based on the spline
}

void MotionProfile::calculateMotionProfile() {
    const std::vector<Point>& splinePoints = splineGen.getSpline();
    motionProfile.clear(); // Clear any existing profile

    // For each point in the spline, calculate the velocity and acceleration
    for (size_t i = 0; i < splinePoints.size(); ++i) {
        Point motionPoint = splinePoints[i];

        // Calculate velocity and acceleration based on the spline
        if (i > 0) {
            double dx = motionPoint.x - splinePoints[i - 1].x;
            double dy = motionPoint.y - splinePoints[i - 1].y;
            double distance = std::hypot(dx, dy);
            
            // Calculate time to travel this distance at max velocity
            double timeToTravel = distance / max_velocity;

            // Set velocity and time
            motionPoint.velocity = std::min(max_velocity, distance / timeToTravel);
            motionPoint.acceleration = (motionPoint.velocity - splinePoints[i - 1].velocity) / timeToTravel; // Simple acceleration calculation
        } else {
            motionPoint.velocity = 0.0; // Starting point
            motionPoint.acceleration = 0.0; // No acceleration at the start
        }

        motionProfile.push_back(motionPoint);
    }
}

const std::vector<Point>& MotionProfile::getMotionProfile() const {
    return motionProfile;
}