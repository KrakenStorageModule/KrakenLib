#ifndef MOTION_PROFILE_HPP
#define MOTION_PROFILE_HPP

#include <vector>
#include "point.hpp" // Include your existing point structure
#include "splineGen.hpp" // Include the spline generation header

class MotionProfile {
public:
    MotionProfile(double max_velocity, double max_acceleration);

    void addWaypoint(const Point& waypoint);
    void generateMotionProfile();
    const std::vector<Point>& getMotionProfile() const;

private:
    SplineGen splineGen; // Spline generator
    std::vector<Point> motionProfile; // Generated motion profile
    double max_velocity; // Maximum velocity
    double max_acceleration; // Maximum acceleration

    void calculateMotionProfile();
};

#endif // MOTION_PROFILE_HPP