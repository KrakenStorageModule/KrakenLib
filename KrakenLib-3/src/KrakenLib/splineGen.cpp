#include "KrakenLib/splineGen.hpp"
#include <cmath>

SplineGen::SplineGen() {}

void SplineGen::addWaypoint(const Point& waypoint) {
    waypoints.push_back(waypoint);
}

void SplineGen::generateSpline() {
    spline.clear(); // Clear any existing spline points

    if (waypoints.size() < 2) return; // Need at least two waypoints to generate a spline

    // Calculate quintic Hermite spline interpolation
    calculateQuinticHermiteSpline();
}

void SplineGen::calculateQuinticHermiteSpline() {
    size_t n = waypoints.size();

    for (size_t i = 0; i < n - 1; ++i) {
        const Point& start = waypoints[i];
        const Point& end = waypoints[i + 1];
         
         //These next two blocks exist to allow for straight lines
         // Calculate the tangent (velocity) for the start and end points
        double tangentStartX = end.x - start.x; // Simple tangent calculation
        double tangentStartY = end.y - start.y; // Simple tangent calculation
        double tangentEndX = tangentStartX; // Same for straight line
        double tangentEndY = tangentStartY; // Same for straight line

        // Normalize tangents if necessary
        double length = std::sqrt(tangentStartX * tangentStartX + tangentStartY * tangentStartY);
        if (length > 0) {
            tangentStartX /= length;
            tangentStartY /= length;
            tangentEndX /= length;
            tangentEndY /= length;
        }

        // For this implementation, we will assume tangents are provided or calculated externally.
        // You could also add logic to calculate tangents based on the waypoints.

        // Generate spline points between start and end
        int steps = 100; // Number of steps for interpolation
        for (int step = 0; step <= steps; ++step) {
            double t = static_cast<double>(step) / steps; // Normalized time from 0 to 1

            // Hermite basis functions
            double h00 = (1 + 2 * t) * (1 - t) * (1 - t);
            double h10 = t * (1 - t) * (1 - t);
            double h01 = t * t * (3 - 2 * t);
            double h11 = t * t * (t - 1);

             // Create a Point with the required parameters
            Point point(
                h00 * start.x + h10 * tangentStartX + h01 * end.x + h11 * tangentEndX,
                h00 * start.y + h10 * tangentStartY + h01 * end.y + h11 * tangentEndY,
                atan2(end.y - start.y, end.x - start.x), // Calculate heading
                0.0, // Placeholder for velocity, to be handled in 2dmp
                0.0  // Placeholder for acceleration, to be handled in 2dmp
            );

            spline.push_back(point);
        }
    }
}

const std::vector<Point>& SplineGen::getSpline() const {
    return spline;
}