// #include "KrakenLib/splineGen.hpp"
// #include <cmath>

// SplineGen::SplineGen() {}

// void SplineGen::addWaypoint(const Point& waypoint) {
//     waypoints.push_back(waypoint);
// }

// void SplineGen::generateSpline() {
//     spline.clear(); // Clear any existing spline points

//     if (waypoints.size() < 2) return; // Need at least two waypoints to
//     generate a spline

//     // Calculate quintic Hermite spline interpolation
//     calculateQuinticHermiteSpline();
// }

/**
 * \brief Calculates a quintic Hermite spline through the waypoints.
 *
 * This function calculates a quintic Hermite spline through the waypoints,
 * using automated tension to create a smooth path. The result is a sequence of
 * points that make up the spline, which can be used to generate a path for a
 * robot to follow.
 */
// void SplineGen::calculateQuinticHermiteSpline() {
//     size_t n = waypoints.size();
//     if (n < 2) return; // Need at least two waypoints to generate a spline

//     // Number of steps for interpolation
//     const int steps = 100;

//     // Helper function to calculate tangents with automated tension
//     auto calculateTangent = [&](size_t index) {
//         double tangentX = 0.0;
//         double tangentY = 0.0;
//         double tension = 1.0; // Default tension

//         if (index > 0 && index < n - 1) {
//             double prevDist = std::hypot(waypoints[index].x - waypoints[index
//             - 1].x,
//                                           waypoints[index].y -
//                                           waypoints[index - 1].y);
//             double nextDist = std::hypot(waypoints[index + 1].x -
//             waypoints[index].x,
//                                           waypoints[index + 1].y -
//                                           waypoints[index].y);
//             tension = (prevDist + nextDist) / 2.0; // Average distance for
//             tension

//             // Scale the tension to avoid excessive values
//             tension = std::max(0.1, std::min(tension / 10.0, 1.0)); // Clamp
//             tension between 0.1 and 1.0
//         }

//         if (index == 0) { // First segment
//             tangentX = (waypoints[1].x - waypoints[0].x) / 2.0;
//             tangentY = (waypoints[1].y - waypoints[0].y) / 2.0;
//         } else if (index == n - 1) { // Last segment
//             tangentX = (waypoints[n - 1].x - waypoints[n - 2].x) / 2.0;
//             tangentY = (waypoints[n - 1].y - waypoints[n - 2].y) / 2.0;
//         } else { // Middle segments
//             tangentX = tension * (waypoints[index + 1].x - waypoints[index -
//             1].x); tangentY = tension * (waypoints[index + 1].y -
//             waypoints[index - 1].y);
//         }

//         return std::make_pair(tangentX, tangentY);
//     };

//     for (size_t i = 0; i < n - 1; ++i) {
//         const Point& start = waypoints[i];
//         const Point& end = waypoints[i + 1];

//         // Calculate tangents for the start and end points
//         auto [tangentStartX, tangentStartY] = calculateTangent(i);
//         auto [tangentEndX, tangentEndY] = calculateTangent(i + 1);

//         // Generate spline points between start and end
//         for (int step = 0; step <= steps; ++step) {
//             double t = static_cast<double>(step) / steps; // Normalized time
//             from 0 to 1

//             // Quintic Hermite basis functions
//             double h0 = 1 - 10 * t * t * t + 15 * t * t * t * t - 6 * t * t *
//             t * t * t; // h_0(t) double h1 = t - 6 * t * t * t + 8 * t * t *
//             t * t - 3 * t * t * t * t * t; // h_1(t) double h2 = t * t * (2 *
//             t - 3) + 9 * t * t * t * t - 2 * t * t * t * t * t; // h_2(t)
//             double h3 = 10 * t * t * t - 15 * t * t * t * t + 6 * t * t * t *
//             t * t; // h_3(t)

//             // Create a Point with the required parameters
//             Point point(
//                 h0 * start.x + h1 * tangentStartX + h2 * end.x + h3 *
//                 tangentEndX, h0 * start.y + h1 * tangentStartY + h2 * end.y +
//                 h3 * tangentEndY, atan2(
//                     h1 * tangentStartY + h2 * tangentEndY,
//                     h1 * tangentStartX + h2 * tangentEndX
//                 ), // Calculate heading based on tangents
//                 0.0, // Placeholder for velocity, to be handled in 2dmp
//                 0.0  // Placeholder for acceleration, to be handled in 2dmp
//             );

//             spline.push_back(point);
//         }
//     }
// }

#include "KrakenLib/splineGen.hpp"
#include <cmath>

SplineGen::SplineGen() {}

void SplineGen::addWaypoint(const Point &waypoint) {
  waypoints.push_back(waypoint);
}

void SplineGen::generateSpline() {
  spline.clear(); // Clear any existing spline points

  if (waypoints.size() < 2)
    return; // Need at least two waypoints to generate a spline

  // Calculate quintic Hermite spline interpolation
  calculateQuinticHermiteSpline();
}
const std::vector<Point> &SplineGen::getSpline() const { return spline; }
/**
 * \brief Calculates a quintic Hermite spline through the waypoints.
 *
 * This function calculates a quintic Hermite spline through the waypoints,
 * using automated tension to create a smooth path. The result is a sequence of
 * points that make up the spline, which can be used to generate a path for a
 * robot to follow.
 */
void SplineGen::calculateQuinticHermiteSpline() {
  size_t n = waypoints.size();
  if (n < 2)
    return; // Need at least two waypoints to generate a spline

  // Number of steps for interpolation
  const int steps = 100;

  // Helper function to calculate tangents with automated tension
  auto calculateTangent = [&](size_t index) {
    double tangentX = 0.0;
    double tangentY = 0.0;
    double tension = 1.0; // Default tension

    if (index > 0 && index < n - 1) {
      double prevDist = std::hypot(waypoints[index].x - waypoints[index - 1].x,
                                   waypoints[index].y - waypoints[index - 1].y);
      double nextDist = std::hypot(waypoints[index + 1].x - waypoints[index].x,
                                   waypoints[index + 1].y - waypoints[index].y);
      tension = (prevDist + nextDist) / 2.0; // Average distance for tension

      // Scale the tension to avoid excessive values
      tension =
          std::max(0.1, std::min(tension / 10.0,
                                 1.0)); // Clamp tension between 0.1 and 1.0
    }

    if (index == 0) { // First segment
      tangentX = (waypoints[1].x - waypoints[0].x) / 2.0;
      tangentY = (waypoints[1].y - waypoints[0].y) / 2.0;
    } else if (index == n - 1) { // Last segment
      tangentX = (waypoints[n - 1].x - waypoints[n - 2].x) / 2.0;
      tangentY = (waypoints[n - 1].y - waypoints[n - 2].y) / 2.0;
    } else { // Middle segments
      tangentX = tension * (waypoints[index + 1].x - waypoints[index - 1].x);
      tangentY = tension * (waypoints[index + 1].y - waypoints[index - 1].y);
    }

    return std::make_pair(tangentX, tangentY);
  };

  for (size_t i = 0; i < n - 1; ++i) {
    const Point &start = waypoints[i];
    const Point &end = waypoints[i + 1];

    // Calculate tangents for the start and end points
    auto [tangentStartX, tangentStartY] = calculateTangent(i);
    auto [tangentEndX, tangentEndY] = calculateTangent(i + 1);

    // Generate spline points between start and end
    for (int step = 0; step <= steps; ++step) {
      double t =
          static_cast<double>(step) / steps; // Normalized time from 0 to 1

      // Quintic Hermite basis functions
      double h0 = 1 - 10 * t * t * t + 15 * t * t * t * t -
                  6 * t * t * t * t * t; // h_0(t)
      double h1 = t - 6 * t * t * t + 8 * t * t * t * t -
                  3 * t * t * t * t * t; // h_1(t)
      double h2 = t * t * (2 * t - 3) + 9 * t * t * t * t -
                  2 * t * t * t * t * t; // h_2(t)
      double h3 =
          10 * t * t * t - 15 * t * t * t * t + 6 * t * t * t * t * t; // h_3(t)

      // Create a Point with the required parameters
      Point point(
          h0 * start.x + h1 * tangentStartX + h2 * end.x + h3 * tangentEndX,
          h0 * start.y + h1 * tangentStartY + h2 * end.y + h3 * tangentEndY,
          atan2(h1 * tangentStartY + h2 * tangentEndY,
                h1 * tangentStartX +
                    h2 * tangentEndX), // Calculate heading based on tangents
          0.0, // Placeholder for velocity, to be handled in 2dmp
          0.0  // Placeholder for acceleration, to be handled in 2dmp
      );

      spline.push_back(point);
    }
  }
}
