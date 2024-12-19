
#include "Krakenlib/odom.hpp"
#include "devices.hpp"
Odometry::Odometry(pros::Imu &imu, pros::Rotation *parallelWheelSensor,
                   pros::Rotation *perpendicularWheelSensor, float wheel_diameter,
                   float gear_ratio, float tracking_wheel_diameter,
                   double parallelOffset, double perpendicularOffset)
    : imu(imu), parallelWheelSensor(parallelWheelSensor),
      perpendicularWheelSensor(perpendicularWheelSensor),
      wheel_diameter(wheel_diameter), gear_ratio(gear_ratio),
      tracking_wheel_diameter(tracking_wheel_diameter),
      parallelOffset(parallelOffset), perpendicularOffset(perpendicularOffset) {}

void Odometry::update() {
    calculateOdometry();
}

void Odometry::reset() {
    x = 0.0;
    y = 0.0;
    heading = 0.0;
    previous_heading = 0.0;
    previous_parallel_distance = 0.0;
    previous_perpendicular_distance = 0.0;
    previous_distance_traveled = 0.0;
    imu.set_heading(0.0);
}

void Odometry::setPose(double x, double y, double heading) {
    this->x = x;
    this->y = y;
    this->heading = heading;
    start_heading = heading;
}

void Odometry::calculateOdometry() {
    // Get the current heading
    double heading = std::fmod((360 - imu.get_heading()) + start_heading, 360);
    double headingRad = heading * (M_PI / 180);

    // Calculate the change in heading (deltaTheta)
    double deltaTheta = heading - previous_heading;

    // Normalize heading change to [-180, 180]
    if (deltaTheta < -180) {
        deltaTheta += 360;
    } else if (deltaTheta > 180) {
        deltaTheta -= 360;
    }

    // Initialize deltas for position calculation
    double deltaParallel = 0;
    double deltaPerpendicular = 0;

    // Process parallel wheel sensor
    if (parallelWheelSensor) {
        double parallelDistance = (parallelWheelSensor->get_position() / 360.0) *
                                  (M_PI * tracking_wheel_diameter); // Convert to distance
        deltaParallel = parallelDistance - previous_parallel_distance;
        previous_parallel_distance = parallelDistance;
    }

    // Process perpendicular wheel sensor
    if (perpendicularWheelSensor) {
        double perpendicularDistance = (perpendicularWheelSensor->get_position() / 360.0) *
                                      (M_PI * tracking_wheel_diameter); // Convert to distance
        deltaPerpendicular = perpendicularDistance - previous_perpendicular_distance;
        previous_perpendicular_distance = perpendicularDistance;
    }

    // Use IME if parallel or perpendicular wheel is missing
    if (!parallelWheelSensor || !perpendicularWheelSensor) {
        double averageEncoderPosition = (left_motor_group.get_position() +
                                         right_motor_group.get_position()) /
                                        2;
        double distanceTraveled = (averageEncoderPosition / 360.0) *
                                  (M_PI * wheel_diameter) * gear_ratio;

        if (!parallelWheelSensor) {
            deltaParallel = distanceTraveled - previous_distance_traveled;
        }
        if (!perpendicularWheelSensor) {
            deltaPerpendicular = distanceTraveled - previous_distance_traveled;
        }

        previous_distance_traveled = distanceTraveled;
    }

    // Calculate global position from local deltas
    double deltaX = (deltaParallel * std::cos(headingRad)) -
                    (deltaPerpendicular * std::sin(headingRad));
    double deltaY = (deltaParallel * std::sin(headingRad)) +
                    (deltaPerpendicular * std::cos(headingRad));

    // Apply wheel offsets
    deltaX += (parallelOffset * std::sin(headingRad) + perpendicularOffset * std::cos(headingRad));
    deltaY += (parallelOffset * std::cos(headingRad) - perpendicularOffset * std::sin(headingRad));

    // Update the global position
    x += deltaX;
    y += deltaY;

    // Update heading
    heading += deltaTheta;
    heading = std::fmod(heading, 360);

    // Update previous heading for the next cycle
    previous_heading = heading;
}


// #include "KrakenLib/odom.hpp"
// #include "devices.hpp"
// /**
//  * @brief Continuously update the odometry system.
//  *
//  * This function continuously updates the odometry system in a loop. It is
//  * intended to be run in a separate task from the main robot control task.
//  *
//  * @param odom The odometry system to update.
//  */
// void runOdom(Odometry &odom) {
//   while (true) {
//     odom.update();
//     pros::delay(10);
//   }
// }

// /**
//  * @brief Construct an Odometry object with an IMU and optional wheel sensors.
//  *
//  * This constructor initializes the odometry system using an IMU for heading
//  * information, and optional rotation sensors for tracking wheel displacements
//  * in parallel and perpendicular directions.
//  *
//  * @param imu The IMU used to determine the robot's heading.
//  * @param parallelWheelSensor Optional sensor for parallel wheel movement.
//  * @param perpendicularWheelSensor Optional sensor for perpendicular wheel
//  * movement.
//  */
// Odometry::Odometry(pros::Imu &imu, pros::Rotation *parallelWheelSensor,
//                    pros::Rotation *perpendicularWheelSensor,
//                    float wheel_diameter, float gear_ratio,
//                    float tracking_wheel_diameter, double parallelOffset,
//                    double perpendicularOffset)
//     : imu(imu), parallelWheelSensor(parallelWheelSensor),
//       perpendicularWheelSensor(perpendicularWheelSensor),
//       wheel_diameter(wheel_diameter), gear_ratio(gear_ratio),
//       tracking_wheel_diameter(tracking_wheel_diameter),
//       parallelOffset(parallelOffset), perpendicularOffset(perpendicularOffset) {
// }

// // Update function
// /**
//  * @brief Update the odometry system with the latest sensor data.
//  *
//  * This function is intended to be called repeatedly in a loop. It will call
//  * the appropriate calculation method based on the sensor type and update the
//  * odometry system with the latest calculated position and heading.
//  */
// void Odometry::update() {
//   // Call the appropriate calculation method based on the sensor type
//   calculateOdometry();
// }

// // Reset function

// /**
//  * @brief Resets the odometry system to its initial state.
//  *
//  * This function resets the robot's position to (0, 0) and its heading to 0.
//  * It also resets the motor group positions to 0 and the IMU to its initial
//  * state.
//  */
// void Odometry::reset() {
//   x = 0;
//   y = 0;
//   left_motor_group.set_zero_position(0);
//   right_motor_group.set_zero_position(0);
//   imu.set_heading(0);
//   start_heading = 0;
// }

// /**
//  * @brief Set the robot's position and heading.
//  *
//  * This function updates the odometry system by setting the robot's
//  * x and y coordinates as well as the starting heading.
//  *
//  * @param x The x-coordinate to set.
//  * @param y The y-coordinate to set.
//  * @param heading The starting heading to set.
//  */
// void Odometry::setPose(double x, double y, double heading) {
//   this->x = x;
//   this->y = y;
//   this->heading = heading;
//   start_heading = heading;
// }

// // Private method to calculate odometry
// /**
//  * @brief Update the odometry system with the latest sensor data.
//  *
//  * This function is called by the public update() function and should not be
//  * called directly. It calculates the robot's position and heading based on the
//  * sensor data and updates the x and y coordinates accordingly.
//  */
// // void Odometry::calculateOdometry() {
// //   bool parallelCheck = false;
// //   bool perpendicularCheck = false;
// //   // uses imu to get heading
// //   double heading = std::fmod((360 - imu.get_heading()) + start_heading, 360);
// //   double deltaTheta = heading - previous_heading;
// //   double headingRad = heading * (M_PI / 180);
// //   if (parallelWheelSensor) {
// //     // uses parallel tracking wheel to get y
// //     double parallel_distance = (parallelWheelSensor->get_position() / 360.0)
// //     *
// //                                tracking_wheel_circumference;

// //     double delta_parallel = parallel_distance - previous_parallel_distance;

// //     previous_parallel_distance = parallel_distance;
// //     parallelCheck = true;
// //     if (perpendicularWheelSensor) {
// //       double perpendicular_distance =
// //           (perpendicularWheelSensor->get_position() / 360.0) *
// //           tracking_wheel_circumference;
// //       double deltaPerpendicular =
// //           perpendicular_distance - previous_perpendicular_distance;

// //       perpendicularCheck = true;
// //     }
// //     // Use motor encoders to get x and y
// //     double average_encoder_position =
// //         (left_motor_group.get_position() + right_motor_group.get_position())
// //         / 2;
// //     double distance_traveled =
// //         (average_encoder_position / 360) * wheel_circumference * gear_ratio;

// //     double change_in_distance = distance_traveled -
// //     previous_distance_traveled;
// //     // Update X w/IME if no perpendicular wheel
// //     if (perpendicularCheck) {
// //       x += change_in_distance * std::cos(heading * (M_PI / 180));
// //     } else {
// //       x += delta_parallel * std::cos(headingRad) - deltaPerpendicular *
// //            std::sin(headingRad);
// //     }
// //     // Update Y w/IME if no parallel wheel
// //     if (parallelCheck) {
// //       y += change_in_distance * std::sin(heading * (M_PI / 180));
// //     } else {
// //       y += delta_parallel * std::sin(headingRad) + deltaPerpendicular *
// //            std::cos(headingRad);
// //     }
// //     // At the end of the loop, set previous_distance_traveled for the next
// //     loop
// //     // iteration
// //     previous_distance_traveled = distance_traveled;
// //     previous_heading = heading;

// //   }
// // }
// // void Odometry::calculateOdometry() {
// //   // Calculate heading and convert to radians

// //   double heading = std::fmod((360 - imu.get_heading()) + start_heading, 360);
// //   double headingRad = heading * (M_PI / 180);

// //   // Change in heading
// //   double deltaTheta = heading - previous_heading;

// //   // Initialize deltas
// //   double deltaParallel = 0;
// //   double deltaPerpendicular = 0;

// //   // Process parallel wheel
// //   if (parallelWheelSensor) {
// //     double parallelDistance = (parallelWheelSensor->get_position() / 360.0) *
// //                                tracking_wheel_circumference;
// //     deltaParallel = parallelDistance - previous_parallel_distance;
// //     previous_parallel_distance = parallelDistance;
// //   }

// //   // Process perpendicular wheel
// //   if (perpendicularWheelSensor) {
// //     double perpendicularDistance = (perpendicularWheelSensor->get_position()
// //     / 360.0) *
// //                                     tracking_wheel_circumference;
// //     deltaPerpendicular = perpendicularDistance -
// //     previous_perpendicular_distance; previous_perpendicular_distance =
// //     perpendicularDistance;
// //   }

// //   // Fallback to motor encoders if wheels are unavailable
// //   double distanceTraveled = 0;
// //   if (!parallelWheelSensor || !perpendicularWheelSensor) {
// //     double averageEncoderPosition = (left_motor_group.get_position() +
// //                                      right_motor_group.get_position()) / 2;
// //     distanceTraveled = (averageEncoderPosition / 360.0) *
// //                         wheel_circumference * gear_ratio;
// //     double deltaDistance = distanceTraveled - previous_distance_traveled;

// //     if(!parallelWheelSensor){
// //     deltaParallel = parallelWheelSensor ? deltaParallel : deltaDistance;
// //    }
// //    if(!perpendicularWheelSensor){
// //     deltaPerpendicular = perpendicularWheelSensor ? deltaPerpendicular :
// //     deltaDistance; previous_distance_traveled = distanceTraveled;
// //   }
// //   }

// //   // Transform local deltas to global coordinates
// //  // double deltaY = deltaParallel * std::cos(headingRad) - deltaPerpendicular
// //  * std::sin(headingRad);
// //   //double deltaX = deltaParallel * std::sin(headingRad) + deltaPerpendicular
// //   * std::cos(headingRad);

// //    double deltaY = (deltaParallel * std::cos(headingRad) - deltaPerpendicular
// //    * std::sin(headingRad)) +
// //                     (parallelOffset * std::sin(headingRad) +
// //                     perpendicularOffset * std::cos(headingRad));
// //     double deltaX = (deltaParallel * std::sin(headingRad) +
// //     deltaPerpendicular * std::cos(headingRad)) +
// //                     (parallelOffset * std::cos(headingRad) -
// //                     perpendicularOffset * std::sin(headingRad));

// //   // Update global position
// //   x += deltaX;
// //   y += deltaY;

// //   // Update previous heading
// //   previous_heading = heading;
// // }
// void Odometry::calculateOdometry() {
//   // Calculate the current heading and convert to radians
//   double heading = std::fmod((360 - imu.get_heading()) + start_heading,
//                              360); // Heading is normalized to [0, 360)
//   double headingRad = heading * (M_PI / 180); // Convert heading to radians

//   // Calculate the change in heading (deltaTheta) since the last update
//   double deltaTheta = heading - previous_heading;

//   // Normalize heading change to be within [-180, 180]
//   if (deltaTheta < -180) {
//     deltaTheta += 360;
//   } else if (deltaTheta > 180) {
//     deltaTheta -= 360;
//   }

//   // Initialize deltas for position calculation
//   double deltaParallel = 0;
//   double deltaPerpendicular = 0;

//   // Process parallel wheel if available
//   if (parallelWheelSensor) {
//     double parallelDistance =
//         (parallelWheelSensor->get_position() / 360.0) *
//         tracking_wheel_circumference; // Convert to actual distance
//     deltaParallel =
//         parallelDistance -
//         previous_parallel_distance; // Calculate change in distance for X-axis
//     previous_parallel_distance = parallelDistance;
//   }

//   // Process perpendicular wheel if available
//   if (perpendicularWheelSensor) {
//     double perpendicularDistance =
//         (perpendicularWheelSensor->get_position() / 360.0) *
//         tracking_wheel_circumference; // Convert to actual distance
//     deltaPerpendicular = perpendicularDistance -
//                          previous_perpendicular_distance; // Calculate change in
//                                                           // distance for Y-axis
//     previous_perpendicular_distance = perpendicularDistance;
//   }

//   // If only the parallel wheel is used, use IME to replace perpendicular
//   // movement (Y-axis)
//   if (parallelWheelSensor && !perpendicularWheelSensor) {
//     double averageEncoderPosition =
//         (left_motor_group.get_position() + right_motor_group.get_position()) /
//         2;
//     double distanceTraveled =
//         (averageEncoderPosition / 360.0) * wheel_circumference * gear_ratio;
//     deltaPerpendicular =
//         distanceTraveled - previous_distance_traveled; // Use IME for Y-axis

//     previous_distance_traveled = distanceTraveled;
//   }

//   // If only the perpendicular wheel is used, use IME to replace parallel
//   // movement (X-axis)
//   if (!parallelWheelSensor && perpendicularWheelSensor) {
//     double averageEncoderPosition =
//         (left_motor_group.get_position() + right_motor_group.get_position()) /
//         2;
//     double distanceTraveled =
//         (averageEncoderPosition / 360.0) * wheel_circumference * gear_ratio;
//     deltaParallel =
//         distanceTraveled - previous_distance_traveled; // Use IME for X-axis

//     previous_distance_traveled = distanceTraveled;
//   }

//   // If neither parallel nor perpendicular wheels are used, use IME for both
//   // axes
//   if (!parallelWheelSensor && !perpendicularWheelSensor) {
//     double averageEncoderPosition =
//         (left_motor_group.get_position() + right_motor_group.get_position()) /
//         2;
//     double distanceTraveled =
//         (averageEncoderPosition / 360.0) * wheel_circumference * gear_ratio;
//     deltaParallel = distanceTraveled;      // Use IME for X-axis
//     deltaPerpendicular = distanceTraveled; // Use IME for Y-axis

//     previous_distance_traveled = distanceTraveled;
//   }

//   // Update heading with deltaTheta (change in robot's orientation)
//   heading += deltaTheta;
//   heading =
//       std::fmod(heading, 360); // Normalize to keep within [0, 360) degrees
//   headingRad = heading * (M_PI / 180); // Convert to radians for math use

//   // Update the robot's position based on wheel distances
//   double deltaX =
//       (deltaParallel * std::cos(headingRad)) -
//       (deltaPerpendicular * std::sin(headingRad)); // Forward/backward motion
//   double deltaY =
//       (deltaParallel * std::sin(headingRad)) +
//       (deltaPerpendicular * std::cos(headingRad)); // Side-to-side motion

//   // Correct offsets if necessary (adjust for physical offsets between wheels)
//   deltaX += (parallelOffset * std::sin(headingRad) +
//              perpendicularOffset * std::cos(headingRad));
//   deltaY += (parallelOffset * std::cos(headingRad) -
//              perpendicularOffset * std::sin(headingRad));

//   // Update global position with the calculated changes
//   x += deltaX;
//   y += deltaY;

//   // Update previous heading for next cycle
//   previous_heading = heading;
// }
