#include "KrakenLib/odom.hpp"
#include "devices.hpp"
/**
 * @brief Continuously update the odometry system.
 *
 * This function continuously updates the odometry system in a loop. It is
 * intended to be run in a separate task from the main robot control task.
 *
 * @param odom The odometry system to update.
 */
void runOdom(Odometry &odom) {
  while (true) {
    odom.update();
    pros::delay(10);
  }
}

/**
 * @brief Construct an Odometry object with an IMU and optional wheel sensors.
 *
 * This constructor initializes the odometry system using an IMU for heading
 * information, and optional rotation sensors for tracking wheel displacements
 * in parallel and perpendicular directions.
 *
 * @param imu The IMU used to determine the robot's heading.
 * @param parallelWheelSensor Optional sensor for parallel wheel movement.
 * @param perpendicularWheelSensor Optional sensor for perpendicular wheel
 * movement.
 */
Odometry::Odometry(pros::Imu &imu, pros::Rotation *parallelWheelSensor,
                   pros::Rotation *perpendicularWheelSensor,
                   float wheel_diameter, float gear_ratio,
                   float tracking_wheel_diameter)
    : imu(imu), parallelWheelSensor(parallelWheelSensor),
      perpendicularWheelSensor(perpendicularWheelSensor),
      wheel_diameter(wheel_diameter), gear_ratio(gear_ratio),
      tracking_wheel_diameter(tracking_wheel_diameter) {}

// Update function
/**
 * @brief Update the odometry system with the latest sensor data.
 *
 * This function is intended to be called repeatedly in a loop. It will call
 * the appropriate calculation method based on the sensor type and update the
 * odometry system with the latest calculated position and heading.
 */
void Odometry::update() {
  // Call the appropriate calculation method based on the sensor type
  calculateOdometry();
}

// Reset function

/**
 * @brief Resets the odometry system to its initial state.
 *
 * This function resets the robot's position to (0, 0) and its heading to 0.
 * It also resets the motor group positions to 0 and the IMU to its initial
 * state.
 */
void Odometry::reset() {
  x = 0;
  y = 0;
  left_motor_group.set_zero_position(0);
  right_motor_group.set_zero_position(0);
  imu.set_pitch(0);
  imu.set_roll(0);
  imu.set_yaw(0);
  start_heading = 0;
}

/**
 * @brief Set the robot's position and heading.
 *
 * This function updates the odometry system by setting the robot's
 * x and y coordinates as well as the starting heading.
 *
 * @param x The x-coordinate to set.
 * @param y The y-coordinate to set.
 * @param heading The starting heading to set.
 */
void Odometry::setPose(double x, double y, double heading) {
  this->x = x;
  this->y = y;
  start_heading = heading;
}



// Private method to calculate odometry
/**
 * @brief Update the odometry system with the latest sensor data.
 *
 * This function is called by the public update() function and should not be
 * called directly. It calculates the robot's position and heading based on the
 * sensor data and updates the x and y coordinates accordingly.
 */
void Odometry::calculateOdometry() {
  //resets imes
  left_motor_group.set_zero_position(0);
  right_motor_group.set_zero_position(0);
  //uses imu to get heading
  double imuConversion = (imu.get_heading() * 180) / M_PI;
  double heading = std::fmod((360 - imuConversion) + start_heading, 360);
  if (parallelWheelSensor) {
    //uses parallel tracking wheel to get y
    double parallel_distance = (parallelWheelSensor->get_position() / 360.0) *
                               tracking_wheel_circumference;

    double delta_parallel = parallel_distance - previous_parallel_distance;
    y += delta_parallel * std::sin(heading * (M_PI / 180));
    previous_parallel_distance = parallel_distance;
  }
  if (perpendicularWheelSensor) {
    double perpendicular_distance =
        (perpendicularWheelSensor->get_position() / 360.0) *
        tracking_wheel_circumference;
    double delta_perpendicular =
        perpendicular_distance - previous_perpendicular_distance;
    x += delta_perpendicular * std::cos(heading * (M_PI / 180));
    previous_perpendicular_distance = perpendicular_distance;
  } else {
    // Use motor encoders to get x and y
    double average_encoder_position =
        (left_motor_group.get_position() + right_motor_group.get_position()) /
        2;
    double distance_traveled =
        (average_encoder_position / 360) * wheel_circumference * gear_ratio;

    double change_in_distance = distance_traveled - previous_distance_traveled;

    x += change_in_distance * std::cos(heading * (M_PI / 180));
    y += change_in_distance * std::sin(heading * (M_PI / 180));

    // At the end of the loop, set previous_distance_traveled for the next loop
    // iteration
    previous_distance_traveled = distance_traveled;
  }
}