#ifndef ODOMETRY_HPP
#define ODOMETRY_HPP

#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/motors.hpp"
#include <cmath>
#include "pros/rotation.hpp"
class Odometry {
public:
    /**
     * @brief Construct an Odometry object with an IMU and optional wheel sensors.
     * 
     * @param imu The IMU used to determine the robot's heading.
     * @param parallelWheelSensor Optional sensor for parallel wheel movement.
     * @param perpendicularWheelSensor Optional sensor for perpendicular wheel movement.
     * @param wheel_diameter The diameter of the robot's wheels (in inches).
     * @param gear_ratio The gear ratio between the motor and the wheels.
     * @param tracking_wheel_diameter The diameter of the tracking wheels.
     * @param parallelOffset The offset for the parallel wheel sensor.
     * @param perpendicularOffset The offset for the perpendicular wheel sensor.
     */
    Odometry(pros::Imu &imu, pros::Rotation *parallelWheelSensor,
             pros::Rotation *perpendicularWheelSensor, float wheel_diameter,
             float gear_ratio, float tracking_wheel_diameter,
             double parallelOffset = 0.0, double perpendicularOffset = 0.0);

    /**
     * @brief Update the odometry system with the latest sensor data.
     */
    void update();

    /**
     * @brief Reset the odometry system to its initial state.
     */
    void reset();

    /**
     * @brief Set the robot's position and heading.
     * 
     * @param x The x-coordinate to set.
     * @param y The y-coordinate to set.
     * @param heading The heading to set.
     */
    void setPose(double x, double y, double heading);

    /**
     * @brief Get the current x-coordinate of the robot.
     * @return The x-coordinate.
     */
    double getX() const { return x; }

    /**
     * @brief Get the current y-coordinate of the robot.
     * @return The y-coordinate.
     */
    double getY() const { return y; }

    /**
     * @brief Get the current heading of the robot.
     * @return The heading in degrees.
     */
    double getHeading() const { return heading; }

private:
    /**
     * @brief Internal method to calculate the robot's position and heading.
     */
    void calculateOdometry();

    pros::Imu &imu;
    pros::Rotation *parallelWheelSensor;
    pros::Rotation *perpendicularWheelSensor;
    float wheel_diameter;
    float gear_ratio;
    float tracking_wheel_diameter;
    double parallelOffset;
    double perpendicularOffset;

    double x = 0.0;
    double y = 0.0;
    double heading = 0.0;
    double previous_heading = 0.0;
    double previous_parallel_distance = 0.0;
    double previous_perpendicular_distance = 0.0;
    double previous_distance_traveled = 0.0;
    double start_heading = 0.0;
};

#endif // ODOMETRY_HPP



// #ifndef ODOM_HPP
// #define ODOM_HPP
// #include "pros/imu.hpp"
// #include "pros/rotation.hpp"
// #include <cmath>

// /**
//  * @brief Class to track the robot's position and heading using encoders, a
//  * gyro, and optional tracking wheels.
//  *
//  * The Odometry class uses the data from a gyro and optional tracking wheels to
//  * calculate the position and heading of the robot. The class uses a Kalman
//  * filter to smooth the data and give the most accurate readings. The class also
//  * provides functions to reset the odometry and set the initial pose of the
//  * robot.
//  *
//  * @param imu The imu object to use for getting the gyro data.
//  * @param parallelWheelSensor The rotation sensor object to use for getting the
//  * parallel wheel data. Set to nullptr to disable.
//  * @param perpendicularWheelSensor The rotation sensor object to use for getting
//  * the perpendicular wheel data. Set to nullptr to disable.
//  * @param wheel_diameter The diameter of the wheel in inches.
//  * @param gear_ratio The gear ratio of the robot's gear box.
//  * @param tracking_wheel_diameter The diameter of the tracking wheel in inches.
//  */
// class Odometry {
// public:
//   // Constructor with optional parameters
//   Odometry(pros::Imu &imu, pros::Rotation *parallelWheelSensor = nullptr,
//            pros::Rotation *perpendicularWheelSensor = nullptr,
//            float wheel_diameter = 3.25, float gear_ratio = (36.0 / 48.0),
//            float tracking_wheel_diameter = 2.75, double parallelOffset = 0.0,
//            double perpendicularOffset = 0.0);

//   // Member functions
//   void update();
//   void reset();
//   double getX() const { return x; }
//   double getY() const { return y; }
//   double getHeading() const { return heading; }
//   void setPose();

// private:
//   pros::Imu &imu;
//   pros::Rotation *parallelWheelSensor; // Pointer to allow for optional sensor
//   pros::Rotation
//       *perpendicularWheelSensor; // Pointer to allow for optional sensor

//   double x = 0; // Front/back position
//   double y = 0; // Left/right position
//   double previous_parallel_distance = 0;
//   double previous_perpendicular_distance = 0;
//   double gear_ratio = ((double)36 / 48); // CHANGE THIS TO CURRENT GEAR RATIO
//   double wheel_diameter = 3.25;          // Wheel Size
//   double wheel_circumference = M_PI * wheel_diameter;
//   double tracking_wheel_diameter = 2.75;
//   double tracking_wheel_circumference = M_PI * tracking_wheel_diameter;
//   double start_heading = 0;
//   double heading = 0;
//   double previous_distance_traveled = 0;
//   double parallelOffset = 0.0;
//   double perpendicularOffset = 0.0;
//   double previous_heading = 0;
//   double deltaTheta = 0;
//   double deltaParallel = 0;
//   double deltaPerpendicular = 0;
//   double headingRad = 0;
//   void calculateOdometry();
//   void setPose(double x, double y, double heading);
// };

// void runOdom(Odometry &odom);
// #endif // ODOM_HPP