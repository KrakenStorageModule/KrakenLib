#ifndef ODOM_HPP
#define ODOM_HPP
#include <cmath>
#include "pros/rotation.hpp"
#include "pros/imu.hpp"
/**
 * @brief Class to track the robot's position and heading using encoders, a gyro, and
 * optional tracking wheels.
 *
 * The Odometry class uses the data from a gyro and optional tracking wheels to calculate
 * the position and heading of the robot. The class uses a Kalman filter to smooth the data
 * and give the most accurate readings. The class also provides functions to reset the
 * odometry and set the initial pose of the robot.
 *
 * @param imu The imu object to use for getting the gyro data.
 * @param parallelWheelSensor The rotation sensor object to use for getting the
 * parallel wheel data. Set to nullptr to disable.
 * @param perpendicularWheelSensor The rotation sensor object to use for getting the
 * perpendicular wheel data. Set to nullptr to disable.
 * @param wheel_diameter The diameter of the wheel in inches.
 * @param gear_ratio The gear ratio of the robot's gear box.
 * @param tracking_wheel_diameter The diameter of the tracking wheel in inches.
 */
class Odometry {
public:
    // Constructor with optional parameters
    Odometry(pros::Imu &imu, 
             pros::Rotation *parallelWheelSensor = nullptr, 
             pros::Rotation *perpendicularWheelSensor = nullptr,
             float wheel_diameter = 3.25,                  
             float gear_ratio = (36.0 / 48.0),
             float tracking_wheel_diameter = 2.75,
             double parallelOffset = 0.0,
             double perpendicularOffset = 0.0);

    // Member functions
    void update();
    void reset();
    double getX() const { return x; }
    double getY() const { return y; }
    double getHeading() const { return heading; }
    void setPose();
    private:
    pros::Imu &imu;
    pros::Rotation *parallelWheelSensor; // Pointer to allow for optional sensor
    pros::Rotation *perpendicularWheelSensor; // Pointer to allow for optional sensor

    double x = 0; // Front/back position
    double y = 0; // Left/right position
    double previous_parallel_distance = 0;
    double previous_perpendicular_distance = 0;
    double gear_ratio = ((double)36 / 48); // CHANGE THIS TO CURRENT GEAR RATIO
    double wheel_diameter = 3.25; // Wheel Size
    double wheel_circumference = M_PI * wheel_diameter;
    double tracking_wheel_diameter = 2.75;
    double tracking_wheel_circumference = M_PI * tracking_wheel_diameter;
    double start_heading = 0;
    double heading = 0;
    double previous_distance_traveled = 0;
    double parallelOffset = 0.0;
    double perpendicularOffset = 0.0;
    double previous_heading = 0;
    double deltaTheta = 0;
    double deltaParallel = 0;
    double deltaPerpendicular = 0;
    double headingRad = 0;
    void calculateOdometry();
    void setPose(double x, double y, double heading);
};

void runOdom(Odometry &odom);
#endif // ODOM_HPP