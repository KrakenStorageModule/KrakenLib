#pragma once
#include "KrakenLib/ramsete.hpp"
#include "KrakenLib/odom.hpp"
#include "KrakenLib/point.hpp"
#include "KrakenLib/2dmp.hpp"
// Define a threshold for reaching the desired state
extern const double threshold;

// Function to check if the robot has reached the desired state
bool reachedDesiredState(const Point& currentPose, const Point& desiredState);

// Function to execute the movement routine
void run(RamseteController& ramseteController, 
         Odometry& odometry, 
         pros::MotorGroup& leftDrive, 
         pros::MotorGroup& rightDrive,
         bool reverse = false,
         bool clockwise = false);

