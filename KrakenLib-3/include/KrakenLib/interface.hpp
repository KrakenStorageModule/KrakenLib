#pragma once
#include "2dmp.hpp"
#include "KrakenLib/ramsete.hpp"
#include "KrakenLib/odom.hpp"
#include "KrakenLib/point.hpp"
// Define a threshold for reaching the desired state
extern const double threshold;

// Function to check if the robot has reached the desired state
bool reachedDesiredState(const Point& currentPose, const Point& desiredState);

// Function to execute the movement routine
void run(RamseteController& ramseteController, 
         Odometry& odometry, MotionProfile,
         bool reverse = false,
         bool clockwise = false);

