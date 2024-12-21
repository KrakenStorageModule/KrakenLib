#pragma once
#include "KrakenLib/2dmp.hpp"
#include "KrakenLib/odom.hpp"
#include "KrakenLib/point.hpp"
#include "KrakenLib/ramsete.hpp"
// Define a threshold for reaching the desired state
extern const double threshold;

// Function to check if the robot has reached the desired state
bool reachedDesiredState(const Point &currentPose, const Point &desiredState);

// Function to execute the movement routine
void run(RamseteController &ramseteController, Odometry &odometry,
         MotionProfile motionProfile, SplineGen &splineGen, bool reverse = false,
         bool clockwise = false);
