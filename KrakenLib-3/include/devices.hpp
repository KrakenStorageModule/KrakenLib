#pragma once

#include "pros/adi.hpp"
#include "pros/misc.hpp"
#include "pros/motors.hpp"
#include "pros/imu.hpp"
#include "pros/rotation.hpp"
#include "pros/motor_group.hpp"

// ADI Devices (pistons)
extern pros::adi::DigitalOut mogo1;
extern pros::adi::DigitalOut mogo2;
extern pros::adi::DigitalOut wallArm1;
extern pros::adi::DigitalOut wallArm2;
extern pros::adi::DigitalOut intakeLift;

// Motors
extern pros::Motor intakeFront;
extern pros::Motor intakeHood;

// Sensors
extern pros::Imu imu;
extern pros::Rotation horizontal_encoder;
extern pros::Rotation vertical_encoder;



//drivetrain motor groups
extern pros::MotorGroup left_motor_group;
extern pros::MotorGroup right_motor_group;

// State Variables
extern bool mogoToggle;
extern bool wallToggle;
extern bool intakePistonToggle;
extern bool intakeState;
extern bool autonRan;

// Odometry Variables
extern float xValue;
extern float yValue;
extern float thetaValue;

// Controller
extern pros::Controller controller;
extern double avgTempLeft;
extern double avgTempRight;
extern int avgTempTotal;
extern std::string tempReturn;


// Function Declarations
void controllerHud();