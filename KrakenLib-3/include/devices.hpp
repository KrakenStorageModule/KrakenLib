#pragma once

#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"
#include "pros/optical.hpp"

// ADI Devices (pistons)
extern pros::adi::DigitalOut mogo1;
extern pros::adi::DigitalOut mogo2;
extern pros::adi::DigitalOut intakeLift;
extern pros::adi::DigitalOut doinker;
extern pros::adi::DigitalOut rushClamp;

// Motors
extern pros::Motor intakeFront;
extern pros::Motor intakeHook;
extern pros::Motor lbArm;

// Sensors
extern pros::Imu imu;
extern pros::Rotation parallelTracker;
extern pros::Rotation perpedicularTracker;
extern pros::Rotation lbArmTrack;
extern pros::Optical vision;

// drivetrain motor groups
extern pros::MotorGroup left_motor_group;
extern pros::MotorGroup right_motor_group;

// State Variables
extern bool mogoToggle;
extern bool doinkerToggle;
extern bool rushClampToggle;
extern bool intakePistonToggle;
extern bool autonRan;



// Controller
extern pros::Controller controller;
extern double avgTempLeft;
extern double avgTempRight;
extern int avgTempTotal;
extern std::string tempReturn;

// Function Declarations
void controllerHud();
void PIDTest();
void pneumaticDriverControl();
void lbArmControl();
void antijam();