#include "devices.hpp"
#include "KrakenLib/pid.hpp"
#include "auton.hpp"
#include "pros/imu.hpp"

pros::Controller controller(pros::E_CONTROLLER_MASTER);
double avgTempLeft = 0;
double avgTempRight = 0;
int avgTempTotal = 0;
std::string tempReturn = " ";

// PID Testing
PID lbPID(1, 2, 3);

void pidTest() {
  while (lbPID.deadband > lbPID.error) {
    lbArm.move_voltage(lbPID.update(90, lbArmTrack.get_position()));
  }
}

// ADI Devices (pistons)

// Motor Groups
pros::MotorGroup left_motor_group({1, 2, 3});
pros::MotorGroup right_motor_group({4, 5, 6});

//motors
pros::Motor intakeFront(8);
pros::Motor intakeHook(9);
pros::Motor lbArm(10);
// sensors
pros::Imu imu(2);
pros::Rotation parallelTracker(1);
pros::Rotation perpedicularTracker(3);
pros::Rotation lbArmTrack(4);
pros::Optical vision(5);

/// colorsort
void colorsort() {
  // RED

  if ((color.compare("red")) < 0) {
    if ((color.compare("red")) < 0) {
      if ((220 >= vision.get_hue()) && (200 <= vision.get_hue())) {
        // kicks out the wrong color ring
        intakeHook.move_voltage(9000);
      }
    }
    // BLUE
    if ((color.compare("blue")) < 0) {
      if ((350 >= vision.get_hue()) && (10 <= vision.get_hue())) {
        // kicks out the wrong color ring
        intakeHook.move_voltage(9000);
      }
    }
  }
}

// intake driver control -> integrate colorsort later on
void intakeControl() {
  
  if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
    intakeFront.move_voltage(12000);
    intakeHook.move_voltage(12000);
  } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
    intakeFront.move_voltage(-12000);
    intakeHook.move_voltage(-12000);
  } else {
    intakeFront.move_voltage(0);
    intakeHook.move_voltage(0);
  }
}

// Temp Display Code
void controllerHud() {
  while (true) {
    // Averaging each dt half
    avgTempLeft = (left_motor_group.get_temperature(0) +
                   left_motor_group.get_temperature(1) +
                   left_motor_group.get_temperature(2)) /
                  3;

    avgTempRight = (right_motor_group.get_temperature(0) +
                    right_motor_group.get_temperature(1) +
                    right_motor_group.get_temperature(2)) /
                   3;
    // Convert to F while averaging both sides
    avgTempTotal = int((((avgTempLeft + avgTempRight) * 1.8) / 2) + 32);
    // Convert to string and display
    tempReturn = std::to_string(avgTempTotal);

    pros::delay(100);
  }
}