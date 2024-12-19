#include "devices.hpp"
#include "KrakenLib/pid.hpp"
#include "auton.hpp"
#include "pros/imu.hpp"
#include "pros/misc.h"

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

// motors
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

/**
 * @brief Runs an anti-jam loop on the intake hook motor to clear any
 * stuck rings.
 *
 * This function will run indefinitely until the robot is turned off.
 *
 * @note This function is not thread-safe and should not be run in a
 * separate thread. It should be run in a task, such as a periodic
 * task.
 */


//DRIVER CONTROL CODE
void pneumaticDriverControl() {

  if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
    mogoToggle = !mogoToggle;
    mogo1.set_value(mogoToggle);
    mogo2.set_value(mogoToggle);
  }
  if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
    doinkerToggle = !doinkerToggle;
    doinker.set_value(doinkerToggle);
  }
  if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
    rushClampToggle = !rushClampToggle;
    rushClamp.set_value(rushClampToggle);
  }
  if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
    intakePistonToggle = !intakePistonToggle;
    intakeLift.set_value(intakePistonToggle);
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

void lbArmControl() {
  //disengaged
  if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
   while (lbPID.deadband > lbPID.error) {
    lbArm.move_voltage(lbPID.update(0, lbArmTrack.get_position()));
  }
  }
  //loading
  if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)){
   while (lbPID.deadband > lbPID.error) {
    lbArm.move_voltage(lbPID.update(30, lbArmTrack.get_position()));
  }
  }
  //scoring
  if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){
   while (lbPID.deadband > lbPID.error) {
    lbArm.move_voltage(lbPID.update(110, lbArmTrack.get_position()));
  }
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