#include "devices.hpp"
pros::Controller controller(pros::E_CONTROLLER_MASTER);
double avgTempLeft = 0;
double avgTempRight = 0;
int avgTempTotal = 0;
std::string tempReturn = " ";

// Autonomous Selector Variables
int selectedAuton = 0;
bool autonSelected = false;

// Motor Groups
pros::MotorGroup left_motor_group({1, 2, 3});
pros::MotorGroup right_motor_group({4, 5, 6});

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