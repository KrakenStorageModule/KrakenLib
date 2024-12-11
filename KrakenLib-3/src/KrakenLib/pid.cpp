#include "KrakenLib/pid.hpp"

/**
 * @brief Constructor for the PID class
 * @param kP The proportional gain
 * @param kI The integral gain
 * @param kD The derivative gain
 */
PID::PID(float kP, float kI, float kD) 
    : kP(kP), 
      kI(kI), 
      kD(kD), 
      integral(0), 
      prevError(0) {
    // Initialize PID gains and reset internal state
}

/**
 * @brief Update the PID output based on the current error
 * 
 * This function calculates the PID output using the current error value.
 * It updates the integral and derivative components based on the error
 * and returns the computed output.
 *
 * @param target The target value
 * @param currentPos The current position
 * @return The updated PID output
 */
float PID::update(float target, float currentPos) {
    error = target - currentPos;
    // Calculate integral term
    integral += error;

    // Calculate derivative term
    const float derivative = error - prevError;
    prevError = error;

    // Compute and return the PID output
    return (error * kP) + (integral * kI) + (derivative * kD);
}

/**
 * @brief Reset the PID controller
 * 
 * This function resets the internal state of the PID controller,
 * including the integral and previous error terms. This can be
 * useful when starting a new control process or when the system
 * needs to be re-initialized.
 */
void PID::reset() {
    integral = 0;      // Reset integral to zero
    prevError = 0;     // Reset previous error to zero
}