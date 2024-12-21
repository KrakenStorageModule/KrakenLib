#pragma once
#include <cmath> // Include cmath for fabs()

/**
 * @brief Class for PID control
 * @details Class for PID control using proportional, integral, and derivative
 * components.
 */
class PID {
public:
  /**
   * @brief Constructor for the PID class
   * @param kP The proportional gain
   * @param kI The integral gain
   * @param kD The derivative gain
   */
  PID(float kP, float kI, float kD);

  /**
   * @brief Update the PID output based on the current error
   * @param error The current error
   * @return The updated PID output
   */
  float update(float target, float currentPos);

  /**
   * @brief Reset the PID
   */
  void reset();

  float deadband = 1;
  float error = 0;

protected:
  // Member variables
  float kP;
  float kI;
  float kD;
  float integral = 0;
  float prevError = 0;
  float prevIntegral = 0;
};