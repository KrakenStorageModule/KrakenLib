# KrakenLib Tuning Tutorials 🐙

Welcome to the comprehensive tuning tutorials for KrakenLib! This document will provide detailed explanations of the algorithms used in the library, along with step-by-step instructions on how to tune each algorithm for optimal performance in your robotics applications.

## Table of Contents
1. [Overview of Algorithms](#overview-of-algorithms)
2. [How the Algorithms Relate to Each Other](#how-the-algorithms-relate-to-each-other)
3. [Usage Guide](#usage-guide)
4. [Tuning the Ramsete Controller](#tuning-the-ramsete-controller)
5. [Tuning the Motion Profile](#tuning-the-motion-profile)
6. [Tuning the PID Controller](#tuning-the-pid-controller)

---

## Overview of Algorithms 🐙

### 1. Ramsete Controller
The Ramsete controller is a control algorithm designed to follow a predefined path. It computes the necessary motor voltages to minimize the error between the robot's current position and the desired position along the path. The controller uses a combination of proportional, integral, and derivative gains to achieve smooth and accurate trajectory tracking.

### 2. Motion Profile
The motion profile algorithm generates a trajectory for the robot to follow, specifying how the robot should accelerate and decelerate along the path. This algorithm ensures that the robot moves smoothly, avoiding sudden changes in speed that could lead to instability.

### 3. PID Controller
The PID (Proportional-Integral-Derivative) controller is used for precise control of the robot's movements. It calculates the error between a desired setpoint and the current state, applying corrections based on proportional, integral, and derivative terms. This controller is essential for maintaining accurate positioning and speed control.

---

### How the Algorithms Relate to Each Other 🐙

The algorithms in KrakenLib are designed to work together to achieve smooth and accurate motion control in robotics. Here’s how they relate:

1. **Ramsete Controller**: This controller uses the output from the motion profile to determine how to adjust the robot's speed and direction to follow a path. It directly influences the motor voltages based on the robot's current position and the desired trajectory.
   
2. **Motion Profile**: The motion profile generates the trajectory that the Ramsete controller will follow. It specifies how the robot should accelerate and decelerate along the path, providing a smooth transition between waypoints.
   
3. **PID Controller**: The PID controller can be used to fine-tune the robot's movements by adjusting the speed and position based on real-time feedback. It can be integrated with the Ramsete controller to improve overall accuracy and stability during trajectory tracking.

By tuning these algorithms together, users can achieve optimal performance in their robotics applications, ensuring that the robot follows paths smoothly and accurately.

---

## Usage Guide 🐙

To use KrakenLib in your robotics projects, follow these steps:

1. **Include the Library**: Ensure you include the necessary headers in your project files. For example:
   ```cpp
   #include "KrakenLib/interface.hpp"
   ```

2. **Initialize Components**: Create instances of the required controllers and systems:
   ```cpp
   RamseteController ramseteController(...);
   Odometry odometry(...);
   pros::MotorGroup leftDrive(...);
   pros::MotorGroup rightDrive(...);
   ```

3. **Set Up Motion Profile**: Define the waypoints and create a motion profile:
   ```cpp
   MotionProfile motionProfile(...);
   motionProfile.addWaypoint(...);
   ```

4. **Run the Control Loop**: Use the `run` function to execute the movement routine:
   ```cpp
   run(ramseteController, odometry, leftDrive, rightDrive);
   ```

5. **Tune Parameters**: Adjust the tuning parameters for the Ramsete controller, motion profile, and PID controller as needed to optimize performance.

By following these steps, you can effectively implement the KrakenLib library in your robotics projects!

---

## Tuning the Ramsete Controller 🐙

The Ramsete controller is crucial for accurately following paths. Here’s how to tune it:

### Parameters to Tune:
- **Kp (Proportional Gain)**: Affects how aggressively the robot responds to errors.
- **Ki (Integral Gain)**: Helps eliminate steady-state errors.
- **Kd (Derivative Gain)**: Reduces overshoot and oscillations.

### Tuning Steps:
1. **Start with Initial Values**: Set Kp to a moderate value (e.g., 0.5).
2. **Increase Kp**: Gradually increase Kp until the robot responds quickly but begins to overshoot.
3. **Introduce Ki**: Add Ki to eliminate steady-state errors, starting with a small value (e.g., 0.1).
4. **Adjust Kd**: Fine-tune Kd to reduce overshoot, starting with a small value (e.g., 0.1).
5. **Test and Iterate**: Continuously test the robot's performance and adjust parameters as needed.

---

## Tuning the Motion Profile 🐙

The motion profile determines how the robot accelerates and decelerates along a path.

### Parameters to Tune:
- **Max Velocity**: The maximum speed the robot can achieve.
- **Max Acceleration**: The rate at which the robot can reach its maximum speed.

### Tuning Steps:
1. **Set Initial Values**: Start with conservative values for max velocity and acceleration.
2. **Test Performance**: Observe how the robot behaves on the desired path.
3. **Gradually Increase Max Velocity**: Increase until you observe instability or overshooting.
4. **Adjust Max Acceleration**: Ensure smooth transitions between waypoints by adjusting this value.

---

## Tuning the PID Controller 🐙

The PID controller is used for precise control of the robot's movements.

### Parameters to Tune:
- **Kp**: Proportional gain affecting the immediate response to errors.
- **Ki**: Integral gain affecting the long-term stability.
- **Kd**: Derivative gain affecting the response to the rate of error change.

### Tuning Steps:
1. **Set Initial Values**: Start with Kp set to a moderate value (e.g., 1.0).
2. **Increase Kp**: Gradually increase until you observe oscillations.
3. **Introduce Ki**: Add Ki to eliminate steady-state error, starting with a small value.
4. **Adjust Kd**: Fine-tune Kd to dampen oscillations.
5. **Iterate**: Continuously test and refine the parameters based on performance.

---

Feel free to reach out if you have any questions or need further assistance! 🐙
