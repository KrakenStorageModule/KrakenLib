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

## 🐙 Tuning the Ramsete Controller 🐙

The Ramsete controller is a powerful tool for following paths smoothly and accurately. Tuning the Ramsete controller involves adjusting its parameters to achieve the desired responsiveness and stability during path following. Here’s how to effectively tune the Ramsete controller for your robotics project.

### 🐙 Key Parameters

1. **b (Gain)**:
   - This parameter determines how aggressively the robot tries to follow the path. A higher value will make the robot respond more quickly to deviations from the desired trajectory.
   - **Typical Range**: Start with values around 2.0 to 5.0 and adjust based on performance.

2. **zeta (Damping Ratio)**:
   - The damping ratio affects how oscillatory the response is. A value closer to 1 will reduce overshoot and oscillations, leading to a smoother response.
   - **Typical Range**: Values between 0.5 and 1.0 are common. Experiment with these values to find the right balance for your robot.

### 🐙 Steps for Tuning the Ramsete Controller

1. **Define the Path**:
   - Create a set of waypoints that define the desired path. Ensure that the waypoints are spaced appropriately to allow for smooth movement.

2. **Initial Parameter Setup**:
   - Set initial values for `b` and `zeta`. A good starting point is `b = 3.0` and `zeta = 0.8`.

3. **Run Tests**:
   - Execute a series of test runs where the robot follows the defined path. Observe its behavior, including how well it stays on the path and how it reacts to changes in direction.

4. **Adjust `b`**:
   - If the robot is slow to respond to deviations, increase the `b` value. If it overshoots or oscillates too much, decrease the `b` value.
   - **Tip**: Make small adjustments (e.g., 0.5 increments) and test after each change.

5. **Adjust `zeta`**:
   - If the robot is oscillating or overshooting, increase `zeta` to dampen the response. If the robot feels sluggish, decrease `zeta`.
   - **Tip**: Again, make small adjustments and observe the effects on performance.

6. **Monitor Feedback**:
   - Use sensors (like encoders or gyroscopes) to monitor the robot's position and orientation. This feedback can help you understand how well the robot is following the path and where adjustments may be needed.

7. **Iterate**:
   - Continue to iterate on the `b` and `zeta` values based on the test results. Document the changes you make and their effects on performance to help refine your tuning process.

8. **Final Testing**:
   - Once you have achieved satisfactory performance, conduct thorough testing in various scenarios to ensure the robot behaves consistently.



---

## 🐙 Tuning the Motion Profile 🐙

The motion profile is crucial for controlling the robot's movement along a defined path. Proper tuning of the motion profile ensures smooth acceleration and deceleration, minimizing jerk and improving overall performance. Here’s how to effectively tune the motion profile for your robotics project.

### 🐙 Key Parameters

1. **Max Velocity**:
   - This parameter defines the maximum speed at which the robot can travel along the path. Setting this value too high may lead to instability, while setting it too low can slow down the robot unnecessarily.
   - **Typical Range**: Start with a value that matches the capabilities of your robot's motors and adjust based on testing.

2. **Max Acceleration**:
   - This parameter determines how quickly the robot can accelerate to its maximum speed. Proper tuning of acceleration helps to reduce the risk of tipping and improves control during rapid movements.
   - **Typical Range**: Similar to max velocity, choose an acceleration that aligns with your robot's design and capabilities.

3. **Jerk Limit**:
   - Jerk is the rate of change of acceleration. Limiting jerk helps to create smoother transitions between acceleration and deceleration, reducing mechanical stress and improving control.
   - **Typical Range**: Set a jerk limit based on your robot's mechanical design and the responsiveness you desire.

### 🐙 Steps for Tuning the Motion Profile

1. **Initial Parameter Setup**:
   - Start with conservative values for max velocity, max acceleration, and jerk limit. For example, set max velocity to 100 units, max acceleration to 50 units/s², and jerk limit to 20 units/s³.

2. **Run Test Profiles**:
   - Execute a series of test runs using the defined motion profile. Observe how the robot accelerates, travels, and decelerates along the path.

3. **Adjust Max Velocity**:
   - If the robot feels sluggish or does not reach the desired speed, gradually increase the max velocity. Monitor for stability and control during these adjustments.

4. **Adjust Max Acceleration**:
   - If the robot is tipping or losing control during acceleration, decrease the max acceleration. Conversely, if it accelerates too slowly, increase the value.

5. **Tune Jerk Limit**:
   - If the robot exhibits jerky movements or oscillations, consider reducing the jerk limit. This will smooth out the transitions between acceleration and deceleration.

6. **Monitor Performance**:
   - Use sensors to monitor the robot's speed, acceleration, and position during tests. Adjust parameters based on real-time feedback to improve performance.

7. **Iterate**:
   - Continue to iterate on the max velocity, max acceleration, and jerk limit based on the test results. Document the changes you make and their effects on performance to help refine your tuning process.

8. **Final Testing**:
   - Once you have achieved satisfactory performance, conduct thorough testing in various scenarios to ensure the robot behaves consistently under different conditions.

---

## 🐙 Tuning PID Controllers 🐙

PID (Proportional, Integral, Derivative) controllers are essential for controlling the behavior of your robot. Proper tuning of these controllers ensures that your robot responds accurately and smoothly to changes in its environment. Here’s how to effectively tune PID controllers for your robotics project.

### 🐙 Key Parameters

1. **Kp (Proportional Gain)**:
   - This parameter determines the reaction to the current error. A higher `Kp` value results in a stronger response to the error, which can reduce the time it takes to reach the desired setpoint.
   - **Typical Range**: Start with a value around 0.5 and adjust based on performance.

2. **Ki (Integral Gain)**:
   - This parameter accounts for past errors. It integrates the error over time and helps eliminate steady-state errors. However, too high a value can lead to instability and oscillations.
   - **Typical Range**: Start with a small value, such as 0.1, and increase it gradually.

3. **Kd (Derivative Gain)**:
   - This parameter predicts future errors based on the rate of change of the error. It helps dampen the system's response, reducing overshoot and oscillations. 
   - **Typical Range**: Start with a small value, such as 0.1, and adjust as needed.

### 🐙 Steps for Tuning PID Controllers

1. **Start with Kp**:
   - Start with a value of 0 for Kp and Kd
   - Raise Kp until the robot starts to oscilate around the target

2. **Increase Kd**:
   - Cycle kd up until there is no oscillation

3. **Iterate**:
   - Increase kP until the robot starts to oscilate around the target
   - Cycle kd up until there is no oscillation
   - Repeat this until kd can no longer compensate for kP, and revert to the last working values
   - Expect kd to be roughly 6 times kP

4. **Add in ki**



### 🐙 Practical Tips

- **Use a Step Response Test**: Apply a step input to the system and observe how it responds. This can help you visualize the effects of your tuning adjustments.
- **Monitor System Behavior**: Use sensors to monitor the system's response during tests. Look for overshoot, settling time, and steady-state error.
- **Be Patient**: Tuning PID controllers can be a trial-and-error process. Take your time and make incremental adjustments to avoid destabilizing the system.

### 🐙 Conclusion

Tuning PID controllers is a critical step in optimizing your robot's performance. By understanding the key parameters and following the outlined steps, you can ensure that your robot responds accurately and smoothly to changes in its environment. 🐙🐙🐙

---



Feel free to reach out if you have any questions or need further assistance! 🐙
(Discord: twokrakensandabiscuit. Youtube: https://www.youtube.com/channel/UCB5k3LFcfnKPgs1FRq7SPzw. Github: https://github.com/Twokrakensandabiscuit)
