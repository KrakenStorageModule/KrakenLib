# KrakenLib Tutorials 🐙

Welcome to the KrakenLib tutorials! This document provides step-by-step guides on how to use the various features of the KrakenLib motion control library for VEX robotics.

## Table of Contents
1. [Getting Started](#getting-started)
2. [Using the Ramsete Controller](#using-the-ramsete-controller)
3. [Creating a Motion Profile](#creating-a-motion-profile)
4. [Implementing PID Control](#implementing-pid-control)
5. [Tuning Algorithms](#tuning-algorithms)
6. [Example Usage: Test Autonomous Routine](#example-usage-test-autonomous-routine)

---

## Getting Started 🐙
To get started with KrakenLib, ensure you have the following prerequisites:
1. A VEX robotics kit.
2. PROS development environment set up.
3. Basic knowledge of C++ programming.

### Installation 🐙
1. Clone the repository:
   ```bash
   git clone https://github.com/yourusername/KrakenLib.git
   ```
2. Navigate to the project directory:
   ```bash
   cd KrakenLib
   ```
3. Open the project in your preferred IDE.

## Using the Ramsete Controller 🐙
The Ramsete controller is essential for following paths accurately. Here’s how to implement it:

### Implementation Steps:
1. **Include the Header**: Ensure you include the necessary header in your project:
   ```cpp
   #include "KrakenLib/interface.hpp"
   ```
2. **Initialize the Controller**: Create an instance of the Ramsete controller:
   ```cpp
   RamseteController ramseteController(...);
   ```
3. **Update the Controller**: Use the `update` method to adjust the controller based on the current pose and desired state:
   ```cpp
   ramseteController.update(currentPose, desiredState);
   ```

## Creating a Motion Profile 🐙
The motion profile defines how the robot should move along a path. Here’s how to create one:

### Implementation Steps:
1. **Include the Header**: Include the necessary header:
   ```cpp
   #include "KrakenLib/interface.hpp"
   ```
2. **Initialize the Motion Profile**: Create an instance of the motion profile:
   ```cpp
   MotionProfile motionProfile(...);
   ```
3. **Add Waypoints**: Define waypoints for the robot to follow:
   ```cpp
   motionProfile.addWaypoint(...);
   ```

## Implementing PID Control 🐙
The PID controller is used for precise control of the robot’s movements. Here’s how to implement it:

### Implementation Steps:
1. **Include the Header**: Include the necessary header:
   ```cpp
   #include "KrakenLib/pid.hpp"
   ```
2. **Initialize the PID Controller**: Create an instance of the PID controller:
   ```cpp
   PIDController pidController(...);
   ```
3. **Update the PID Controller**: Use the `update` method to adjust the controller based on the current error:
   ```cpp
   pidController.update(currentError);
   ```

## Tuning Algorithms 🐙
Refer to the [Tuning Tutorials](docs/Tuning_Tutorials.md) for detailed instructions on how to tune each algorithm for optimal performance in your robotics applications.

## Example Usage: Test Autonomous Routine 🐙

To illustrate how to interact with the KrakenLib library, we will use the `test` autonomous routine found in `auton.cpp`. This example demonstrates how to set up and execute an autonomous routine using the various components of the library.

### Code Example:
```cpp
#include "auton.hpp"
#include "KrakenLib/2dmp.hpp"
#include "devices.hpp"
#include "KrakenLib/odom.hpp"
#include "pros/rtos.hpp"
#include "pros/llemu.hpp"
#include "KrakenLib/ramsete.hpp"
#include "KrakenLib/splineGen.hpp"
#include "KrakenLib/point.hpp"
#include "KrakenLib/interface.hpp"

// Declare odom object here
Odometry odom(imu, &parallelTracker, &perpedicularTracker, 3.25, .75, 2.75);

// Declare RAMSETE Controller
RamseteController ramseteController(0.5, 0.5, 0.5, 3.25, 4.0, 12.0);

// Declare Spline Generation Tool
SplineGen splineGen;

// Auton Subsystem Functions
void autonIntake(string state) {
    // Implementation for intake control
}

// Test Autonomous Routine
void test() {
    // Initialize components
    // Example waypoints and motion profile setup
    MotionProfile motionProfile;
    motionProfile.addWaypoint(Point(0, 0, 0));
    motionProfile.addWaypoint(Point(12, 0, 0));

    // Execute the movement routine
    run(ramseteController, odom, leftDrive, rightDrive);
}
```

### Explanation:
1. **Include Necessary Headers**: Ensure all required headers are included to access the library's functionality.
2. **Initialize Components**: Create instances of `Odometry`, `RamseteController`, and `SplineGen` to manage the robot's position and path.
3. **Define Autonomous Functions**: Implement any necessary autonomous functions, such as `autonIntake`, to control subsystems.
4. **Set Up the Autonomous Routine**: In the `test` function, define waypoints and create a motion profile for the robot to follow.
5. **Run the Routine**: Call the `run` function with the initialized components to execute the autonomous routine.

This example serves as a template for how to structure your autonomous routines using KrakenLib. Adjust the waypoints and control logic as needed for your specific application.

---

Feel free to reach out if you have any questions or need further assistance! 🐙
