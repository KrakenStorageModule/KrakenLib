# KrakenLib Tutorials

Welcome to the KrakenLib tutorials! This document provides step-by-step guides to help you understand how to use the various features of the KrakenLib library.

## Tutorial 1: Setting Up the Environment
1. **Clone the Repository**
   ```bash
   git clone https://github.com/your-repo/KrakenLib.git
   cd KrakenLib
   ```
2. **Install Dependencies**
   Make sure you have all the required dependencies installed. Refer to the `README.md` for details.

## Tutorial 2: Using the PID Controller
1. **Include the PID Header**
   ```cpp
   #include "KrakenLib/pid.hpp"
   ```
2. **Create an Instance of the PID Controller**
   ```cpp
   PID myPID(1.0, 0.1, 0.01);
   ```
3. **Update the PID Output**
   ```cpp
   float output = myPID.update(targetValue, currentValue);
   ```

## Tutorial 3: Implementing Odometry
1. **Include the Odometry Header**
   ```cpp
   #include "KrakenLib/odom.hpp"
   ```
2. **Create an Instance of the Odometry Class**
   ```cpp
   Odometry odom(imu, &parallelWheelSensor, &perpendicularWheelSensor);
   ```
3. **Track the Robot's Position**
   Call the appropriate methods to update and retrieve the robot's position.

## Tutorial 4: Motion Profiling and Splines
1. **Include the Spline Generator Header**
   ```cpp
   #include "KrakenLib/splineGen.hpp"
   ```
2. **Create and Use Splines**
   Implement the logic to generate and follow splines for smooth motion.

## Conclusion
These tutorials are designed to get you started with KrakenLib. For more detailed information, refer to the documentation in the `README.md` and the code comments throughout the library.

Happy coding! 🦑
