# KrakenLib: An Advanced Motion Control Library For Vex V5 Robotics

       .--.
      |o_o |
      |:_/ |
     //   \ \
    (|     | )
   /'\_   _/`\
   \___)=(___/


Welcome to the KrakenLib project! This documentation is designed to help new users understand the structure of the code, how to use the various components, and how to contribute to the project.

## Table of Contents
- [Introduction](#introduction)
- [Getting Started](#getting-started)
- [Using the PID Controller](#using-the-pid-controller)
- [Implementing Odometry](#implementing-odometry)
- [Motion Profiling and Splines](#motion-profiling-and-splines)
- [Contributing](#contributing)

## To Do List
- Implement motion profiling and spline generation.
- Integrate Ramsete controller for trajectory following.
- Develop a user interface for easier interaction.
- Add more tutorials and examples in the documentation.
- Test and refine existing features.

## Introduction
KrakenLib is a robotics library designed to facilitate the development of autonomous robot systems. It provides various tools and classes to handle motion, control, and sensor integration.

## Getting Started
To get started with KrakenLib, clone the repository and set up your development environment. Ensure you have the necessary dependencies installed.

```bash
git clone https://github.com/your-repo/KrakenLib.git
cd KrakenLib
```

## Using the PID Controller
The PID controller is implemented in the `pid.hpp` and `pid.cpp` files. You can create an instance of the PID controller and use it to manage the robot's movements based on error values.

### Example Usage
```cpp
PID myPID(1.0, 0.1, 0.01);
float output = myPID.update(targetValue, currentValue);
```

## Implementing Odometry
The odometry system is designed to track the robot's position and heading using various sensors. Refer to the `odom.hpp` for details on how to initialize and use the odometry class.

## Motion Profiling and Splines
This section will cover how to implement motion profiling and spline generation for smooth movement paths.

## Contributing
We welcome contributions! Please fork the repository and submit a pull request with your changes. Ensure your code is well-documented and tested.

Happy coding! 🦑
