// #ifndef RAMSETE_HPP
// #define RAMSETE_HPP

// #include <cmath>
// #include "pros/motor_group.hpp"
// #include "point.hpp" // Include the Point struct
// class RamseteController {
// public:
//     RamseteController(double k1, double k2, double k3, double L, double max_velocity, double max_voltage);

//     void update(const Point& currentPose, const Point& desiredState);
//     void applyMotorVoltages(pros::MotorGroup& leftDrive, pros::MotorGroup& rightDrive);
//     double sinc(double x);
// private:
//     double k1, k2, k3; // Gains for the controller
//     double L;          // Track width
//     double max_velocity, max_voltage; // Limits
//     double v_L, v_R;  // Wheel velocities

//     void saturateVelocities();
//     double normalizeAngle(double angle);
//     double velocityToVoltage(double velocity) const;
// };

// #endif // RAMSETE_HPP
#ifndef RAMSETE_HPP
#define RAMSETE_HPP

#include <cmath>
#include "point.hpp"

class RamseteController {
public:
    RamseteController(double k1, double k2, double k3, double L, double max_velocity);

    void update(const Point& currentPose, const Point& desiredState);
    void applyMotorVoltages();

private:
    double k1, k2, k3;       // Gains for the controller
    double L;                // Track width
    double max_velocity;     // Maximum velocity (m/s)
    double max_voltage = 12000;      // Maximum voltage (mV)
    double v_L, v_R;         // Wheel velocities
    
    void saturateVelocities();
    double normalizeAngle(double angle);
    double velocityToVoltage(double velocity) const;
    double sinc(double x) const; // Use const since this doesn't modify the object
};

#endif // RAMSETE_HPP
