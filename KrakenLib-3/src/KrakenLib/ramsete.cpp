// #include "ramsete.hpp"

// RamseteController::RamseteController(double k1, double k2, double k3, double L, double max_velocity, double max_voltage)
//     : k1(k1), k2(k2), k3(k3), L(L), max_velocity(max_velocity), max_voltage(max_voltage) {}

// void RamseteController::update(const Point& currentPose, const Point& desiredState) {
//     double e_x = desiredState.x - currentPose.x; // Error in x
//     double e_y = desiredState.y - currentPose.y; // Error in y
//     double e_theta = normalizeAngle(desiredState.theta - currentPose.theta); // Error in heading

//     double v_d = std::hypot(e_x, e_y); // Desired linear velocity
//     double omega_d = k3 * e_theta + k2 * v_d * sinc(e_theta) * e_y; // Desired angular velocity

//     double u1 = -k1 * e_x; // Control input for velocity
//     double u2 = -k3 * e_theta - k2 * v_d * sinc(e_theta) * e_y; // Control input for turning rate

//     double v = v_d * cos(currentPose.theta) + u1; // Actual linear velocity
//     double omega = omega_d + u2; // Actual angular velocity

//     v_L = v - (L / 2.0) * omega; // Left wheel velocity
//     v_R = v + (L / 2.0) * omega; // Right wheel velocity

//     saturateVelocities();
// }

// void RamseteController::applyMotorVoltages(pros::MotorGroup& leftDrive, pros::MotorGroup& rightDrive) {
//     leftDrive.move_voltage(static_cast<int>(velocityToVoltage(v_L)));
//     rightDrive.move_voltage(static_cast<int>(velocityToVoltage(v_R)));
// }

// void RamseteController::saturateVelocities() {
//     // Ensure the velocities do not exceed the maximum limits
//     if (std::abs(v_L) > max_velocity) {
//         v_L = (v_L > 0) ? max_velocity : -max_velocity;
//     }
//     if (std::abs(v_R) > max_velocity) {
//         v_R = (v_R > 0) ? max_velocity : -max_velocity;
//     }
// }

// double RamseteController::normalizeAngle(double angle) {
//     // Normalize the angle to be within -pi to pi
//     while (angle > M_PI) angle -= 2 * M_PI;
//     while (angle < -M_PI) angle += 2 * M_PI;
//     return angle;
// }

// double RamseteController::velocityToVoltage(double velocity) const {
//     // Convert velocity to motor voltage (this is a placeholder)
//     return (velocity / max_velocity) * max_voltage;
// }

// // Define the sinc function
// double sinc(double x) {
//     if (x == 0) {
//         return 1.0; // sinc(0) = 1
//     }
//     return std::sin(x) / x; // sinc(x) = sin(x)/x
// }

// #include "ramsete.hpp"
// #include "devices.hpp"

// RamseteController::RamseteController(double k1, double k2, double k3, double L, double max_velocity)
//     : k1(k1), k2(k2), k3(k3), L(L), max_velocity(max_velocity), max_voltage(12000), v_L(0), v_R(0) {}

// void RamseteController::update(const Point& currentPose, const Point& desiredState) {
//     double e_x = desiredState.x - currentPose.x;
//     double e_y = desiredState.y - currentPose.y;
//     double e_theta = normalizeAngle(desiredState.theta - currentPose.theta);

//     // Transform errors to the robot's local frame
//     double e_x_local = cos(currentPose.theta) * e_x + sin(currentPose.theta) * e_y;
//     double e_y_local = -sin(currentPose.theta) * e_x + cos(currentPose.theta) * e_y;

//     double v_d = std::hypot(desiredState.x, desiredState.y); // Desired linear velocity
//     double omega_d = desiredState.theta; // Desired angular velocity

//     // Ramsete control laws
//     double v = v_d * cos(e_theta) + k1 * e_x_local;
//     double omega = omega_d + k2 * v_d * sinc(e_theta) * e_y_local + k3 * e_theta;

//     // Compute wheel velocities
//     v_L = v - (L / 2.0) * omega;
//     v_R = v + (L / 2.0) * omega;

//     saturateVelocities();
// }

// void RamseteController::applyMotorVoltages() {
//     left_motor_group.move_voltage(static_cast<int>(velocityToVoltage(v_L)));
//     right_motor_group.move_voltage(static_cast<int>(velocityToVoltage(v_R)));
// }

// void RamseteController::saturateVelocities() {
//     double max_wheel_velocity = std::max(std::abs(v_L), std::abs(v_R));
//     if (max_wheel_velocity > max_velocity) {
//         double scale = max_velocity / max_wheel_velocity;
//         v_L *= scale;
//         v_R *= scale;
//     }
// }

// double RamseteController::normalizeAngle(double angle) {
//     while (angle > M_PI) angle -= 2 * M_PI;
//     while (angle < -M_PI) angle += 2 * M_PI;
//     return angle;
// }

// double RamseteController::velocityToVoltage(double velocity) const {
//     return (velocity / max_velocity) * max_voltage;
// }

// double RamseteController::sinc(double x) const {
//     if (std::abs(x) < 1e-6) return 1.0; // Avoid numerical instability
//     return std::sin(x) / x;
// }
#include "ramsete.hpp"
#include "devices.hpp"

RamseteController::RamseteController(double k1, double k2, double k3, double L, double max_velocity)
    : k1(k1), k2(k2), k3(k3), L(L), max_velocity(max_velocity), max_voltage(12000), v_L(0), v_R(0) {}

void RamseteController::update(const Point& currentPose, const Point& desiredState) {
    double e_x = desiredState.x - currentPose.x;
    double e_y = desiredState.y - currentPose.y;
    double e_theta = normalizeAngle(desiredState.theta - currentPose.theta);

    // Transform errors to the robot's local frame
    double e_x_local = cos(currentPose.theta) * e_x + sin(currentPose.theta) * e_y;
    double e_y_local = -sin(currentPose.theta) * e_x + cos(currentPose.theta) * e_y;

    // Compute desired linear and angular velocities
    double v_d = std::hypot(e_x_local, e_y_local); // Linear velocity towards the target
    double omega_d = desiredState.theta; // Desired angular velocity (heading)

    // Ramsete control laws
    double v = v_d * cos(e_theta) + k1 * e_x_local;
    double omega = omega_d + k2 * e_y_local + k3 * e_theta; // Adjust omega_d for heading error

    // Compute wheel velocities
    v_L = v - (L / 2.0) * omega;
    v_R = v + (L / 2.0) * omega;

    saturateVelocities();
}

void RamseteController::applyMotorVoltages() {
    left_motor_group.move_voltage(static_cast<int>(velocityToVoltage(v_L)));
    right_motor_group.move_voltage(static_cast<int>(velocityToVoltage(v_R)));
}

void RamseteController::saturateVelocities() {
    double max_wheel_velocity = std::max(std::abs(v_L), std::abs(v_R));
    if (max_wheel_velocity > max_velocity) {
        double scale = max_velocity / max_wheel_velocity;
        v_L *= scale;
        v_R *= scale;
    }
}

double RamseteController::normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}

double RamseteController::velocityToVoltage(double velocity) const {
    return (velocity / max_velocity) * max_voltage;
}

double RamseteController::sinc(double x) const {
    if (std::abs(x) < 1e-6) return 1.0; // Avoid numerical instability
    return std::sin(x) / x;
}
