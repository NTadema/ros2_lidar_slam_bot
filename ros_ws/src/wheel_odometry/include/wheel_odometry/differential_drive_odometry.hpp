// Prevent multiple inclusion of this header file
#ifndef DIFFERENTIAL_DRIVE_ODOMETRY_HPP
#define DIFFERENTIAL_DRIVE_ODOMETRY_HPP

#include <cstdint>

// Implements differential drive odometry calculations for a robot
class DifferentialDriveOdometry
{
public:
    DifferentialDriveOdometry(double wheel_radius, double wheel_separation);

    // Updates the robot's pose based on the left and right wheel positions and delta time
    void update(double left_wheel_position, double right_wheel_position, double dt);

    // Returns the current x position of the robot in meters
    double x() const;
    double y() const;

    // Returns the current orientation (yaw) of the robot in radians
    double yaw() const;

    // Returns the current linear and angular velocities of the robot
    double linearVelocity() const;
    double angularVelocity() const;

    bool isInitialized() const;

private:
    // Robot geometry parameters
    double wheel_radius_;      // Radius of the wheels in meters
    double wheel_separation_;  // Distance between the wheels in meters

    // Robot wheel positions
    double last_left_wheel_position_;   // Last recorded left wheel position in radians
    double last_right_wheel_position_;  // Last recorded right wheel position in radians

    // Robot pose
    double x_;    // Current x position in meters
    double y_;    // Current y position in meters
    double yaw_;  // Current orientation (yaw) in radians

    // Robot velocities
    double linear_velocity_;   // Current linear velocity in meters per second
    double angular_velocity_;  // Current angular velocity in radians per second

    bool initialized_;  // Flag indicating if the odometry has been initialized
};

#endif  // DIFFERENTIAL_DRIVE_ODOMETRY_HPP
