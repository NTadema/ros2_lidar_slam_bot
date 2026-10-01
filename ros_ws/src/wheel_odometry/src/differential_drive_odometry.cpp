#include "wheel_odometry/differential_drive_odometry.hpp"

#include <cmath>

// Constructor that initializes the odometry with wheel radius and separation
DifferentialDriveOdometry::DifferentialDriveOdometry(double wheel_radius, double wheel_separation)
    : wheel_radius_(wheel_radius),
      wheel_separation_(wheel_separation),
      last_left_wheel_position_(0.0),
      last_right_wheel_position_(0.0),
      x_(0.0),
      y_(0.0),
      yaw_(0.0),
      linear_velocity_(0.0),
      angular_velocity_(0.0),
      initialized_(false)
{
}

void DifferentialDriveOdometry::update(double left_wheel_position, double right_wheel_position, double dt)
{   
    // Initialize the odometry state on the first update call
    if (!initialized_)
    {
        last_left_wheel_position_ = left_wheel_position;
        last_right_wheel_position_ = right_wheel_position;
        initialized_ = true;
        return;
    }

    // Prevent division by zero when calculating velocities
    if (dt <= 0.0)
    {
        return;
    }

    // Calculate the change in wheel positions
    const double delta_left_wheel_position = left_wheel_position - last_left_wheel_position_;
    const double delta_right_wheel_position = right_wheel_position - last_right_wheel_position_;

    // Convert wheel rotations to linear distances traveled by each wheel
    const double delta_left_distance = delta_left_wheel_position * wheel_radius_;
    const double delta_right_distance = delta_right_wheel_position * wheel_radius_;

    // Calculate the average distance traveled
    const double delta_distance = (delta_left_distance + delta_right_distance) / 2.0;

    // Calculate the change in orientation (yaw) based on wheel distances
    const double delta_yaw = (delta_right_distance - delta_left_distance) / wheel_separation_;

    // Halfway orientation (yaw) for better accuracy in position update
    const double halfway_yaw = yaw_ + delta_yaw / 2.0;

    // Update robot position
    x_ += delta_distance * std::cos(halfway_yaw);
    y_ += delta_distance * std::sin(halfway_yaw);

    // Update robot orientation (yaw)
    yaw_ += delta_yaw;

    // Calculate robot linear and angular velocities
    linear_velocity_ = delta_distance / dt;
    angular_velocity_ = delta_yaw / dt;

    // Store the current wheel positions for the next update
    last_left_wheel_position_ = left_wheel_position;
    last_right_wheel_position_ = right_wheel_position;
}

// Accessor methods to retrieve the current robot pose and velocities
double DifferentialDriveOdometry::x() const
{
    return x_;
}

double DifferentialDriveOdometry::y() const
{
    return y_;
}

double DifferentialDriveOdometry::yaw() const
{
    return yaw_;
}

double DifferentialDriveOdometry::linearVelocity() const
{
    return linear_velocity_;
}

double DifferentialDriveOdometry::angularVelocity() const
{
    return angular_velocity_;
}

bool DifferentialDriveOdometry::isInitialized() const
{
    return initialized_;
}







