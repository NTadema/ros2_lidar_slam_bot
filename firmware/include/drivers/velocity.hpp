#pragma once

#include "drivers/encoders.hpp"

namespace wheel_velocity
{
    struct VelocityData
    {
        float left_mps = 0.0f;
        float right_mps = 0.0f;
        float dt_s = 0.0f;
        bool valid = false;
    };

    // Initialize the velocity estimator.
    void init();

    // Calculate wheel velocities from a new encoder measurement.
    // The first measurement establishes a baseline and is invalid.
    VelocityData update(const encoders::EncoderData& encoder_data);

    // Reset the estimator's measurement history.
    void reset();
}
