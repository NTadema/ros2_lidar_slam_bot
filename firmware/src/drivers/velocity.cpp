#include <Arduino.h>
#include <cstdint>

#include "drivers/velocity.hpp"

namespace
{
    // Verified effective encoder resolution.
    constexpr float COUNTS_PER_REV = 2024.0f;

    // Wheel radius in metres.
    constexpr float WHEEL_RADIUS_M = 0.0325f;

    // Distance travelled per decoded encoder count.
    constexpr float METRES_PER_COUNT =
        (2.0f * PI * WHEEL_RADIUS_M) / COUNTS_PER_REV;

    // Reject intervals too short for reliable timing.
    constexpr uint32_t MIN_INTERVAL_US = 1000;

    // Do not calculate velocity over excessively long intervals.
    // A longer interval establishes a new baseline instead.
    constexpr uint32_t MAX_INTERVAL_US = 1000000;

    encoders::EncoderData previous_data{};
    bool initialized = false;
}

void wheel_velocity::init()
{
    previous_data = encoders::read();
    initialized = true;
}

void wheel_velocity::reset()
{
    initialized = false;
    previous_data = {};
}

wheel_velocity::VelocityData wheel_velocity::update(
    const encoders::EncoderData& encoder_data)
{
    VelocityData result{};

    // Establish the first measurement baseline.
    if (!initialized)
    {
        previous_data = encoder_data;
        initialized = true;
        return result;
    }

    // Unsigned subtraction handles micros() wrapping around.
    const uint32_t elapsed_us =
        encoder_data.timestamp_us - previous_data.timestamp_us;

    // Calculate count differences using wider integers.
    const int64_t delta_left =
        static_cast<int64_t>(encoder_data.left_ticks) -
        static_cast<int64_t>(previous_data.left_ticks);

    const int64_t delta_right =
        static_cast<int64_t>(encoder_data.right_ticks) -
        static_cast<int64_t>(previous_data.right_ticks);

    // Always advance the baseline, including after invalid intervals.
    previous_data = encoder_data;

    if (elapsed_us < MIN_INTERVAL_US ||
        elapsed_us > MAX_INTERVAL_US)
    {
        return result;
    }

    const float dt_s = elapsed_us * 1.0e-6f;

    result.left_mps =
        static_cast<float>(delta_left) * METRES_PER_COUNT / dt_s;

    result.right_mps =
        static_cast<float>(delta_right) * METRES_PER_COUNT / dt_s;

    result.dt_s = dt_s;
    result.valid = true;

    return result;
}
