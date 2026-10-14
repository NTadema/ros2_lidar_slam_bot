#include <Arduino.h>
#include <cmath>
#include <algorithm>

#include "drivers/wheel_pi.hpp"

namespace
{
    constexpr float KP = 400.0f;
    constexpr float KI = 120.0f;

    constexpr float PWM_MIN = -255.0f;
    constexpr float PWM_MAX = 255.0f;

    constexpr float ZERO_TARGET_MPS = 0.001f;

    struct PIController
    {
        float integral = 0.0f;

        int update(float target_mps, float measured_mps, float dt_s)
        {
            // A zero target means stop this wheel and clear
            // accumulated integral output.
            if (std::fabs(target_mps) < ZERO_TARGET_MPS)
            {
                integral = 0.0f;
                return 0;
            }

            const float error = target_mps - measured_mps;

            const float proportional = KP * error;

            const float proposed_integral =
                integral + KI * error * dt_s;

            const float proposed_output =
                proportional + proposed_integral;

            // Conditional integration:
            // Integrate normally unless doing so would drive
            // an already saturated output further into saturation.
            const bool saturated_high =
                proposed_output > PWM_MAX && error > 0.0f;

            const bool saturated_low =
                proposed_output < PWM_MIN && error < 0.0f;

            if (!saturated_high && !saturated_low)
            {
                integral = std::clamp(
                    proposed_integral,
                    PWM_MIN,
                    PWM_MAX);
            }

            const float output = std::clamp(
                proportional + integral,
                PWM_MIN,
                PWM_MAX);

            return static_cast<int>(std::lround(output));
        }

        void reset()
        {
            integral = 0.0f;
        }
    };

    PIController left_controller;
    PIController right_controller;
}

void wheel_pi::init()
{
    reset();
}

void wheel_pi::reset()
{
    left_controller.reset();
    right_controller.reset();
}

wheel_pi::MotorOutput wheel_pi::update(
    float target_left_mps,
    float target_right_mps,
    float measured_left_mps,
    float measured_right_mps,
    float dt_s)
{
    MotorOutput output;

    // Reject invalid measurements or timing.
    if (!std::isfinite(target_left_mps) ||
        !std::isfinite(target_right_mps) ||
        !std::isfinite(measured_left_mps) ||
        !std::isfinite(measured_right_mps) ||
        !std::isfinite(dt_s) ||
        dt_s < 0.005f ||
        dt_s > 0.050f)
    {
        reset();
        return output;
    }

    output.left_pwm = left_controller.update(
        target_left_mps,
        measured_left_mps,
        dt_s);

    output.right_pwm = right_controller.update(
        target_right_mps,
        measured_right_mps,
        dt_s);

    return output;
}
