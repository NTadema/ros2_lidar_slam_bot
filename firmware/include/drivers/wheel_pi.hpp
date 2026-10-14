#pragma once

namespace wheel_pi
{
    struct MotorOutput
    {
        int left_pwm = 0;
        int right_pwm = 0;
    };

    void init();

    MotorOutput update(
        float target_left_mps,
        float target_right_mps,
        float measured_left_mps,
        float measured_right_mps,
        float dt_s);

    void reset();
}
