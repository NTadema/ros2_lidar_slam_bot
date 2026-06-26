#pragma once

// Motor driver configuration
namespace motor_config{
    // Motor driver pins
    constexpr int ENA_PIN = 25;
    constexpr int ENB_PIN = 13;
    constexpr int LEFT_IN1_PIN = 26;
    constexpr int LEFT_IN2_PIN = 27;
    constexpr int RIGHT_IN3_PIN = 14;
    constexpr int RIGHT_IN4_PIN = 12;
    // PWM channels
    constexpr int LEFT_PWM_CHANNEL = 2;
    constexpr int RIGHT_PWM_CHANNEL = 3;
    // Define motor direction
    constexpr int LEFT_MOTOR_DIR = 1;
    constexpr int RIGHT_MOTOR_DIR = 1;
}

// Motor control functions
namespace motors{
    void init();
    void set_speed(int left_speed, int right_speed);
    void stop();
}
