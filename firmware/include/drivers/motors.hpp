#pragma once

// Motor driver configuration
namespace motor_config{
    // PWM enable pins for motor speed control.
    // ENA: PWM input for Left motor speed (0-255 duty cycle)
    // ENB: PWM input for Right motor speed (0-255 duty cycle)
    constexpr int ENA_PIN = 25;
    constexpr int ENB_PIN = 13;

    // Direction control pins for Left motor (H-bridge inputs)
    // IN1/IN2 control the direction of the left motor:
    //   IN1=HIGH, IN2=LOW Forward
    //   IN1=LOW,  IN2=HIGH Reverse
    //   IN1=LOW,  IN2=LOW Coast (free spin)
    //   IN1=HIGH, IN2=HIGH Brake (hard stop)
    
    // Left motor
    constexpr int LEFT_IN1_PIN = 26;
    constexpr int LEFT_IN2_PIN = 27;

    // Right motor
    constexpr int RIGHT_IN3_PIN = 14;
    constexpr int RIGHT_IN4_PIN = 12;

    // Hardware PWM channels for ESP32
    // Each channel maps to a timer; must be unique per motor
    constexpr int LEFT_PWM_CHANNEL = 2;
    constexpr int RIGHT_PWM_CHANNEL = 3;

    // Motor direction multipliers:
    //   +1: Normal direction (positive speed = forward)
    //   -1: Reversed direction (positive speed = backward)
    // Adjust if the motor spins in the opposite direction, due to wiring or mechanical orientation
    constexpr int LEFT_MOTOR_DIR = 1;
    constexpr int RIGHT_MOTOR_DIR = 1;
}

// Motor control functions
namespace motors{

    // Initializes motor driver
    void init();

    // Sets the speed for both motors
    // Args:
    //   left_speed:  Speed for left motor (-255 to 255)
    //                Positive = forward, negative = reverse
    //                0 = stop (coast if driver allows, else brake)
    //   right_speed: Speed for right motor (-255 to 255)
    void set_speed(int left_speed, int right_speed);

    // Stops both motors immediately
    void stop();
}
