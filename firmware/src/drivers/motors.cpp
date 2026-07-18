#include <Arduino.h>
#include "drivers/motors.hpp"

// Initialize motor control pins and PWM
void motors::init() {
    pinMode(motor_config::LEFT_IN1_PIN, OUTPUT);
    pinMode(motor_config::LEFT_IN2_PIN, OUTPUT);
    pinMode(motor_config::RIGHT_IN3_PIN, OUTPUT);
    pinMode(motor_config::RIGHT_IN4_PIN, OUTPUT);

    // Setup PWM channels
    ledcSetup(motor_config::LEFT_PWM_CHANNEL, 5000, 8);
    ledcSetup(motor_config::RIGHT_PWM_CHANNEL, 5000, 8);

    // Attach enable pins to PWM channels
    ledcAttachPin(motor_config::ENA_PIN, motor_config::LEFT_PWM_CHANNEL);
    ledcAttachPin(motor_config::ENB_PIN, motor_config::RIGHT_PWM_CHANNEL);
}

// Sets the speed for both left and right motors
// Args:
//   left_speed:  Desired speed for left motor (-255 to 255)
//   right_speed: Desired speed for right motor (-255 to 255)
// Note:
//   - Negative values = reverse direction
//   - Zero = stop (coast mode: motors free to spin)
void motors::set_speed(int left_speed, int right_speed) {

    // Clamp speed values to valid range [-255, 255]
    left_speed = constrain(left_speed, -255, 255);
    right_speed = constrain(right_speed, -255, 255);
    
    // Apply direction multipliers
    left_speed *= motor_config::LEFT_MOTOR_DIR;
    right_speed *= motor_config::RIGHT_MOTOR_DIR;


    // Left motor
    if (left_speed > 0) {         // Forward: IN1=LOW, IN2=HIGH
        digitalWrite(motor_config::LEFT_IN1_PIN, LOW);
        digitalWrite(motor_config::LEFT_IN2_PIN, HIGH);
    } else if (left_speed < 0) { // Reverse: IN1=HIGH, IN2=LOW
        digitalWrite(motor_config::LEFT_IN1_PIN, HIGH);
        digitalWrite(motor_config::LEFT_IN2_PIN, LOW);
        left_speed = -left_speed; // Convert to positive for PWM
    } else {                      // Stop: IN1=LOW, IN2=LOW
        digitalWrite(motor_config::LEFT_IN1_PIN, LOW);
        digitalWrite(motor_config::LEFT_IN2_PIN, LOW);
    }

    // Same logic as left motor, but for right motor
    if (right_speed > 0) {
        digitalWrite(motor_config::RIGHT_IN3_PIN, LOW);
        digitalWrite(motor_config::RIGHT_IN4_PIN, HIGH);
    } else if (right_speed < 0) {
        digitalWrite(motor_config::RIGHT_IN3_PIN, HIGH);
        digitalWrite(motor_config::RIGHT_IN4_PIN, LOW);
        right_speed = -right_speed;
    } else {
        digitalWrite(motor_config::RIGHT_IN3_PIN, LOW);
        digitalWrite(motor_config::RIGHT_IN4_PIN, LOW);
    }

    // Set PWM duty cycle for both motors
    ledcWrite(motor_config::LEFT_PWM_CHANNEL, left_speed);
    ledcWrite(motor_config::RIGHT_PWM_CHANNEL, right_speed);
}

// Stops both motors immediately
void motors::stop(){
    set_speed(0, 0);
}
