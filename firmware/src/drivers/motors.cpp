#include <Arduino.h>
#include "drivers/motors.h"

// Initialize motor control pins and PWM
void motors::init() {
    pinMode(motor_config::LEFT_IN1_PIN, OUTPUT);
    pinMode(motor_config::LEFT_IN2_PIN, OUTPUT);
    pinMode(motor_config::RIGHT_IN3_PIN, OUTPUT);
    pinMode(motor_config::RIGHT_IN4_PIN, OUTPUT);

    // Setup PWM channels
    ledcSetup(motor_config::LEFT_PWM_CHANNEL, 5000, 8);
    ledcSetup(motor_config::RIGHT_PWM_CHANNEL, 5000, 8);

    ledcAttachPin(motor_config::ENA_PIN, motor_config::LEFT_PWM_CHANNEL);
    ledcAttachPin(motor_config::ENB_PIN, motor_config::RIGHT_PWM_CHANNEL);
}

void motors::set_speed(int left_speed, int right_speed) {

    left_speed = constrain(left_speed, -255, 255);
    right_speed = constrain(right_speed, -255, 255);

    left_speed *= motor_config::LEFT_MOTOR_DIR;
    right_speed *= motor_config::RIGHT_MOTOR_DIR;


    // Left motor
    if (left_speed > 0) {        // forward
        digitalWrite(motor_config::LEFT_IN1_PIN, LOW);
        digitalWrite(motor_config::LEFT_IN2_PIN, HIGH);
    } else if (left_speed < 0) { // backward
        digitalWrite(motor_config::LEFT_IN1_PIN, HIGH);
        digitalWrite(motor_config::LEFT_IN2_PIN, LOW);
        left_speed = -left_speed;
    } else {
        digitalWrite(motor_config::LEFT_IN1_PIN, LOW);
        digitalWrite(motor_config::LEFT_IN2_PIN, LOW);
    }

    // Right motor
    if (right_speed > 0) {       // forward
        digitalWrite(motor_config::RIGHT_IN3_PIN, LOW);
        digitalWrite(motor_config::RIGHT_IN4_PIN, HIGH);
    } else if (right_speed < 0) { // backward
        digitalWrite(motor_config::RIGHT_IN3_PIN, HIGH);
        digitalWrite(motor_config::RIGHT_IN4_PIN, LOW);
        right_speed = -right_speed;
    } else {
        digitalWrite(motor_config::RIGHT_IN3_PIN, LOW);
        digitalWrite(motor_config::RIGHT_IN4_PIN, LOW);
    }

    ledcWrite(motor_config::LEFT_PWM_CHANNEL, left_speed);
    ledcWrite(motor_config::RIGHT_PWM_CHANNEL, right_speed);
}

// Stop the robot
void motors::stop(){
    set_speed(0, 0);
}
