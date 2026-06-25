#pragma once

// L298N pins
#define L298N_ENA_PIN 25
#define L298N_ENB_PIN 13
#define L298N_LEFT_IN1_PIN 26
#define L298N_LEFT_IN2_PIN 27
#define L298N_RIGHT_IN3_PIN 14
#define L298N_RIGHT_IN4_PIN 12

// PWM channels
#define L298N_LEFT_PWM_CHANNEL 2
#define L298N_RIGHT_PWM_CHANNEL 3

// Define motor direction
#define LEFT_MOTOR_DIR 1
#define RIGHT_MOTOR_DIR 1

// Speed settings
void set_motors_speed(int left_speed, int right_speed);

// Motor pin definitions
void motors_init();

// Motor control functions
void stop_motors();
