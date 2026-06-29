#include <Arduino.h>
#include "drivers/encoders.h"

// Global variables to store encoder ticks
volatile long left_ticks = 0;
volatile long right_ticks = 0;

volatile uint8_t left_prev_state = 0;
volatile uint8_t right_prev_state = 0;

// quadrature lookup table
const int8_t quad_table[4][4] = {
  { 0, +1, -1,  0 },
  { -1, 0,  0, +1 },
  { +1, 0,  0, -1 },
  { 0, -1, +1,  0 }
};

// Read the current state of the left encoder
inline uint8_t read_left_state(){
    return (digitalRead(encoder_config::LEFT_A_PIN) << 1) | digitalRead(encoder_config::LEFT_B_PIN);
}

// Read the current state of the right encoder
inline uint8_t read_right_state(){
    return (digitalRead(encoder_config::RIGHT_A_PIN) << 1) | digitalRead(encoder_config::RIGHT_B_PIN);
}

// Interrupt Service Routines (ISR)
void IRAM_ATTR left_encoder_isr() {
    uint8_t curr = read_left_state();
    int8_t delta = quad_table[left_prev_state][curr];
    
    left_ticks += delta;
    left_prev_state = curr;
}

// Interrupt Service Routines (ISR)
void IRAM_ATTR right_encoder_isr() {
    uint8_t curr = read_right_state();
    int8_t delta = quad_table[right_prev_state][curr];

    right_ticks += delta;
    right_prev_state = curr;
}



// Initialize encoder pins and attach interrupts
void encoders::init() {
    // Set encoder pins as input
    pinMode(encoder_config::LEFT_A_PIN, INPUT_PULLUP);
    pinMode(encoder_config::LEFT_B_PIN, INPUT_PULLUP);
    pinMode(encoder_config::RIGHT_A_PIN, INPUT_PULLUP);
    pinMode(encoder_config::RIGHT_B_PIN, INPUT_PULLUP);

    // Initialize previous states
    left_prev_state = read_left_state();
    right_prev_state = read_right_state();

    // Attach interrupts for encoder A pins
    attachInterrupt(encoder_config::LEFT_A_PIN, left_encoder_isr, CHANGE);
    attachInterrupt(encoder_config::LEFT_B_PIN, left_encoder_isr, CHANGE);
    attachInterrupt(encoder_config::RIGHT_A_PIN, right_encoder_isr, CHANGE);
    attachInterrupt(encoder_config::RIGHT_B_PIN, right_encoder_isr, CHANGE);
}

// Get the number of ticks for the left encoder
long encoders::get_left_ticks() {
    noInterrupts();
    long ticks = left_ticks * encoder_config::LEFT_DIR;
    interrupts();
    return ticks;
}

// Get the number of ticks for the right encoder
long encoders::get_right_ticks() {
    noInterrupts();
    long ticks = right_ticks * encoder_config::RIGHT_DIR;
    interrupts();
    return ticks;
}

// Reset encoder ticks
void encoders::reset() {
    noInterrupts();
    left_ticks = 0;
    right_ticks = 0;
    interrupts();
}
