#include <Arduino.h>
#include "drivers/encoders.hpp"

// Cumulative tick counts for each encoder
// Positive = forward rotation, negative = reverse rotation
volatile long left_ticks = 0;
volatile long right_ticks = 0;

// Previous encoder states (2-bit values: [A][B])
// Used to detect state transitions for direction calculation
volatile uint8_t left_prev_state = 0;
volatile uint8_t right_prev_state = 0;

// Maps [previous_state][current_state] to tick delta (+1, -1, or 0)
// For a standard quadrature encoder:
//   - +1: Clockwise rotation
//   - -1: Counter-clockwise rotation
//   - 0:  No change or invalid transition
// States are encoded as 2-bit values:
//   00 = 0, 01 = 1, 10 = 2, 11 = 3
const int8_t quad_table[4][4] = {
  { 0, +1, -1,  0 },
  { -1, 0,  0, +1 },
  { +1, 0,  0, -1 },
  { 0, -1, +1,  0 }
};

// Reads the current 2-bit state of the left encoder
inline uint8_t read_left_state(){
    return (digitalRead(encoder_config::LEFT_A_PIN) << 1) | digitalRead(encoder_config::LEFT_B_PIN);
}

// Reads the current 2-bit state of the left encoder
inline uint8_t read_right_state(){
    return (digitalRead(encoder_config::RIGHT_A_PIN) << 1) | digitalRead(encoder_config::RIGHT_B_PIN);
}

// Interrupt Service Routines (ISR)
// Left encoder ISR: Triggered on any change (RISING or FALLING) of A or B pins
// IRAM_ATTR: Stores ISR in RAM (not flash) for faster execution on ESP32
//            Critical for minimizing interrupt latency
void IRAM_ATTR left_encoder_isr() {
    uint8_t curr = read_left_state(); // Read current state
    int8_t delta = quad_table[left_prev_state][curr]; // Look up tick delta
    
    left_ticks += delta; // Update tick count
    left_prev_state = curr; // Save state for next
}

// Right encoder ISR: Triggered on any change (RISING or FALLING) of A or B pins
// IRAM_ATTR: Stores ISR in RAM (not flash) for faster execution on ESP32
//            Critical for minimizing interrupt latency
void IRAM_ATTR right_encoder_isr() {
    uint8_t curr = read_right_state();
    int8_t delta = quad_table[right_prev_state][curr];

    right_ticks += delta;
    right_prev_state = curr;
}



// Initialize encoder pins and attach interrupts
void encoders::init() {

    // Configure all encoder pins as inputs with pull-up resistors
    pinMode(encoder_config::LEFT_A_PIN, INPUT_PULLUP);
    pinMode(encoder_config::LEFT_B_PIN, INPUT_PULLUP);
    pinMode(encoder_config::RIGHT_A_PIN, INPUT_PULLUP);
    pinMode(encoder_config::RIGHT_B_PIN, INPUT_PULLUP);

    // Initialize previous states
    left_prev_state = read_left_state();
    right_prev_state = read_right_state();

    // Attach interrupts to all encoder pins
    // CHANGE: Trigger ISR on any edge (RISING or FALLING)
    attachInterrupt(encoder_config::LEFT_A_PIN, left_encoder_isr, CHANGE);
    attachInterrupt(encoder_config::LEFT_B_PIN, left_encoder_isr, CHANGE);
    attachInterrupt(encoder_config::RIGHT_A_PIN, right_encoder_isr, CHANGE);
    attachInterrupt(encoder_config::RIGHT_B_PIN, right_encoder_isr, CHANGE);
}

// Returns the left encoder tick count, scaled by direction
// noInterrupts()/interrupts(): Temporarily disables/enables all interrupts
//                              to safely read shared variables
long encoders::get_left_ticks() {
    noInterrupts();
    long ticks = left_ticks * encoder_config::LEFT_DIR; // Apply direction sign
    interrupts();
    return ticks;
}

// Returns the right encoder tick count, scaled by direction
// Same logic as get_left_ticks
long encoders::get_right_ticks() {
    noInterrupts();
    long ticks = right_ticks * encoder_config::RIGHT_DIR; // Apply direction sign
    interrupts();
    return ticks;
}

// Resets both encoder tick counters to zero
// noInterrupts()/interrupts(): Ensures atomic reset of both counters
void encoders::reset() {
    noInterrupts();
    left_ticks = 0;
    right_ticks = 0;
    interrupts();
}
