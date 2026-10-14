#include <Arduino.h>
#include "drivers/encoders.hpp"

// Synchronization for encoder counters and quadrature states.
// Protects shared data between encoder ISRs and application code.
static portMUX_TYPE encoder_mux = portMUX_INITIALIZER_UNLOCKED;

// Cumulative raw tick counts.
// Direction multipliers are applied when readings are returned.
static volatile int32_t left_ticks = 0;
static volatile int32_t right_ticks = 0;

// Previous encoder states: [A][B], encoded as 00, 01, 10, 11.
static volatile uint8_t left_prev_state = 0;
static volatile uint8_t right_prev_state = 0;

// Quadrature transition table.
// Rows = previous state; columns = current state.
static const int8_t quad_table[4][4] = {
    {  0, +1, -1,  0 },
    { -1,  0,  0, +1 },
    { +1,  0,  0, -1 },
    {  0, -1, +1,  0 }
};

// Read the current left encoder state.
// Bit 1 = channel A; bit 0 = channel B.
static inline uint8_t read_left_state()
{
    return
        (digitalRead(encoder_config::LEFT_A_PIN) << 1) |
         digitalRead(encoder_config::LEFT_B_PIN);
}

// Read the current right encoder state.
static inline uint8_t read_right_state()
{
    return
        (digitalRead(encoder_config::RIGHT_A_PIN) << 1) |
         digitalRead(encoder_config::RIGHT_B_PIN);
}

// Left encoder interrupt handler.
// Both A and B channels trigger this handler on CHANGE.
void IRAM_ATTR left_encoder_isr()
{
    portENTER_CRITICAL_ISR(&encoder_mux);

    const uint8_t curr = read_left_state();
    const uint8_t prev = left_prev_state;

    left_ticks += quad_table[prev][curr];
    left_prev_state = curr;

    portEXIT_CRITICAL_ISR(&encoder_mux);
}

// Right encoder interrupt handler.
void IRAM_ATTR right_encoder_isr()
{
    portENTER_CRITICAL_ISR(&encoder_mux);

    const uint8_t curr = read_right_state();
    const uint8_t prev = right_prev_state;

    right_ticks += quad_table[prev][curr];
    right_prev_state = curr;

    portEXIT_CRITICAL_ISR(&encoder_mux);
}

// Initialize encoder GPIOs and interrupts.
void encoders::init()
{
    pinMode(encoder_config::LEFT_A_PIN, INPUT_PULLUP);
    pinMode(encoder_config::LEFT_B_PIN, INPUT_PULLUP);

    pinMode(encoder_config::RIGHT_A_PIN, INPUT_PULLUP);
    pinMode(encoder_config::RIGHT_B_PIN, INPUT_PULLUP);

    // Capture initial states before enabling encoder interrupts.
    portENTER_CRITICAL(&encoder_mux);

    left_prev_state = read_left_state();
    right_prev_state = read_right_state();

    portEXIT_CRITICAL(&encoder_mux);

    attachInterrupt(
        encoder_config::LEFT_A_PIN,
        left_encoder_isr,
        CHANGE
    );

    attachInterrupt(
        encoder_config::LEFT_B_PIN,
        left_encoder_isr,
        CHANGE
    );

    attachInterrupt(
        encoder_config::RIGHT_A_PIN,
        right_encoder_isr,
        CHANGE
    );

    attachInterrupt(
        encoder_config::RIGHT_B_PIN,
        right_encoder_isr,
        CHANGE
    );
}

// Return a consistent snapshot of cumulative counts and timestamp.
encoders::EncoderData encoders::read()
{
    EncoderData data;

    portENTER_CRITICAL(&encoder_mux);

    data.left_ticks =
        left_ticks * encoder_config::LEFT_DIR;

    data.right_ticks =
        right_ticks * encoder_config::RIGHT_DIR;

    // Timestamp corresponds to this counter snapshot.
    data.timestamp_us = micros();

    portEXIT_CRITICAL(&encoder_mux);

    return data;
}

// Return the cumulative left encoder count.
int32_t encoders::get_left_ticks()
{
    int32_t ticks;

    portENTER_CRITICAL(&encoder_mux);

    ticks = left_ticks * encoder_config::LEFT_DIR;

    portEXIT_CRITICAL(&encoder_mux);

    return ticks;
}

// Return the cumulative right encoder count.
int32_t encoders::get_right_ticks()
{
    int32_t ticks;

    portENTER_CRITICAL(&encoder_mux);

    ticks = right_ticks * encoder_config::RIGHT_DIR;

    portEXIT_CRITICAL(&encoder_mux);

    return ticks;
}

// Reset cumulative counts without changing the quadrature states.
void encoders::reset()
{
    portENTER_CRITICAL(&encoder_mux);

    left_ticks = 0;
    right_ticks = 0;

    portEXIT_CRITICAL(&encoder_mux);
}
