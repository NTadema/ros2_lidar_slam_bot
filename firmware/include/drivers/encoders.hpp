#pragma once


// Encoder hardware configuration
namespace encoder_config{
    // Quadrature encoder channel pins
    // Each encoder requires two pins (A and B channels) for:
    //   - Counting pulses (A channel)
    //   - Determining direction (A vs. B phase relationship)

    // Left encoder
    constexpr int LEFT_A_PIN = 32;
    constexpr int LEFT_B_PIN = 33;

    // Right encoder
    constexpr int RIGHT_A_PIN = 19;
    constexpr int RIGHT_B_PIN = 18;

    // Encoder direction multiplier:
    //   +1: Normal direction (forward rotation increases ticks)
    //   -1: Reversed direction (forward rotation decreases ticks)
    // Adjust these if the encoder counts backward relative to the robot's
    // forward motion (e.g., due to wiring or mechanical orientation)
    constexpr int LEFT_DIR = 1;
    constexpr int RIGHT_DIR = 1;
}

namespace encoders{

    // Struct to hold a single encoders measurement frame
    struct EncoderData{
        int32_t left_ticks;
        int32_t right_ticks;
        uint32_t timestamp_us; // Timestamp in microseconds
    };

    // Initializes encoder hardware.
    void init();

    // Returns the current encoder tick counts and timestamp
    EncoderData read();

    // Returns the cumulative tick count for the left encoder.
    // Ticks are signed (int32_t) to support:
    //   - Forward motion (positive ticks)
    //   - Reverse motion (negative ticks)
    //   - Large distances without overflow (int32_t = ±2.1 billion ticks)
    int32_t get_left_ticks();

    // Returns the cumulative tick count for the right encoder
    // Same behavior as get_left_ticks(), but for the right wheel
    int32_t get_right_ticks();

    // Resets both left and right encoder tick counters to zero
    // Call this:
    //   - At system startup (after init())
    //   - Before starting a new motion command (for relative distance tracking)
    void reset();
}

