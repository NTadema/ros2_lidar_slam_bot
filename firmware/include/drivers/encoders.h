#pragma once

namespace encoder_config{
    // Encoder driver pins
    constexpr int LEFT_A_PIN = 32;
    constexpr int LEFT_B_PIN = 33;
    constexpr int RIGHT_A_PIN = 21;
    constexpr int RIGHT_B_PIN = 19;
    // Define encoder direction
    constexpr int LEFT_DIR = 1;
    constexpr int RIGHT_DIR = 1;
}

namespace encoders{
    void init();
    long get_left_ticks();
    long get_right_ticks();
    long get_left_ticks_raw();
    long get_right_ticks_raw();
    void reset();
}

