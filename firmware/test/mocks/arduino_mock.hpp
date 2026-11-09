
#pragma once

#include <cstdint>

namespace ArduinoMock {

    struct State {
        uint8_t pin_modes[256] = {};
        uint8_t digital_values[256] = {};

        uint32_t pwm_values[256] = {};
        uint8_t attached_channels[256] = {};

        double pwm_frequency[256] = {};
        uint8_t pwm_resolution[256] = {};
    };

    State& state();

    void reset();

}
