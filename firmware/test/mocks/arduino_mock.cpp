#include "mocks/Arduino.h"
#include "mocks/arduino_mock.hpp"

namespace ArduinoMock {

    static State mock_state{};

    State& state() {
        return mock_state;
    }

    void reset() {
        mock_state = State{};
    }

}

void pinMode(uint8_t pin, uint8_t mode) {
    ArduinoMock::state().pin_modes[pin] = mode;
}

void digitalWrite(uint8_t pin, uint8_t value) {
    ArduinoMock::state().digital_values[pin] = value;
}

double ledcSetup(uint8_t channel, double freq, uint8_t resolution) {
    auto& s = ArduinoMock::state();

    s.pwm_frequency[channel] = freq;
    s.pwm_resolution[channel] = resolution;

    return freq;
}

void ledcAttachPin(uint8_t pin, uint8_t channel) {
    ArduinoMock::state().attached_channels[pin] = channel;
}

void ledcWrite(uint8_t channel, uint32_t duty) {
    ArduinoMock::state().pwm_values[channel] = duty;
}