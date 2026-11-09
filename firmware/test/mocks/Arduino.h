
#pragma once

#include <cstdint>

#define HIGH 1
#define LOW  0
#define OUTPUT 1

// Mock Arduino functions
void pinMode(uint8_t pin, uint8_t mode);
void digitalWrite(uint8_t pin, uint8_t value);

double ledcSetup(uint8_t channel, double freq, uint8_t resolution);
void ledcAttachPin(uint8_t pin, uint8_t channel);
void ledcWrite(uint8_t channel, uint32_t duty);

// Arduino constrain replacement
template <typename T>
T constrain(T value, T low, T high) {
    return (value < low) ? low : (value > high) ? high : value;
}
