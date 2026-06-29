#pragma once

#include <Arduino.h>


namespace imu_config{
    constexpr uint8_t I2C_ADDRESS = 0X68;

    constexpr uint8_t SDA_PIN = 21;
    constexpr uint8_t SCL_PIN = 22;

    constexpr uint32_t I2C_CLOCK = 400000; // 400 kHz
}


namespace imu{
    struct ImuData
    {
        uint32_t timestamp_us;

        float ax;
        float ay;
        float az;

        float gx;
        float gy;
        float gz;
    };
    bool init();
    ImuData read();
}
