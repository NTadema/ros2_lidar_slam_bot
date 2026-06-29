#include "drivers/imu.h"
#include <Wire.h>

namespace
{
    constexpr uint8_t PWR_MGMT_1 = 0x6B;
    constexpr uint8_t ACCEL_XOUT_H = 0x3B;

    const float G = 9.80665f;

    // Default sensitivity
    constexpr float ACCEL_SCALE = 16384.0f; // ±2 g
    constexpr float GYRO_SCALE = 131.0f; // ±250 °/s
}

namespace
{
    int16_t read_int16(const uint8_t* data)
    {
        return static_cast<int16_t>(
            (static_cast<uint16_t>(data[0]) << 8) |
             static_cast<uint16_t>(data[1]));
    }
}

namespace
{
    bool read_registers(uint8_t start_register, uint8_t* buffer, size_t length)
    {
        Wire.beginTransmission(imu_config::I2C_ADDRESS);
        Wire.write(start_register);

        if (Wire.endTransmission(false) != 0)
            return false;
        return Wire.requestFrom(imu_config::I2C_ADDRESS, static_cast<uint8_t>(length)) 
        == length && Wire.readBytes(buffer, length) == length;
    }
}

bool imu::init()
{
    Wire.begin(imu_config::SDA_PIN, imu_config::SCL_PIN, imu_config::I2C_CLOCK);
    Wire.beginTransmission(imu_config::I2C_ADDRESS);
    Wire.write(PWR_MGMT_1);
    Wire.write(0x00); // Wake MPU6050

    return (Wire.endTransmission() == 0);
}

imu::ImuData imu::read()
{
    ImuData data;

    data.timestamp_us = micros();

    uint8_t buffer[14];


    if (!read_registers(ACCEL_XOUT_H, buffer, sizeof(buffer)))
    {
        return data; // Set error flag TODO
    }

    // Read accelerometer 
    int16_t raw_ax = read_int16(&buffer[0]);
    int16_t raw_ay = read_int16(&buffer[2]);
    int16_t raw_az = read_int16(&buffer[4]);

    // buffer[6] and [7] are temperature

    // Read gyroscope
    int16_t raw_gx = read_int16(&buffer[8]);
    int16_t raw_gy = read_int16(&buffer[10]);
    int16_t raw_gz = read_int16(&buffer[12]);

    // Convert to 
    data.ax = raw_ax / ACCEL_SCALE * G;
    data.ay = raw_ay / ACCEL_SCALE * G;
    data.az = raw_az / ACCEL_SCALE * G;

    data.gx = raw_gx / GYRO_SCALE * DEG_TO_RAD;
    data.gy = raw_gy / GYRO_SCALE * DEG_TO_RAD;
    data.gz = raw_gz / GYRO_SCALE * DEG_TO_RAD;

    return data;
}
