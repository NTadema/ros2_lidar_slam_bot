#include <Arduino.h>
#include <Wire.h>
#include "drivers/imu.hpp"

// Private constants (MPU6050 registers)
namespace
{
    // Register addresses
    constexpr uint8_t PWR_MGMT_1 = 0x6B; // Power management, controls wake/sleep
    constexpr uint8_t ACCEL_XOUT_H = 0x3B; // Start of accelerometer/gyro data

    const float G = 9.80665f; // Standard gravity

    // Default sensor sensitivity scaling
    constexpr float ACCEL_SCALE = 16384.0f; // ±2 g
    constexpr float GYRO_SCALE = 131.0f; // ±250 °/s
}

// Helper functions
namespace
{
    // Converts two 8-bit registers to a 16-bit signed integer
    // IMU stores 16-bit values as MSB and LSB in adjacent registers
    int16_t read_int16(const uint8_t* data)
    {
        return static_cast<int16_t>(
            (static_cast<uint16_t>(data[0]) << 8) | // MSB shifted left
             static_cast<uint16_t>(data[1])); // LSB
    }
}

// Reads a block of registers from the IMU over I2C
// Args:
    //   start_register: First register address to read (e.g., ACCEL_XOUT_H).
    //   buffer: Output array to store read bytes.
    //   length: Number of bytes to read.
    // Returns:
    //   true if all bytes were read successfully, false on I2C error.
namespace
{
    bool read_registers(uint8_t start_register, uint8_t* buffer, size_t length)
    {
        // Send the register address to start reading from
        Wire.beginTransmission(imu_config::I2C_ADDRESS);
        Wire.write(start_register);
        
        // End transmission with "false" to keep I2C bus active for reading
        if (Wire.endTransmission(false) != 0)
            return false;
        
        // Request "length" bytes from the IMU
        return Wire.requestFrom(imu_config::I2C_ADDRESS, static_cast<uint8_t>(length)) 
        == length && Wire.readBytes(buffer, length) == length;
    }
}


// Initializes the IMU (MPU6050) and I2C interface.
// Returns:
//   True if IMU responds and wakes up, False on I2C failure.
bool imu::init()
{
    // Start I2C transmission to PWR_MGMT_1 register
    Wire.begin(imu_config::SDA_PIN, imu_config::SCL_PIN, imu_config::I2C_CLOCK);
    Wire.beginTransmission(imu_config::I2C_ADDRESS);
    Wire.write(PWR_MGMT_1);
    Wire.write(0x00); // Wake MPU6050

    // End transmission and check for ACK (0 = success)
    return (Wire.endTransmission() == 0);
}

// Reads a full IMU data frame (accel + gyro + timestamp).
// Returns:
//   ImuData struct with:
//     - timestamp_us: Microseconds since system boot (for sensor fusion timing).
//     - ax/ay/az: Acceleration in m/s² (gravity-compensated if needed).
//     - gx/gy/gz: Angular velocity in rad/s (robotics standard unit).
imu::ImuData imu::read()
{
    ImuData data{};

    uint8_t buffer[14]; // Holds raw data for: accel (6B) + temp (2B) + gyro (6B)

    // Read 14 bytes starting from ACCEL_XOUT_H
    if (!read_registers(ACCEL_XOUT_H, buffer, sizeof(buffer)))
    {
        return data; // Set error flag TODO
    }

    // Register map: 0x3B-0x40 = [AX_H, AX_L, AY_H, AY_L, AZ_H, AZ_L]
    int16_t raw_ax = read_int16(&buffer[0]);
    int16_t raw_ay = read_int16(&buffer[2]);
    int16_t raw_az = read_int16(&buffer[4]);

    // Skip temperature (buffer[6] and [7]); not used here

    // Register map: 0x43-0x48 = [GX_H, GX_L, GY_H, GY_L, GZ_H, GZ_L]
    int16_t raw_gx = read_int16(&buffer[8]);
    int16_t raw_gy = read_int16(&buffer[10]);
    int16_t raw_gz = read_int16(&buffer[12]);

    // Convert raw data to physical units
    // Accelerometer: Scale raw LSB to m/s² using sensitivity and gravity
    data.ax = raw_ax / ACCEL_SCALE * G;
    data.ay = raw_ay / ACCEL_SCALE * G;
    data.az = raw_az / ACCEL_SCALE * G;

    // Gyroscope: Scale raw LSB to rad/s
    // 1. Convert LSB to °/s
    // 2. Convert °/s to rad/s
    data.gx = raw_gx / GYRO_SCALE * DEG_TO_RAD;
    data.gy = raw_gy / GYRO_SCALE * DEG_TO_RAD;
    data.gz = raw_gz / GYRO_SCALE * DEG_TO_RAD;

    // Timestamp in microseconds since system boot
    data.timestamp_us = micros();

    return data;
}
