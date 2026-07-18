#pragma once

// IMU hardware configuration
namespace imu_config{
    
    constexpr uint8_t I2C_ADDRESS = 0X68;

    // ESP32 default pins for I2C communication
    // SDA: Serial Data Line (bidirectional)
    // SCL: Serial Clock Line (driven by master)
    constexpr uint8_t SDA_PIN = 21;
    constexpr uint8_t SCL_PIN = 22;

     // I2C bus clock frequency in Hz
    constexpr uint32_t I2C_CLOCK = 400000; // 400 kHz
}

// IMU data and API
namespace imu{
    // Struct to hold a single IMU measurement frame
    struct ImuData
    {
        // Timestamp in microseconds since system boot
        uint32_t timestamp_us;
        
        // Accelerometer data in m/s² (gravity-inclusive)
        // Positive values:
        //   - ax: Forward (robot's +X axis).
        //   - ay: Left (robot's +Y axis).
        //   - az: Up (robot's +Z axis).
        // Note: az ≈ +9.81 m/s² when the robot is stationary and level
        float ax;
        float ay;
        float az;

         // Gyroscope data in rad/s (right-hand rule)
        // Positive values:
        //   - gx: Roll right (robot's +X axis rotation)
        //   - gy: Pitch up (robot's +Y axis rotation)
        //   - gz: Yaw left (robot's +Z axis rotation)
        // Note: Raw gyro data drifts over time; requires calibration or fusion
        float gx;
        float gy;
        float gz;
    };

    // Initializes the IMU hardware and I2C
    bool init();

    // Reads a single frame of IMU data (accel + gyro)
    ImuData read();
}
