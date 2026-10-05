#pragma once
#include <cstdint>
#include <HardwareSerial.h>


// UART configuration for ESP32
namespace uart_config
{
    const uint32_t BAUD_RATE = 115200;
    // TX pin of UART communication is on GPIO 16 (TX2) of ESP32
    const uint8_t TXD2_PIN = 16;
    // RX pin of UART communication is on GPIO 17 (RX2) of ESP32
    const uint8_t RXD2_PIN = 17;
    extern HardwareSerial SerialPort; // UART Serial port instance for communication
}

// Packet types
enum PacketType : uint8_t
{
    ENCODER = 1,
    IMU = 2,
    MOTOR = 3
};

struct EncoderPacket
{
    uint32_t timestamp_us; // Timestamp in microseconds since system boot
    int32_t left_ticks;  // Cumulative tick count for left encoder
    int32_t right_ticks; // Cumulative tick count for right encoder
};

struct ImuPacket
{
    uint32_t timestamp_us; // Timestamp in microseconds since system boot
    float ax;              // Acceleration in X-axis (m/s²)
    float ay;              // Acceleration in Y-axis (m/s²)
    float az;              // Acceleration in Z-axis (m/s²)
    float gx;              // Angular velocity around X-axis (rad/s)
    float gy;              // Angular velocity around Y-axis (rad/s)
    float gz;              // Angular velocity around Z-axis (rad/s)
};

struct MotorCommand
{
    uint32_t timestamp_us; // Timestamp in microseconds since system boot
    int16_t left_speed;  // Speed command for left motor (-255 to 255)
    int16_t right_speed; // Speed command for right motor (-255 to 255)
};


namespace uart_comm
{
    void init();

    bool send_encoder_packet(const EncoderPacket& packet);
    bool send_imu_packet(const ImuPacket& packet);

    MotorCommand receive_motor_command();

    void process_byte(uint8_t b); // RX parser entry point
}

