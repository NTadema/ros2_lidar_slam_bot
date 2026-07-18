#pragma once

#include <cstdint>
#include <cstddef>


namespace protocol
{

constexpr uint8_t START1 = 0xAA;
constexpr uint8_t START2 = 0x55;

constexpr uint8_t MAX_PAYLOAD = 32;


// Packet types
enum class PacketType : uint8_t
{
    ENCODER = 1,
    IMU     = 2,
    MOTOR   = 3
};


// Data structures

struct EncoderPacket
{
    int32_t left_ticks;
    int32_t right_ticks;
};


struct ImuPacket
{
    uint32_t timestamp_us;

    float ax;
    float ay;
    float az;

    float gx;
    float gy;
    float gz;
};


struct MotorCommand
{
    int16_t left_speed;
    int16_t right_speed;
};


// Callback type for received packets

using PacketCallback =
    void(*)(PacketType type,
            const uint8_t* payload,
            uint8_t length);


// Protocol initialization

void init();


// Packet creation

size_t create_encoder_packet(
    const EncoderPacket& packet,
    uint8_t* output,
    size_t max_size);


// Parser

void process_byte(uint8_t byte);


// Register packet callback

void set_callback(PacketCallback callback);


// CRC

uint16_t crc16_ccitt(
    const uint8_t* data,
    size_t length);


}
