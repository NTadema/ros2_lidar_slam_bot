#pragma once

#include <cstdint>

namespace esp32_bridge
{

// Packet framing constants
constexpr uint8_t START1 = 0xAA;
constexpr uint8_t START2 = 0x55;


// Packet types
enum class PacketType : uint8_t
{
    ENCODER = 1,
    IMU = 2,
    MOTOR = 3
};


// Maximum payload size.
// Adjust later if IMU packets require more data.
constexpr uint8_t MAX_PAYLOAD_SIZE = 64;


// Encoder packet payload
// Matches ESP32 serialization:
//
// int32_t left_ticks
// int32_t right_ticks
//
struct EncoderPacket
{
    int32_t left_ticks;
    int32_t right_ticks;
};


// Generic packet container.
// Used by parser before decoding payloads.
struct Packet
{
    PacketType type;
    uint8_t length;
    uint8_t payload[MAX_PAYLOAD_SIZE];
};


} // namespace esp32_bridge
