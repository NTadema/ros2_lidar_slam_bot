#pragma once

#include <cstdint>
#include <vector>

enum class PacketType : uint8_t
{
    ENCODER = 1,
    IMU      = 2,
    MOTOR    = 3
};

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

class SerialProtocol
{
public:

    SerialProtocol();

    // Build packets
    std::vector<uint8_t> createMotorPacket(const MotorCommand& cmd);

    // Feed one received byte into the parser
    bool processByte(uint8_t byte);

    // Results after a packet has been parsed
    PacketType packetType() const;

    EncoderPacket encoderPacket() const;

    ImuPacket imuPacket() const;

private:

    uint16_t crc16(const std::vector<uint8_t>& data);

    bool verifyCRC();

    void resetParser();

    enum class ParserState
    {
        WAIT1,
        WAIT2,
        TYPE,
        LENGTH,
        PAYLOAD,
        CRC_LOW,
        CRC_HIGH
    };

    ParserState state_;

    uint8_t type_;
    uint8_t length_;

    std::vector<uint8_t> payload_;

    uint8_t crcLow_;
    uint8_t crcHigh_;

    EncoderPacket encoder_;
    ImuPacket imu_;
};
