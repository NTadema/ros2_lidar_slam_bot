#include "esp32_bridge/serial_protocol.hpp"
#include <cstring>

// Private helper functions for byte conversion
namespace
{
// Converts four little-endian bytes into an unsigned 32-bit integer
uint32_t readUint32LE(const std::vector<uint8_t>& data, size_t offset)
{
    return static_cast<uint32_t>(data[offset]) |
           (static_cast<uint32_t>(data[offset + 1]) << 8) |
           (static_cast<uint32_t>(data[offset + 2]) << 16) |
           (static_cast<uint32_t>(data[offset + 3]) << 24);
}

// Converts four little-endian bytes into a floating-point number
float readFloatLE(const std::vector<uint8_t>& data, size_t offset)
{
    uint32_t bits = readUint32LE(data, offset);

    float value;
    std::memcpy(&value, &bits, sizeof(float));

    return value;
}

// Converts four little-endian bytes into a signed integer
int32_t readInt32LE(const std::vector<uint8_t>& data, size_t offset)
{
    return static_cast<int32_t>(readUint32LE(data, offset));
}
}  // namespace

// Initialize parser and stored packet values
SerialProtocol::SerialProtocol()
: state_(ParserState::WAIT1)
, type_(0)
, length_(0)
, crcLow_(0)
, crcHigh_(0)
, last_packet_type_(PacketType::MOTOR)
{
    resetParser();
}

std::vector<uint8_t> SerialProtocol::createMotorPacket(
        const MotorCommand& cmd)
{   
    // Create packet buffer
    std::vector<uint8_t> packet;

    // Add packet synchronization bytes
    packet.push_back(0xAA);
    packet.push_back(0x55);

    // Add motor command packet identifier
    packet.push_back(static_cast<uint8_t>(PacketType::MOTOR));

    // Add payload size
    packet.push_back(sizeof(MotorCommand));

    // Encode motor speeds as little-endian bytes
    packet.push_back(cmd.left_speed & 0xFF);
    packet.push_back((cmd.left_speed >> 8) & 0xFF);

    packet.push_back(cmd.right_speed & 0xFF);
    packet.push_back((cmd.right_speed >> 8) & 0xFF);

    // Calculate checksum for packet verification
    uint16_t crc = crc16(packet);

    packet.push_back(crc & 0xFF);
    packet.push_back((crc >> 8) & 0xFF);

    return packet;
}

// Process incoming bytes using a state machine
bool SerialProtocol::processByte(uint8_t byte)
{
    switch (state_)
    {   
        // Wait for first packet header byte
        case ParserState::WAIT1:
            if (byte == 0xAA)
            {
                state_ = ParserState::WAIT2;
            }
            break;

        // Confirm second packet header byte
        case ParserState::WAIT2:
            if (byte == 0x55)
            {
                state_ = ParserState::TYPE;
            }
            else
            {
                resetParser();
            }
            break;

        // Store packet type
        case ParserState::TYPE:
            type_ = byte;
            state_ = ParserState::LENGTH;
            break;

        // Store expected packet length
        case ParserState::LENGTH:
            length_ = byte;
            payload_.clear();
            state_ = (length_ == 0) ? ParserState::CRC_LOW : ParserState::PAYLOAD;
            break;
        
        // Collect payload bytes until complete
        case ParserState::PAYLOAD:
            payload_.push_back(byte);
            if (payload_.size() >= length_)
            {
                state_ = ParserState::CRC_LOW;
            }
            break;
        
        // Receive CRC low byte
        case ParserState::CRC_LOW:
            crcLow_ = byte;
            state_ = ParserState::CRC_HIGH;
            break;
        
        // Receive CRC high byte
        case ParserState::CRC_HIGH:
            crcHigh_ = byte;

            // Process packet only if checksum is correct
            if (verifyCRC())
            {   
                // Decode encoder feedback payload
                if (type_ == static_cast<uint8_t>(PacketType::ENCODER) && length_ == 12)
                {
                    encoder_.timestamp_us = readUint32LE(payload_, 0);
                    encoder_.left_ticks = readInt32LE(payload_, 4);
                    encoder_.right_ticks = readInt32LE(payload_, 8);
                }

                // Decode IMU feedback payload
                else if (type_ == static_cast<uint8_t>(PacketType::IMU) && length_ == 28)
                {
                    imu_.timestamp_us = readUint32LE(payload_, 0);

                    imu_.ax = readFloatLE(payload_, 4);
                    imu_.ay = readFloatLE(payload_, 8);
                    imu_.az = readFloatLE(payload_, 12);

                    imu_.gx = readFloatLE(payload_, 16);
                    imu_.gy = readFloatLE(payload_, 20);
                    imu_.gz = readFloatLE(payload_, 24);
                }

                // Store the last successfully decoded packet data
                last_packet_type_ = static_cast<PacketType>(type_);
                last_encoder_ = encoder_;
                last_imu_ = imu_;

                resetParser();
                return true;
            }

        resetParser();
        break;
    }
    return false;
}

PacketType SerialProtocol::packetType() const
{
    return last_packet_type_;
}

EncoderPacket SerialProtocol::encoderPacket() const
{
    return last_encoder_;
}

ImuPacket SerialProtocol::imuPacket() const
{
    return last_imu_;
}

uint16_t SerialProtocol::crc16(const std::vector<uint8_t>& data)
{
    uint16_t crc = 0xFFFF;

    for (uint8_t b : data)
    {
        crc ^= static_cast<uint16_t>(b) << 8;

        for (int i = 0; i < 8; ++i)
        {
            if (crc & 0x8000)
            {
                crc = static_cast<uint16_t>((crc << 1) ^ 0x1021);
            }
            else
            {
                crc <<= 1;
            }
        }
    }

    return crc;
}

// Decode encoder feedback payload
bool SerialProtocol::verifyCRC()
{
    std::vector<uint8_t> data;
    data.push_back(0xAA);
    data.push_back(0x55);
    data.push_back(type_);
    data.push_back(length_);
    data.insert(data.end(), payload_.begin(), payload_.end());

    uint16_t expected = crc16(data);
    uint16_t actual = static_cast<uint16_t>(crcLow_ | (crcHigh_ << 8));

    return expected == actual;
}

// Return parser to waiting-for-header state
void SerialProtocol::resetParser()
{
    state_ = ParserState::WAIT1;
    payload_.clear();
    type_ = 0;
    length_ = 0;
    crcLow_ = 0;
    crcHigh_ = 0;
}
