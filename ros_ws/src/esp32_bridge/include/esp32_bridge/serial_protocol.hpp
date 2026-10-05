// Prevent multiple inclusion of this header file
#ifndef SERIAL_PROTOCOL_HPP
#define SERIAL_PROTOCOL_HPP

// C++ standard library includes
#include <cstdint>
#include <vector>

// Defines message types used in the UART protocol
enum class PacketType : uint8_t
{
    ENCODER = 1, // Encoder data packet
    IMU      = 2, // IMU data packet
    MOTOR    = 3 // Motor command packet
};

// Represents encoder data received from the ESP32
struct EncoderPacket
{   
    uint32_t timestamp_us; // Sensor timestamp in microseconds
    
    // Number of ticks counted by the left and right wheel encoders
    int32_t left_ticks;
    int32_t right_ticks;
};

// Represents IMU data received from the ESP32
struct ImuPacket
{
    uint32_t timestamp_us; // Sensor timestamp in microseconds

    // Accelerometer readings in m/s^2
    float ax;
    float ay;
    float az;

    // Gyroscope readings in rad/s
    float gx;
    float gy;
    float gz;
};

// Represents motor command data to be sent to the ESP32
struct MotorCommand
{   
    // Desired speed for the left and right motors
    int16_t left_speed;
    int16_t right_speed;
};

// Handles the parsing and creation of packets for UART communication with the ESP32
class SerialProtocol
{
public:
    // Initializes the packet parser state
    SerialProtocol();

    // Creates a UART packet containing motor commands
    std::vector<uint8_t> createMotorPacket(const MotorCommand& cmd);

    // Feeds incoming UART bytes into the packet parser
    bool processByte(uint8_t byte);

    // Returns data extracted from the last valid packet
    PacketType packetType() const;
    EncoderPacket encoderPacket() const;
    ImuPacket imuPacket() const;

private:
    // Calculates the CRC-16 checksum for packet validation
    uint16_t crc16(const std::vector<uint8_t>& data);

    // Checks whether the received packet checksum is valid
    bool verifyCRC();

    // Resets the parser state for receiving the next packet
    void resetParser(); 

    // Defines the states used while decoding incoming packets
    enum class ParserState
    {
        WAIT1,      // Waiting for the first start byte (0xAA)
        WAIT2,      // Waiting for the second start byte (0x55)
        TYPE,       // Reading packet type
        LENGTH,     // Reading payload length
        PAYLOAD,    // Collecting payload bytes
        CRC_LOW,    // Reading the low byte of the CRC
        CRC_HIGH    // Reading the high byte of the CRC
    };

    // Current parser state
    ParserState state_;

    // Current packet type and length
    uint8_t type_;
    uint8_t length_;

    // Buffer to hold the payload of the current packet
    std::vector<uint8_t> payload_;

    // Current CRC bytes
    uint8_t crcLow_;
    uint8_t crcHigh_;

    // Temporary decoded data for the current packet
    EncoderPacket encoder_;
    ImuPacket imu_;

    // Last successfully decoded packet data
    PacketType last_packet_type_;
    EncoderPacket last_encoder_;
    ImuPacket last_imu_;
};

#endif // SERIAL_PROTOCOL_HPP
