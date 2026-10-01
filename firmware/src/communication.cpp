#include <cstring>
#include "communication.hpp"

// -----------------------------------------------------------------------------
// UART packet protocol
//
// Packet format:
//
// +---------+---------+------+--------+--------------+----------+
// | START1  | START2  | TYPE | LENGTH | PAYLOAD      | CRC16    |
// +---------+---------+------+--------+--------------+----------+
// | 0xAA    | 0x55    | 1B   | 1B     | LENGTH bytes | 2B       |
//
// CRC is computed over:
// START1 | START2 | TYPE | LENGTH | PAYLOAD
// -----------------------------------------------------------------------------

static constexpr uint8_t START1 = 0xAA;
static constexpr uint8_t START2 = 0x55;
static constexpr uint8_t HEADER_SIZE = 4;
static constexpr size_t MAX_PAYLOAD = 32;

// UART2 instance
HardwareSerial uart_config::SerialPort(2);

// CRC-16/CCITT-FALSE
static uint16_t crc16_ccitt(const uint8_t* data, size_t len)
{
    uint16_t crc = 0xFFFF;

    for (size_t i = 0; i < len; i++)
    {
        crc ^= static_cast<uint16_t>(data[i]) << 8;

        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x8000)
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF;
            else
                crc = (crc << 1) & 0xFFFF;
        }
    }

    return crc;
}

// Serialize a signed 32-bit integer into little-endian format
static inline void writeInt32(uint8_t* out, int32_t value)
{
    out[0] = (value >> 0) & 0xFF;
    out[1] = (value >> 8) & 0xFF;
    out[2] = (value >> 16) & 0xFF;
    out[3] = (value >> 24) & 0xFF;
}


// UART receive parser
enum class ParserState
{
    WAIT_FOR_START1,
    WAIT_FOR_START2,
    READ_PACKET_TYPE,
    READ_PAYLOAD_LENGTH,
    READ_PAYLOAD,
    READ_CRC_LOW,
    READ_CRC_HIGH
};

// Stores the state of the UART parser while reconstructing packets
struct Parser
{
    ParserState state = ParserState::WAIT_FOR_START1;

    uint8_t type = 0;
    uint8_t len = 0;

    uint8_t payload[MAX_PAYLOAD];
    uint8_t index = 0;

    uint8_t crc_low = 0;
    uint8_t crc_high = 0;
};

static Parser parser;

// Verify CRC of the reconstructed packet
static bool verify_packet_crc(const Parser& parser)
{
    uint8_t tmp[HEADER_SIZE + MAX_PAYLOAD];

    tmp[0] = START1;
    tmp[1] = START2;
    tmp[2] = parser.type;
    tmp[3] = parser.len;

    memcpy(&tmp[4], parser.payload, parser.len);

    uint16_t computed = crc16_ccitt(tmp, HEADER_SIZE + parser.len);

    return computed ==
       ((static_cast<uint16_t>(parser.crc_high) << 8) |
        parser.crc_low);
}


// Packet dispatcher
// Called only after framing and CRC have been successfully verified
static void handle_packet(uint8_t type,
                          const uint8_t* payload,
                          uint8_t len)
{
    switch (type)
    {
        case PacketType::ENCODER:
        {
            int32_t left;
            int32_t right;

            memcpy(&left, &payload[0], sizeof(int32_t));
            memcpy(&right, &payload[4], sizeof(int32_t));

            break;
        }

        case PacketType::IMU:
            // TODO
            break;

        case PacketType::MOTOR:
            // TODO
            break;

        default:
            break;
    }
}

// UART receive state machine
void uart_comm::process_byte(uint8_t byte)
{
    switch (parser.state)
    {
        case ParserState::WAIT_FOR_START1:

            if (byte == START1)
                parser.state = ParserState::WAIT_FOR_START2;

            break;

        case ParserState::WAIT_FOR_START2:

            parser.state =
                (byte == START2)
                    ? ParserState::READ_PACKET_TYPE
                    : ParserState::WAIT_FOR_START1;

            break;

        case ParserState::READ_PACKET_TYPE:

            parser.type = byte;
            parser.state = ParserState::READ_PAYLOAD_LENGTH;
            break;

        case ParserState::READ_PAYLOAD_LENGTH:

            parser.len = byte;

            if (parser.len == 0 || parser.len > MAX_PAYLOAD)
            {
                parser.state = ParserState::WAIT_FOR_START1;
                break;
            }

            parser.index = 0;
            parser.state = ParserState::READ_PAYLOAD;
            break;

        case ParserState::READ_PAYLOAD:

            parser.payload[parser.index++] = byte;

            if (parser.index >= parser.len)
                parser.state = ParserState::READ_CRC_LOW;

            break;

        case ParserState::READ_CRC_LOW:

            parser.crc_low = byte;
            parser.state = ParserState::READ_CRC_HIGH;

            break;


        case ParserState::READ_CRC_HIGH:

            parser.crc_high = byte;

            if (verify_packet_crc(parser))
            {
                handle_packet(parser.type,
                            parser.payload,
                            parser.len);
            }
            else
            {
                Serial.println("CRC FAIL");
            }


            parser.state = ParserState::WAIT_FOR_START1;

            break;
            }
}

// UART initialization PI4b
/*void uart_comm::init()
{
    uart_config::SerialPort.begin(
        uart_config::BAUD_RATE,
        SERIAL_8N1,
        uart_config::RXD2_PIN,
        uart_config::TXD2_PIN);
}*/

// TEMPORARY UART initialization for PC development via USB serial
void uart_comm::init()
{
    Serial.begin(uart_config::BAUD_RATE);
}


// Generic packet transmitter Pi4b
/*static bool send_packet(PacketType type,
                        const uint8_t* payload,
                        uint8_t len)
{
    uint8_t header[HEADER_SIZE] =
    {
        START1,
        START2,
        static_cast<uint8_t>(type),
        len
    };

    uint8_t tmp[HEADER_SIZE + MAX_PAYLOAD];

    memcpy(tmp, header, HEADER_SIZE);
    memcpy(tmp + HEADER_SIZE, payload, len);

    uint16_t crc = crc16_ccitt(tmp, HEADER_SIZE + len);

    bool ok = true;

    ok &= uart_config::SerialPort.write(header, HEADER_SIZE) == HEADER_SIZE;
    ok &= uart_config::SerialPort.write(payload, len) == len;
    ok &= uart_config::SerialPort.write(static_cast<uint8_t>(crc & 0xFF)) == 1;
    ok &= uart_config::SerialPort.write(static_cast<uint8_t>((crc >> 8) & 0xFF)) == 1;

    return ok;
}*/

// TEMPORARY generic packet transmitter for PC development via USB serial
static bool send_packet(PacketType type,
                        const uint8_t* payload,
                        uint8_t len)
{
    uint8_t header[HEADER_SIZE] =
    {
        START1,
        START2,
        static_cast<uint8_t>(type),
        len
    };

    uint8_t tmp[HEADER_SIZE + MAX_PAYLOAD];

    memcpy(tmp, header, HEADER_SIZE);
    memcpy(tmp + HEADER_SIZE, payload, len);

    uint16_t crc = crc16_ccitt(tmp, HEADER_SIZE + len);

    bool ok = true;
    
    ok &= Serial.write(header, HEADER_SIZE) == HEADER_SIZE;
    ok &= Serial.write(payload, len) == len;
    ok &= Serial.write(static_cast<uint8_t>(crc & 0xFF)) == 1;
    ok &= Serial.write(static_cast<uint8_t>((crc >> 8) & 0xFF)) == 1;

    return ok;
}


// Encoder packet
bool uart_comm::send_encoder_packet(const EncoderPacket& packet)
{
    uint8_t payload[sizeof(EncoderPacket)];

    writeInt32(&payload[0], packet.left_ticks);
    writeInt32(&payload[4], packet.right_ticks);

    return send_packet(PacketType::ENCODER,
                       payload,
                       sizeof(payload));
}
