#include "protocol.hpp"

#include <cstring>


namespace protocol
{


// ------------------------------------------------------
// CRC-16 CCITT-FALSE
// ------------------------------------------------------

uint16_t crc16_ccitt(
    const uint8_t* data,
    size_t length)
{
    uint16_t crc = 0xFFFF;


    for(size_t i = 0; i < length; i++)
    {
        crc ^= (uint16_t)data[i] << 8;


        for(uint8_t j = 0; j < 8; j++)
        {
            if(crc & 0x8000)
            {
                crc = (crc << 1) ^ 0x1021;
            }
            else
            {
                crc <<= 1;
            }
        }
    }


    return crc;
}



// ------------------------------------------------------
// Helpers
// ------------------------------------------------------

static void write_int32(
    uint8_t* dst,
    int32_t value)
{
    dst[0] = value & 0xFF;
    dst[1] = (value >> 8) & 0xFF;
    dst[2] = (value >> 16) & 0xFF;
    dst[3] = (value >> 24) & 0xFF;
}



static int32_t read_int32(
    const uint8_t* src)
{
    return
        ((int32_t)src[0]) |
        ((int32_t)src[1] << 8) |
        ((int32_t)src[2] << 16) |
        ((int32_t)src[3] << 24);
}



// ------------------------------------------------------
// Packet creation
// ------------------------------------------------------

size_t create_encoder_packet(
    const EncoderPacket& packet,
    uint8_t* output,
    size_t max_size)
{
    constexpr uint8_t payload_length = 8;

    constexpr size_t packet_size =
        2 +       // start bytes
        1 +       // type
        1 +       // length
        payload_length +
        2;        // CRC


    if(max_size < packet_size)
        return 0;



    output[0] = START1;
    output[1] = START2;

    output[2] =
        static_cast<uint8_t>(PacketType::ENCODER);

    output[3] = payload_length;



    write_int32(&output[4],
                packet.left_ticks);


    write_int32(&output[8],
                packet.right_ticks);



    uint16_t crc =
        crc16_ccitt(output, 12);



    output[12] =
        crc & 0xFF;

    output[13] =
        (crc >> 8) & 0xFF;


    return packet_size;
}



// ------------------------------------------------------
// Parser
// ------------------------------------------------------

enum class ParserState
{
    START1,
    START2,
    TYPE,
    LENGTH,
    PAYLOAD,
    CRC_LOW,
    CRC_HIGH
};


struct Parser
{
    ParserState state =
        ParserState::START1;


    uint8_t type = 0;

    uint8_t length = 0;

    uint8_t payload[MAX_PAYLOAD];

    uint8_t index = 0;


    uint8_t crc_low = 0;

    uint8_t crc_high = 0;
};


static Parser parser;


static PacketCallback callback = nullptr;



void init()
{
    parser = Parser{};
}



void set_callback(
    PacketCallback cb)
{
    callback = cb;
}



static bool verify_crc()
{
    uint8_t buffer[4 + MAX_PAYLOAD];


    buffer[0] = START1;
    buffer[1] = START2;
    buffer[2] = parser.type;
    buffer[3] = parser.length;


    memcpy(
        &buffer[4],
        parser.payload,
        parser.length);



    uint16_t crc =
        crc16_ccitt(
            buffer,
            4 + parser.length);



    uint16_t received =
        ((uint16_t)parser.crc_high << 8) |
        parser.crc_low;


    return crc == received;
}



void process_byte(
    uint8_t byte)
{

switch(parser.state)
{


case ParserState::START1:

    if(byte == START1)
        parser.state = ParserState::START2;

    break;



case ParserState::START2:

    if(byte == START2)
        parser.state = ParserState::TYPE;
    else
        parser.state = ParserState::START1;

    break;



case ParserState::TYPE:

    parser.type = byte;

    parser.state =
        ParserState::LENGTH;

    break;



case ParserState::LENGTH:

    parser.length = byte;


    if(parser.length > MAX_PAYLOAD ||
       parser.length == 0)
    {
        parser.state =
            ParserState::START1;
    }
    else
    {
        parser.index = 0;

        parser.state =
            ParserState::PAYLOAD;
    }

    break;



case ParserState::PAYLOAD:

    parser.payload[parser.index++] = byte;


    if(parser.index >= parser.length)
    {
        parser.state =
            ParserState::CRC_LOW;
    }

    break;



case ParserState::CRC_LOW:

    parser.crc_low = byte;

    parser.state =
        ParserState::CRC_HIGH;

    break;



case ParserState::CRC_HIGH:

    parser.crc_high = byte;


    if(verify_crc())
    {
        if(callback)
        {
            callback(
                static_cast<PacketType>(parser.type),
                parser.payload,
                parser.length);
        }
    }


    parser.state =
        ParserState::START1;


    break;

}


}


}
