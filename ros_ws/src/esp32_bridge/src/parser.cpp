#include "esp32_bridge/parser.hpp"
#include "esp32_bridge/crc.hpp"

namespace esp32_bridge
{

Parser::Parser()
{
    reset();
}

void Parser::reset()
{
    state_ = State::WAIT_START1;

    packet_.length = 0;

    payload_index_ = 0;

    received_crc_ = 0;

    crc_buffer_length_ = 0;
}

const Packet& Parser::getPacket() const
{
    return packet_;
}

bool Parser::parseByte(uint8_t byte)
{
    switch (state_)
    {
        case State::WAIT_START1:
        {
            if (byte == START1)
            {
                crc_buffer_[0] = byte;
                crc_buffer_length_ = 1;

                state_ = State::WAIT_START2;
            }

            break;
        }

        case State::WAIT_START2:
        {
            if (byte == START2)
            {
                crc_buffer_[crc_buffer_length_++] = byte;

                state_ = State::READ_TYPE;
            }
            else
            {
                reset();
            }

            break;
        }

        case State::READ_TYPE:
        {
            packet_.type = static_cast<PacketType>(byte);

            crc_buffer_[crc_buffer_length_++] = byte;

            state_ = State::READ_LENGTH;

            break;
        }

        case State::READ_LENGTH:
        {
            packet_.length = byte;

            if (packet_.length > MAX_PAYLOAD_SIZE)
            {
                reset();
                break;
            }

            crc_buffer_[crc_buffer_length_++] = byte;

            payload_index_ = 0;

            if (packet_.length == 0)
            {
                state_ = State::READ_CRC_LOW;
            }
            else
            {
                state_ = State::READ_PAYLOAD;
            }

            break;
        }

        case State::READ_PAYLOAD:
        {
            packet_.payload[payload_index_] = byte;

            crc_buffer_[crc_buffer_length_++] = byte;

            payload_index_++;

            if (payload_index_ >= packet_.length)
            {
                state_ = State::READ_CRC_LOW;
            }

            break;
        }

        case State::READ_CRC_LOW:
        {
            received_crc_ = byte;

            state_ = State::READ_CRC_HIGH;

            break;
        }

        case State::READ_CRC_HIGH:
        {
            received_crc_ |= static_cast<uint16_t>(byte) << 8;

            uint16_t calculated_crc =
                CRC16::calculate(
                    crc_buffer_.data(),
                    crc_buffer_length_);

            if (calculated_crc == received_crc_)
            {
                state_ = State::WAIT_START1;
                return true;
            }

            reset();
            break;
        }
    }

    return false;
}

} // namespace esp32_bridge
