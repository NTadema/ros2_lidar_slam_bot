#pragma once

#include <cstdint>
#include <cstddef>
#include <array>

#include "esp32_bridge/communication.hpp"

namespace esp32_bridge
{

class Parser
{
public:
    Parser();

    bool parseByte(uint8_t byte);

    const Packet& getPacket() const;

    void reset();

private:

    enum class State
    {
        WAIT_START1,
        WAIT_START2,
        READ_TYPE,
        READ_LENGTH,
        READ_PAYLOAD,
        READ_CRC_LOW,
        READ_CRC_HIGH
    };

    State state_;

    Packet packet_;

    uint8_t payload_index_;

    uint16_t received_crc_;

    std::array<uint8_t, MAX_PAYLOAD_SIZE> crc_buffer_;
    size_t crc_buffer_length_;
};

} // namespace esp32_bridge
