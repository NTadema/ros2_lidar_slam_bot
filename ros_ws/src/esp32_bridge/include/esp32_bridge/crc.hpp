#pragma once

#include <cstdint>
#include <cstddef>

namespace esp32_bridge
{

class CRC16
{
public:

    static uint16_t calculate(
        const uint8_t* data,
        size_t length
    );

};

} // namespace esp32_bridge
