#include "esp32_bridge/crc.hpp"

// CRC-16/CCITT-FALSE
// poly=0x1021, init=0xFFFF, refin=false, refout=false, xorout=0x0000
namespace esp32_bridge
{

uint16_t CRC16::calculate(
    const uint8_t* data,
    size_t length
)
{
    uint16_t crc = 0xFFFF;

    for (size_t i = 0; i < length; i++)
    {
        crc ^= static_cast<uint16_t>(data[i]) << 8;

        for (int bit = 0; bit < 8; bit++)
        {
            if (crc & 0x8000)
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

} // namespace esp32_bridge
