#include <gtest/gtest.h>
#include "esp32_bridge/crc.hpp"

// CRC-16/CCITT-FALSE
// poly=0x1021, init=0xFFFF, refin=false, refout=false, xorout=0x0000
namespace esp32_bridge{

// Test case empty input, verify initial CRC value
TEST(CRCTest, EmptyInput)
{
    uint16_t crc = CRC16::calculate(nullptr, 0);
    EXPECT_EQ(crc, 0xFFFF);
}

// Test case single byte input edge case
TEST(CRCTest, SingleByteZero)
{
    uint8_t data[] = {0x00};
    uint16_t crc = CRC16::calculate(data, sizeof(data));
    EXPECT_EQ(crc, 0xE1F0);
}

// Test case START1 and START2 bytes
TEST(CRCTest, PacketHeaderAA55)
{
    uint8_t data[] = {0xAA, 0x55};
    uint16_t crc = CRC16::calculate(data, sizeof(data));
    EXPECT_EQ(crc, 0xE5EA);
}

// Test case full packet with payload
TEST(CRCTest, FullPacket)
{
    uint8_t data[] = {0xAA, 0x55, 0x01, 0x08};
    uint16_t crc = CRC16::calculate(data, sizeof(data));
    EXPECT_EQ(crc, 0x011A);
}

// Test case standard CRC false validation
TEST(CRCTest, KnownString){
    const char* test_str = "123456789";
    uint16_t crc = CRC16::calculate(reinterpret_cast<const uint8_t*>(test_str), 9);
    EXPECT_EQ(crc, 0x29B1);
}

// Test case repeated pattern
TEST(CRCTest, RepeatedPattern)
{
    uint8_t data[] = {0xAA, 0xAA, 0xAA, 0xAA};
    uint16_t crc = CRC16::calculate(data, sizeof(data));
    EXPECT_EQ(crc, 0x9A55);
}

} // namespace esp32_bridge
