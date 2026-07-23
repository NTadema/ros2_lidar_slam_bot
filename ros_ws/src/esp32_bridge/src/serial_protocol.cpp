#include <cstdint>
#include <vector>

#include "esp32_bridge/serial_protocol.hpp"

std::vector<uint8_t> SerialProtocol::createMotorPacket(
        const MotorCommand& cmd)
{
    std::vector<uint8_t> packet;

    packet.push_back(0xAA);
    packet.push_back(0x55);

    packet.push_back(static_cast<uint8_t>(PacketType::MOTOR));

    packet.push_back(sizeof(MotorCommand));

    packet.push_back(cmd.left_speed & 0xFF);
    packet.push_back((cmd.left_speed >> 8) & 0xFF);

    packet.push_back(cmd.right_speed & 0xFF);
    packet.push_back((cmd.right_speed >> 8) & 0xFF);

    uint16_t crc = crc16(packet);

    packet.push_back(crc & 0xFF);
    packet.push_back((crc >> 8) & 0xFF);

    return packet;
}
