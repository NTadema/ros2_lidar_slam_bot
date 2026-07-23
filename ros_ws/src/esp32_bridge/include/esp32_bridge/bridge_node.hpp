#ifndef BRIDGE_NODE_HPP
#define BRIDGE_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <vector>
#include <cstdint>

#include "esp32_bridge/uart_linux.hpp"

class UartBridge : public rclcpp::Node
{
public:
    UartBridge();

private:
    // ROS callbacks
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void readSerial();

    // UART functions
    void sendPacket(uint8_t type, const std::vector<uint8_t>& payload);

    // Helper functions
    uint16_t crc16(const std::vector<uint8_t>& data);
    void appendInt16(std::vector<uint8_t>& buffer, int16_t value);
    void appendUint16(std::vector<uint8_t>& buffer, uint16_t value);


    // ROS interfaces
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

// Protocol constants
constexpr uint8_t START1 = 0xAA;
constexpr uint8_t START2 = 0x55;

constexpr uint8_t ENCODER = 1;
constexpr uint8_t MOTOR   = 3;

#endif // BRIDGE_NODE_HPP
