// Prevent multiple inclusion of this header file
#ifndef BRIDGE_NODE_HPP
#define BRIDGE_NODE_HPP

// ROS 2 core library
// Velocity command message definition
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <vector>
#include <cstdint>
#include <string>

#include "esp32_bridge/uart_linux.hpp"
#include "esp32_bridge/serial_protocol.hpp"

// ROS 2 node that bridges UART communication with the ESP32
class UartBridge : public rclcpp::Node
{
// Constructor that initializes UART and ROS interfaces
public:
    UartBridge();

private:
    // ROS callback functions
    // Processes incoming velocity commands from ROS
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

    // Reads and processes incoming UART data.
    void readSerial();

    // Time synchronization and offset with the ESP32
    bool esp32_time_initialized_;
    int64_t esp32_time_offset_ns_;

    // ROS interfaces
    // Subscriber for receiving robot velocity commands
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;

    // Publisher for wheel encoder positions
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

    // Publisher for IMU data
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

    // Periodic timer for polling UART data
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Time esp32TimestampToRos(uint32_t timestamp_us);

    // UART device path
    std::string serial_port_;

    // UART communication speed
    int baud_rate_;

    // Effective encoder counts per wheel revolution
    double encoder_counts_per_revolution_;

    // Packet parser for incoming serial data
    SerialProtocol serial_protocol_;
};

#endif // BRIDGE_NODE_HPP
