#include "esp32_bridge/bridge_node.hpp"

#include <algorithm>
#include <array>
#include <filesystem>
#include <sstream>

namespace
{
// Searches for available serial devices to connect
std::vector<std::string> discoverSerialPorts(const std::string& preferred)
{
    std::vector<std::string> candidates;

    // Adds a serial device only if it has not already been added
    auto addCandidate = [&](const std::string& path)
    {
        if (!path.empty() && std::find(candidates.begin(), candidates.end(), path) == candidates.end())
        {
            candidates.push_back(path);
        }
    };

    if (!preferred.empty())
    {
        addCandidate(preferred);
    }

    // Search stable device names under /dev/serial/by-id
    const std::filesystem::path by_id_path("/dev/serial/by-id");
    if (std::filesystem::exists(by_id_path))
    {
        for (const auto& entry : std::filesystem::directory_iterator(by_id_path))
        {
            if (entry.is_symlink() || entry.is_character_file())
            {
                addCandidate(entry.path().string());
            }
        }
    }

    // Search common USB serial device names
    const std::array<std::string, 2> prefixes = {"/dev/ttyUSB", "/dev/ttyACM"};
    for (const auto& prefix : prefixes)
    {
        for (int i = 0; i < 16; ++i)
        {
            addCandidate(prefix + std::to_string(i));
        }
    }

    return candidates;
}
}  // namespace

// UART interface used for communication with the ESP32
UartLinux uart_;

// Initialize the ROS node, UART connection, and ROS interfaces
UartBridge::UartBridge()
: Node("uart_bridge")
, serial_port_(declare_parameter("serial_port", "/dev/ttyUSB0"))
, baud_rate_(declare_parameter("baud_rate", 115200))
, encoder_counts_per_revolution_(declare_parameter("encoder_counts_per_revolution", 2024.0)) // Effective encoder counts per wheel revolution
{
    RCLCPP_INFO(get_logger(), "Opening UART with baud rate %d", baud_rate_);

    const auto candidates = discoverSerialPorts(serial_port_);
    bool opened = false;

    for (const auto& candidate : candidates)
    {
        RCLCPP_INFO(get_logger(), "Trying UART device %s", candidate.c_str());
        if (uart_.open(candidate, baud_rate_))
        {
            serial_port_ = candidate;
            opened = true;
            break;
        }
    }

    // Stop execution if no UART device can be opened
    if (!opened)
    {
        RCLCPP_FATAL(get_logger(), "Unable to open UART device. Tried %s", serial_port_.c_str());
        throw std::runtime_error("UART open failed");
    }

    RCLCPP_INFO(get_logger(), "Opened UART at %s", serial_port_.c_str());

    // Subscribe to robot velocity commands
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel",
        10,
        std::bind(
            &UartBridge::cmdVelCallback,
            this,
            std::placeholders::_1
        )
    );

    // Publish wheel encoder positions as JointState messages
    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(
        "/joint_states",
        10
    );

    // Publish IMU data as Imu messages
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(
        "/imu/data_raw",
        10
    );

    // Periodically check for incoming UART data
    timer_ = create_wall_timer(
        std::chrono::milliseconds(5),
        std::bind(
            &UartBridge::readSerial,
            this
        )
    );
}

void UartBridge::cmdVelCallback(
    const geometry_msgs::msg::Twist::SharedPtr msg)
{
    // Extract linear and angular velocity commands
    float v = msg->linear.x;
    float w = msg->angular.z;

    // Convert robot velocity into left wheel speed
    int16_t left_speed =
        static_cast<int16_t>(v - w * 0.3 / 2.0); // Assuming wheel separation of 0.3 meters

    // Convert robot velocity into right wheel speed
    int16_t right_speed =
        static_cast<int16_t>(v + w * 0.3 / 2.0); // Assuming wheel separation of 0.3 meters

    RCLCPP_INFO(
        get_logger(),
        "Received cmd_vel: v=%.3f w=%.3f -> left=%d right=%d",
        v,
        w,
        left_speed,
        right_speed
    );

    // Build the motor command payload
    std::vector<uint8_t> payload;
    appendInt16(payload, left_speed);
    appendInt16(payload, right_speed);

    // Send motor speeds to the ESP32
    sendPacket(MOTOR, payload);
}

// Store a signed 16-bit value in little-endian byte order
void UartBridge::appendInt16(std::vector<uint8_t> &buffer, int16_t value)
{
    buffer.push_back(value & 0xFF);
    buffer.push_back((value >> 8) & 0xFF);
}

// Store an unsigned 16-bit value in little-endian byte order
void UartBridge::appendUint16(std::vector<uint8_t> &buffer, uint16_t value)
{
    buffer.push_back(value & 0xFF);
    buffer.push_back((value >> 8) & 0xFF);
}

// Build and send a protocol packet over UART
void UartBridge::sendPacket(uint8_t type, const std::vector<uint8_t> &payload)
{
    std::vector<uint8_t> packet;

    // Add packet start markers
    packet.push_back(START1);
    packet.push_back(START2);

    // Store packet type and payload length
    packet.push_back(type);
    packet.push_back(payload.size());

    // Append payload bytes
    packet.insert(packet.end(), payload.begin(), payload.end());

    // Calculate packet checksum
    uint16_t crc = crc16(packet);

    // Append CRC to the end of the packet
    appendUint16(packet, crc);
    
    // Transmit the packet if the UART is available
    if (uart_.isOpen())
    {
        bool ok = uart_.write(packet);


        std::ostringstream oss;
        oss << "Sending UART packet: type=0x" << std::hex << static_cast<int>(type)
            << " len=" << std::dec << packet.size() << " bytes=";

        for (uint8_t byte : packet)
        {
            oss << " 0x" << std::hex << static_cast<int>(byte);
        }

        // Log packet contents for debugging
        if (ok)
        {
            RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());
        }
        else
        {
            RCLCPP_WARN(get_logger(), "%s", oss.str().c_str());
        }
    }
}

// Calculate CRC-16 checksum for packet integrity
uint16_t UartBridge::crc16(const std::vector<uint8_t> &data)
{
    // Initialize CRC accumulator
    uint16_t crc = 0xFFFF;

    // Update CRC with each data byte
    for (uint8_t b : data)
    {
        crc ^= static_cast<uint16_t>(b) << 8;

        // Process each bit using the CRC polynomial
        for (int i = 0; i < 8; i++)
        {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }

    return crc;
}

// Read and process incoming UART data
void UartBridge::readSerial()
{
    // Read all available bytes from the UART buffer
    while (uart_.available())
    {
        uint8_t byte;
        const auto n = uart_.read(&byte, 1);

        // Stop if no more bytes can be read
        if (n <= 0)
        {
            break;
        }

        RCLCPP_INFO(get_logger(), "RX byte: 0x%02x", static_cast<unsigned>(byte));

        // Pass each byte to the packet parser
        if (serial_protocol_.processByte(byte))
        {
            // Process a complete packet once it is received
            const auto type = serial_protocol_.packetType();

            // Handle incoming encoder packets
            if (type == PacketType::ENCODER)
            {
                const auto enc = serial_protocol_.encoderPacket();

                // Create a JointState message to publish wheel positions
                const double left_position = static_cast<double>(enc.left_ticks) / encoder_counts_per_revolution_ * 2.0 * M_PI;
                const double right_position = static_cast<double>(enc.right_ticks) / encoder_counts_per_revolution_ * 2.0 * M_PI;

                sensor_msgs::msg::JointState joint_state_msg;

                // Set the timestamp and joint names for the JointState message
                joint_state_msg.header.stamp = this->now();
                joint_state_msg.name = {"left_wheel_joint", "right_wheel_joint"};
                joint_state_msg.position = {left_position, right_position};

                // Publish the JointState message
                joint_state_pub_->publish(joint_state_msg);

                RCLCPP_INFO(
                    get_logger(),
                    "Encoder: left_ticks=%ld right_ticks=%ld -> left=%.3f rad right=%.3f rad",
                    static_cast<long>(enc.left_ticks),
                    static_cast<long>(enc.right_ticks),
                    left_position,
                    right_position
                );
        
            }

            else if (type == PacketType::IMU)
            {
                const auto imu = serial_protocol_.imuPacket();

                sensor_msgs::msg::Imu imu_msg;

                imu_msg.header.stamp = this->now();
                imu_msg.header.frame_id = "imu_link";

                imu_msg.linear_acceleration.x = imu.ax;
                imu_msg.linear_acceleration.y = imu.ay;
                imu_msg.linear_acceleration.z = imu.az;

                imu_msg.angular_velocity.x = imu.gx;
                imu_msg.angular_velocity.y = imu.gy;
                imu_msg.angular_velocity.z = imu.gz;

                imu_pub_->publish(imu_msg);
            }
            // Ignore unsupported packet types
            else
            {
                RCLCPP_WARN(get_logger(), "Received unsupported packet type %u", static_cast<unsigned>(type));
            }
        }
    }
}

