#include "esp32_bridge/bridge_node.hpp"

#include <algorithm>
#include <array>
#include <filesystem>

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
, esp32_time_initialized_(false)
, esp32_time_offset_ns_(0)
, serial_port_(declare_parameter("serial_port", "/dev/ttyUSB0"))
, baud_rate_(declare_parameter("baud_rate", 115200))
, encoder_counts_per_revolution_(
    declare_parameter("encoder_counts_per_revolution", 2024.0))
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
    float v = msg->linear.x;
    float w = msg->angular.z;

    float left_speed =
        v - w * 0.3f / 2.0f;

    float right_speed =
        v + w * 0.3f / 2.0f;

    RCLCPP_INFO(
        get_logger(),
        "Received cmd_vel: v=%.3f w=%.3f -> left=%.3f right=%.3f",
        v,
        w,
        left_speed,
        right_speed
    );

    MotorCommand command{};
    command.left_speed_mps = left_speed;
    command.right_speed_mps = right_speed;

    std::vector<uint8_t> packet =
        serial_protocol_.createMotorPacket(command);

    if (uart_.isOpen())
    {
        uart_.write(packet);
    }
}

rclcpp::Time UartBridge::esp32TimestampToRos(uint32_t timestamp_us)
{
    const int64_t esp_timestamp_ns = static_cast<int64_t>(timestamp_us) * 1000LL;

    if (!esp32_time_initialized_)
    {
        const int64_t ros_now_ns = this->now().nanoseconds();

        esp32_time_offset_ns_ = ros_now_ns - esp_timestamp_ns;

        esp32_time_initialized_ = true;
    }

    const int64_t ros_timestamp_ns = esp_timestamp_ns + esp32_time_offset_ns_;

    return rclcpp::Time(ros_timestamp_ns);
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

        RCLCPP_DEBUG(get_logger(),"RX byte: 0x%02x", static_cast<unsigned>(byte));

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
                joint_state_msg.header.stamp = esp32TimestampToRos(enc.timestamp_us);
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

                imu_msg.header.stamp = esp32TimestampToRos(imu.timestamp_us);
                imu_msg.header.frame_id = "imu_link";

                imu_msg.linear_acceleration.x = imu.ax;
                imu_msg.linear_acceleration.y = imu.ay;
                imu_msg.linear_acceleration.z = imu.az;

                imu_msg.angular_velocity.x = imu.gx;
                imu_msg.angular_velocity.y = imu.gy;
                imu_msg.angular_velocity.z = imu.gz;

                imu_msg.orientation_covariance[0] = -1.0;
                imu_pub_->publish(imu_msg);
            }
            else
            {
                RCLCPP_WARN(get_logger(), "Received unsupported packet type %u", static_cast<unsigned>(type));
            }
        }
    }
}

