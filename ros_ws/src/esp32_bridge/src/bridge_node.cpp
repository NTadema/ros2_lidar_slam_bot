#include "esp32_bridge/bridge_node.hpp"

UartLinux uart_;

rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
rclcpp::TimerBase::SharedPtr timer_;

UartBridge::UartBridge()
: Node("uart_bridge")
{
    if (!uart_.open("/dev/ttyUSB0", 115200))
    {
        RCLCPP_FATAL(get_logger(), "Unable to open UART");
        throw std::runtime_error("UART open failed");
    }

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel",
        10,
        std::bind(
            &UartBridge::cmdVelCallback,
            this,
            std::placeholders::_1
        )
    );

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

    int16_t left_speed =
        static_cast<int16_t>(v - w * 0.5);

    int16_t right_speed =
        static_cast<int16_t>(v + w * 0.5);


    std::vector<uint8_t> payload;

    appendInt16(payload, left_speed);
    appendInt16(payload, right_speed);

    sendPacket(MOTOR, payload);
}

void UartBridge::appendInt16(std::vector<uint8_t> &buffer, int16_t value)
{
    buffer.push_back(value & 0xFF);
    buffer.push_back((value >> 8) & 0xFF);
}

void UartBridge::appendUint16(std::vector<uint8_t> &buffer, uint16_t value)
{
    buffer.push_back(value & 0xFF);
    buffer.push_back((value >> 8) & 0xFF);
}

void UartBridge::sendPacket(uint8_t type, const std::vector<uint8_t> &payload)
{
    std::vector<uint8_t> packet;

    packet.push_back(START1);
    packet.push_back(START2);
    packet.push_back(type);
    packet.push_back(payload.size());

    packet.insert(packet.end(), payload.begin(), payload.end());

    uint16_t crc = crc16(packet);

    appendUint16(packet, crc);

    if (uart_.isOpen())
        uart_.write(packet);
}

uint16_t UartBridge::crc16(const std::vector<uint8_t> &data)
{
    uint16_t crc = 0xFFFF;

    for (uint8_t b : data)
    {
        crc ^= static_cast<uint16_t>(b) << 8;

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

void UartBridge::readSerial()
{
    while (uart_.available())
    {
        uint8_t byte;
        uart_.read(&byte, 1);

        // TODO:
        // Implement packet state machine here:
        //
        // WAIT1
        // WAIT2
        // TYPE
        // LENGTH
        // PAYLOAD
        // CRC
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<UartBridge>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
