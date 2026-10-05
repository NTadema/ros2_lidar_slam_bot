#include "communication.hpp"
#include "drivers/encoders.hpp"
#include "drivers/imu.hpp"

void setup()
{
    Serial.begin(115200);
    delay(1000);

    uart_comm::init();
    encoders::init();
    encoders::reset();
    imu::init();
}

void loop()
{
    while (Serial.available())
    {
        uart_comm::process_byte(
            static_cast<uint8_t>(Serial.read()));
    }

    static uint32_t last_tx = 0;

    if (millis() - last_tx >= 50)
    {
        auto encoder_data = encoders::read();

        EncoderPacket encoder_packet;;
        encoder_packet.left_ticks = encoder_data.left_ticks;
        encoder_packet.right_ticks = encoder_data.right_ticks;
        encoder_packet.timestamp_us = encoder_data.timestamp_us;

        uart_comm::send_encoder_packet(encoder_packet);

        auto imu_data = imu::read();
            
        ImuPacket imu_packet;
        imu_packet.ax = imu_data.ax;
        imu_packet.ay = imu_data.ay;
        imu_packet.az = imu_data.az;
        imu_packet.gx = imu_data.gx;
        imu_packet.gy = imu_data.gy;
        imu_packet.gz = imu_data.gz;
        imu_packet.timestamp_us = imu_data.timestamp_us;

        uart_comm::send_imu_packet(imu_packet);

        last_tx = millis();
    }
}
