#include "communication.hpp"
#include "drivers/encoders.hpp"

EncoderPacket encoder_data;

void setup()
{
    Serial.begin(115200);        // USB debug serial

    uart_comm::init();           // UART2 to Pi

    Serial.println("ESP32 UART sender started");
}


void loop()
{
    while (uart_config::SerialPort.available())
    {
        uart_comm::process_byte(uart_config::SerialPort.read());
    }

    static uint32_t last_tx = 0;
    if (millis() - last_tx >= 50)   // 20 Hz
    {
        EncoderPacket pkt;
        pkt.left_ticks = encoders::get_left_ticks();
        pkt.right_ticks = encoders::get_right_ticks();

        uart_comm::send_encoder_packet(pkt);
        last_tx = millis();
    }
}
