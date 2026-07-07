#include <Arduino.h>
#include "communication.h"

using namespace uart_comm;

void setup()
{
    Serial.begin(115200);   // debug output
    uart_comm::init();

    Serial.println("UART test start");
}

void loop()
{
    // 1. Send test encoder packet every 1s
    static uint32_t last = 0;

    if (millis() - last > 1000)
    {
        last = millis();

        EncoderPacket pkt;
        pkt.left_ticks = random(-1000, 1000);
        pkt.right_ticks = random(-1000, 1000);

        send_encoder_packet(pkt);

        Serial.println("Sent encoder packet");
    }

    // 2. Read incoming UART bytes and feed parser
    while (uart_config::SerialPort.available())
    {
        uint8_t b = uart_config::SerialPort.read();

        Serial.print("RX byte: ");
        Serial.println(b, HEX);

        uart_comm::process_byte(b);
    }
}
