#include "communication.hpp"
#include "drivers/encoders.hpp"

EncoderPacket encoder_data;

void setup()
{
    Serial.begin(115200);      // USB serial to your PC

    uart_comm::init();         // USB serial for PC development
    encoders::init();         // Initialize encoder pin modes and interrupts
    encoders::reset();        // Clear any startup noise
    Serial.println("ESP32 UART sender started");
}

void loop()
{
    while (Serial.available())
    {
        uart_comm::process_byte(static_cast<uint8_t>(Serial.read()));
    }

    static uint32_t last_tx = 0;
    if (millis() - last_tx >= 50)
    {
        auto left = encoders::get_left_ticks();
        auto right = encoders::get_right_ticks();

        Serial.printf("raw encoder ticks: left=%ld right=%ld\n",
                      (long)left, (long)right);

        EncoderPacket pkt;
        pkt.left_ticks = left;
        pkt.right_ticks = right;
        Serial.printf("sending encoder packet: left=%ld right=%ld\n",
                      (long)pkt.left_ticks, (long)pkt.right_ticks);
        uart_comm::send_encoder_packet(pkt);
        last_tx = millis();
    }
}
