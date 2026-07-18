#include "communication.hpp"

EncoderPacket encoder_data;

void setup()
{
    Serial.begin(115200);        // USB debug serial

    uart_comm::init();           // UART2 to Pi

    Serial.println("ESP32 UART sender started");
}


void loop()
{
    static int32_t ticks = 0;

    encoder_data.left_ticks = ticks;
    encoder_data.right_ticks = ticks + 100;

    if (uart_comm::send_encoder_packet(encoder_data))
    {
        Serial.print("Sent encoder packet: ");
        Serial.print(encoder_data.left_ticks);
        Serial.print(", ");
        Serial.println(encoder_data.right_ticks);
    }
    else
    {
        Serial.println("UART send failed");
    }

    ticks += 10;

    delay(100); // 10Hz packet rate
}
