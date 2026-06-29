#include <Arduino.h>
#include "drivers/imu.h"

void setup()
{
    Serial.begin(115200);
    delay(1000);

    imu::init();
    if (!imu::init())
    {
        Serial.println("IMU init failed");
        while (1);
    }

    Serial.println("IMU OK");
}

void loop()
{
    imu::ImuData data = imu::read();

    Serial.print("ax: "); Serial.print(data.ax);
    Serial.print(" ay: "); Serial.print(data.ay);
    Serial.print(" az: "); Serial.print(data.az);

    Serial.print(" | gx: "); Serial.print(data.gx);
    Serial.print(" gy: "); Serial.print(data.gy);
    Serial.print(" gz: "); Serial.println(data.gz);

    delay(50); // ~20 Hz test rate
}
