#include <Arduino.h>
#include "drivers/motors.h"
#include "drivers/encoders.h"

void setup() {
    Serial.begin(115200);
    motors::init();
    encoders::init();
}

void loop() {
    motors::set_speed(250,250);
    Serial.println("Left ticks: ");
    Serial.println(encoders::get_left_ticks());
    Serial.println("Right ticks: ");
    Serial.println(encoders::get_right_ticks());
    delay(100);
}
