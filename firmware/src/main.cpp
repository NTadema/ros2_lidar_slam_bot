#include <Arduino.h>
#include "drivers/motors.h"

// put function declarations here:
int myFunction(int, int);

void setup() {
  motors_init();
}

void loop() {
  set_motors_speed(250, 250);
}
