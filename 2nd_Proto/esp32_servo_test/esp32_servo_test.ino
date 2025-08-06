#include <ESP32Servo.h>

Servo servo1;
int servoPin = 4;

void setup() {
  Serial.begin(115200);
  servo1.setPeriodHertz(50);          // Set standard servo PWM frequency
  servo1.attach(servoPin, 500, 2400); // Attach with custom pulse width range
  servo1.write(90);                   // Center position
}

void loop() {
  // Sweep
  for (int pos = 0; pos <= 180; pos++) {
    servo1.write(pos);
    delay(10);
  }

  for (int pos = 180; pos >= 0; pos--) {
    servo1.write(pos);
    delay(10);
  }
}