#include <Servo.h>

Servo myServo;  // Create a servo object

void setup() {
  myServo.attach(13);  // Connect servo signal wire to pin 9
}

void loop() {
  // Sweep from 0 to 180 degrees
  for (int angle = 0; angle <= 180; angle++) {
    myServo.write(angle);
    delay(10);  // Wait for the servo to move
  }

  // Sweep back from 180 to 0 degrees
  for (int angle = 180; angle >= 0; angle--) {
    myServo.write(angle);
    delay(10);  // Wait for the servo to move
  }
}