#include <Servo.h>

Servo myServo;  // Create a servo object
Servo myServo2;  // Create a servo object

void setup() {
  myServo.attach(13);  // Connect servo signal wire to pin 9
  myServo2.attach(12);  // Connect servo signal wire to pin 9
  myServo.write(90);
  myServo2.write(90);
  delay(2000);
  for (int pos = 90; pos <= 180; pos++) {
  myServo.write(pos);
  myServo2.write(180-pos);
  delay(30);  // small delay to smooth motion
  }
  for (int pos = 180; pos >= 90; pos--) {
  myServo.write(pos);
  myServo2.write(180-pos);
  delay(30);  // small delay to smooth motion
  }
}

void loop() {
}