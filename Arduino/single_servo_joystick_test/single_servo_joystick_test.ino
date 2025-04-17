#include <Servo.h>

// Create servo objects
Servo servoX;

// Joystick input pins
const int joyX = A0;

// Servo output pins
const int servoXPin = 13;

void setup() {
  servoX.attach(servoXPin);
}

void loop() {
  // Read joystick values (0 to 1023)
  int xVal = analogRead(joyX);

  // Map joystick values to servo angle (0 to 180 degrees)
  int xAngle = map(xVal, 0, 1023, 0, 180);

  // Move the servos
  servoX.write(xAngle);

  delay(15); // Small delay for smooth motion
}