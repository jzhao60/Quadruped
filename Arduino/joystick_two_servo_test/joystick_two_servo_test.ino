#include <Servo.h>

// Create servo objects
Servo servoX;
Servo servoY;

// Joystick input pins
const int joyX = A0;
const int joyY = A1;

// Servo output pins
const int servoXPin = 12;
const int servoYPin = 13;

void setup() {
  servoX.attach(servoXPin);
  servoY.attach(servoYPin);
}

void loop() {
  // Read joystick values (0 to 1023)
  int xVal = analogRead(joyX);
  int yVal = analogRead(joyY);

  // Map joystick values to servo angle (0 to 180 degrees)
  int xAngle = map(xVal, 0, 1023, 0, 180);
  int yAngle = map(yVal, 0, 1023, 0, 180);

  // Move the servos
  servoX.write(xAngle);
  servoY.write(yAngle);

  delay(15); // Small delay for smooth motion
}