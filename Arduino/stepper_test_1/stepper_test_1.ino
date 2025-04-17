#include <Stepper.h>

// # steps for full 360-degree rotation, change to fit your motor
int stepsPerRevolution = 2048;

// set a speed for the stepper motor
int rpm = 10;

// initialize stepper library on pins 8 - 11
// pin order IN1, IN3, IN2, IN4
Stepper myStepper (stepsPerRevolution, 8, 10, 9, 11);

void setup() {
  myStepper.setSpeed(rpm);
}

void loop() {
  // make a full revolution in one direction
  myStepper.step(stepsPerRevolution);
  delay(500);

  // make a full revolution in the opposite direction
  myStepper.step(-stepsPerRevolution);
  delay(500);
}