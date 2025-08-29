#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// PCA9685 object (default I2C address 0x40)
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// servo min/max pulse lengths 
#define SERVO_MIN 150  // 0 degrees (out of 4096)
#define SERVO_MAX 600  // 180 degrees (out of 4096)

// function to map angle (0–180°) to PCA9685 pulse length
int angleToPulse(int angle) {
  return map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
}

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 + PCA9685 Servo Test");

  pwm.begin();          // initialize PCA9685
  pwm.setPWMFreq(50);   // set frequency to 50 Hz (typical for servos)
  delay(10);
}

void loop() {
  int servo1_channel = 0;  // servo connected to servo1_channel 0
  int servo2_channel = 1;  // servo connected to servo1_channel 1

  // sweep from 0° to 120°
  for (int angle = 0; angle <= 120; angle++) {
    pwm.setPWM(servo1_channel, 0, angleToPulse(angle));
    pwm.setPWM(servo2_channel, 0, angleToPulse(120 - angle));
    delay(20);  // small delay for smooth motion
  }

  delay(500);

  // Sweep back from 120° to 0°
  for (int angle = 120; angle >= 0; angle--) {
    pwm.setPWM(servo1_channel, 0, angleToPulse(angle));
    pwm.setPWM(servo2_channel, 0, angleToPulse(120 - angle));
    delay(20);
  }

  delay(500);
}