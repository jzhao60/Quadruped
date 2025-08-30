#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <ESP32Servo.h>

// PCA9685 object (default I2C address 0x40)
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// servo min/max pulse lengths 
#define SERVO_MIN 150  // 0 degrees (out of 4096)
#define SERVO_MAX 600  // 180 degrees (out of 4096)

Servo leg1_s1;
Servo leg1_s2;
Servo leg2_s1;
Servo leg2_s2;
Servo leg3_s1;
Servo leg3_s2;
Servo leg4_s1;
Servo leg4_s2;
Servo leg5_s1;
Servo leg5_s2;
Servo leg6_s1;
Servo leg6_s2;

float theta1;
float theta = 0;
float theta2;
float theta3;
// height off the ground from shaft of servo1
float h = 60.0;
// distance of line away from servo 1
float x = 130.0;
// leg 1 and leg 2
float l1 = 100.0;
float l2 = 150.0;

const int leg1_s1Pin = 15; 
const int leg1_s2Pin = 15; 
const int leg2_s1Pin = 15; 
const int leg2_s2Pin = 15; 
const int leg3_s1Pin = 15; 
const int leg3_s2Pin = 15; 
const int leg4_s1Pin = 15; 
const int leg4_s2Pin = 15; 
const int leg5_s1Pin = 15; 
const int leg5_s2Pin = 15; 
const int leg6_s1Pin = 15; 
const int leg6_s2Pin = 15; 

int servo1_channel = 0;  // servo connected to servo1_channel 0
int servo2_channel = 1;  // servo connected to servo1_channel 1

void setup() {
  Serial.begin(115200);

  pwm.begin();          // initialize PCA9685
  pwm.setPWMFreq(50);   // set frequency to 50 Hz (typical for servos)
  delay(10);
}

void loop() {

  // sweep from 0° to 120°
  for (int angle = 0; angle <= 120; angle++) {
    pwm.setPWM(servo1_channel, 0, angleToPulse(angle));
    pwm.setPWM(servo2_channel, 0, angleToPulse(120 - angle));
    delay(20);  // small delay for smooth motion
  }

  delay(500);

  // sweep back from 120° to 0°
  for (int angle = 120; angle >= 0; angle--) {
    pwm.setPWM(servo1_channel, 0, angleToPulse(angle));
    pwm.setPWM(servo2_channel, 0, angleToPulse(120 - angle));
    delay(20);
  }

  delay(500);
}

void adjustThetas(){
  float thetaRad = theta / (180.0 / PI);
  float L = x / cos(thetaRad);
  float D = sqrt(sq(h) + sq(L));
  float numerator = sq(D) - sq(l1) - sq(l2);
  float denominator = -2.0 * (l1) * (l2);
  float ratio = numerator / denominator;
  ratio = constrain(ratio, -1.0, 1.0);
  float angleRad = acos(ratio);
  theta1 = angleRad * (180.0 / PI);

  numerator = sq(l2) - sq(l1) - sq(D);
  denominator = -2.0 * (l1) * D;
  ratio = numerator / denominator;
  ratio = constrain(ratio, -1.0, 1.0);
  angleRad = acos(ratio);
  theta2 = angleRad * (180.0 / PI);
  
  theta3 = atan(L/h) * (180.0 / PI);;
}

// function to map angle (0–180°) to PCA9685 pulse length
int angleToPulse(int angle) {
  return map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
}