#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// PCA9685 object (default I2C address 0x40)
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// servo min/max pulse lengths 
#define SERVO_MIN 150  // 0 degrees (out of 4096)
#define SERVO_MAX 600  // 180 degrees (out of 4096)

int servo1_channel = 0;  // servo connected to servo1_channel 0
int servo2_channel = 1;  // servo connected to servo1_channel 1

Servo servo3;
Servo servo4;

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

const int servo3Pin = 4; 
const int servo4Pin = 16; 

int currentAngle = 90;
int targetAngle = 20;
int lowAngle = 60;
int hiAngle = 140;

void setup() {

  Serial.begin(115200);
  pwm.begin();          // initialize PCA9685
  pwm.setPWMFreq(50);   // set frequency to 50 Hz (typical for servos)
  delay(2000);

  adjustThetas();

  servo3.setPeriodHertz(50);    // Standard servo frequency
  servo4.setPeriodHertz(50);    // Standard servo frequency

  servo3.attach(servo3Pin, 500, 2400); // Pin, min pulse width, max pulse width
  servo4.attach(servo4Pin, 500, 2400);

  Serial.print("theta1: ");
  Serial.print(theta1);
  Serial.println(" degrees");
  Serial.print("theta2: ");
  Serial.print(theta2);
  Serial.println(" degrees");
  Serial.print("theta3: ");
  Serial.print(theta3);
  Serial.println(" degrees");
  
  pwm.setPWM(servo1_channel, 0, angleToPulse(currentAngle));
  pwm.setPWM(servo2_channel, 0, angleToPulse(currentAngle));
  servo3.write(currentAngle); 
  servo4.write(currentAngle); 
  delay(1000);
  servo3.write(90 - (theta3 + theta2 - 90)); 
  servo4.write(90 - (90 - theta1)); 
  delay(1000);
}

void loop() {
  // servo1 back and forth
  if (currentAngle > targetAngle) {
    delay(1000); // wait before changing direction
    for (int pos = currentAngle; pos >= targetAngle; pos--) {
      pwm.setPWM(servo1_channel, 0, angleToPulse(pos));
      pwm.setPWM(servo2_channel, 0, angleToPulse(180-pos));
      theta = pos - 90;
      moveServos();
      delay(30); 
    }
  }
  else{
    delay(1000); // wait before changing direction
    for (int pos = currentAngle; pos <= targetAngle; pos++) {
      pwm.setPWM(servo1_channel, 0, angleToPulse(pos));
      pwm.setPWM(servo2_channel, 0, angleToPulse(180-pos));
      theta = pos - 90;
      moveServos();
      delay(30); // speed (smaller = faster)
    }
  }

  currentAngle = targetAngle;

  //back and forth
  targetAngle = (targetAngle == hiAngle) ? lowAngle : hiAngle;
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

void moveServos(){
  adjustThetas();
  float servo3Ang = 90 - (theta3 + theta2 - 90);
  servo3.write(servo3Ang);
  servo4.write(90 - (90 - theta1));
}

// function to map angle (0–180°) to PCA9685 pulse length
int angleToPulse(int angle) {
  return map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
}