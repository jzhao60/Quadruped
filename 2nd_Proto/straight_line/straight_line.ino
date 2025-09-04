#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// PCA9685 object (default I2C address 0x40)
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// servo min/max pulse lengths 
#define SERVO_MIN 150  // 0 degrees (out of 4096)
#define SERVO_MAX 600  // 180 degrees (out of 4096)

int servo1_channel = 0;  // servo connected to servo1_channel 0
int servo2_channel = 1;  
int servo5_channel = 4;  
int servo6_channel = 5;  
int servo7_channel = 6;  
int servo8_channel = 7; 
int servo9_channel = 8;  
int servo10_channel = 9;  
int servo11_channel = 10;  
int servo12_channel = 11;  

Servo l1_s1;
Servo l1_s2;
Servo l2_s1;
Servo l2_s2;
Servo l3_s1;
Servo l3_s2;
Servo l4_s1;
Servo l4_s2;
Servo l5_s1;
Servo l5_s2;
Servo l6_s1;
Servo l6_s2;

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

const int l1_s1Pin = 2; 
const int l1_s2Pin = 4; 
const int l2_s1Pin = 16; 
const int l2_s2Pin = 17; 
const int l3_s1Pin = 5; 
const int l3_s2Pin = 18; 
const int l4_s1Pin = 13; 
const int l4_s2Pin = 12; 
const int l5_s1Pin = 14; 
const int l5_s2Pin = 27; 
const int l6_s1Pin = 26; 
const int l6_s2Pin = 25; 

int currentAngle = 90;
int angleDiffMax = 30;
int targetAngle = 70;
int lowAngle = 70;
int hiAngle = 120;

void setup() {

  Serial.begin(115200);
  pwm.begin();          // initialize PCA9685
  pwm.setPWMFreq(50);   // set frequency to 50 Hz (typical for servos)
  delay(10000); // let capacitors charge up

  adjustThetas();

  l1_s1.setPeriodHertz(50);    // Standard servo frequency
  l1_s2.setPeriodHertz(50);    
  l2_s1.setPeriodHertz(50);  
  l2_s2.setPeriodHertz(50);  
  l3_s1.setPeriodHertz(50);  
  l3_s2.setPeriodHertz(50);  
  l4_s1.setPeriodHertz(50);  
  l4_s2.setPeriodHertz(50);  
  l5_s1.setPeriodHertz(50);  
  l5_s2.setPeriodHertz(50);  
  l6_s1.setPeriodHertz(50);  
  l6_s2.setPeriodHertz(50);  

  l1_s1.attach(l1_s1Pin, 500, 2400); // Pin, min pulse width, max pulse width
  l1_s2.attach(l1_s2Pin, 500, 2400);
  l2_s1.attach(l2_s1Pin, 500, 2400); 
  l2_s2.attach(l2_s2Pin, 500, 2400);
  l3_s1.attach(l3_s1Pin, 500, 2400); 
  l3_s2.attach(l3_s2Pin, 500, 2400);
  l4_s1.attach(l4_s1Pin, 500, 2400); 
  l4_s2.attach(l4_s2Pin, 500, 2400);
  l5_s1.attach(l5_s1Pin, 500, 2400); 
  l5_s2.attach(l5_s2Pin, 500, 2400);
  l6_s1.attach(l6_s1Pin, 500, 2400); 
  l6_s2.attach(l6_s2Pin, 500, 2400);

  Serial.print("theta1: ");
  Serial.print(theta1);
  Serial.println(" degrees");
  Serial.print("theta2: ");
  Serial.print(theta2);
  Serial.println(" degrees");
  Serial.print("theta3: ");
  Serial.print(theta3);
  Serial.println(" degrees");
  
  delay(1000);
  pwm.setPWM(servo1_channel, 0, angleToPulse(currentAngle));
  pwm.setPWM(servo2_channel, 0, angleToPulse(currentAngle));
  l1_s1.write(currentAngle); 
  l1_s2.write(currentAngle); 
  l2_s1.write(currentAngle); 
  l2_s2.write(currentAngle); 
  l3_s1.write(currentAngle); 
  l3_s2.write(currentAngle); 
  l4_s1.write(currentAngle); 
  l4_s2.write(currentAngle); 
  l5_s1.write(currentAngle); 
  l5_s2.write(currentAngle); 
  l6_s1.write(currentAngle); 
  l6_s2.write(currentAngle); 
  delay(3000);
  l1_s1.write(90 - (theta3 + theta2 - 90)); 
  l1_s2.write(90 - (90 - theta1)); 
  delay(1000);
}

void loop() {
  // servo1 back and forth
  if (currentAngle > targetAngle) {
    delay(1000); // wait before changing direction
    for (int pos = currentAngle; pos >= targetAngle; pos--) {

      pwm.setPWM(servo1_channel, 0, angleToPulse(pos));
      pwm.setPWM(servo2_channel, 0, angleToPulse(180-pos));
      pwm.setPWM(servo5_channel, 0, angleToPulse(180-pos));
      pwm.setPWM(servo6_channel, 0, angleToPulse(pos));
      pwm.setPWM(servo9_channel, 0, angleToPulse(pos));
      pwm.setPWM(servo10_channel, 0, angleToPulse(180-pos));

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
      pwm.setPWM(servo5_channel, 0, angleToPulse(180-pos));
      pwm.setPWM(servo6_channel, 0, angleToPulse(pos));
      pwm.setPWM(servo9_channel, 0, angleToPulse(pos));
      pwm.setPWM(servo10_channel, 0, angleToPulse(180-pos));

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
  l1_s1.write(servo3Ang);
  l1_s2.write(90 - (90 - theta1));
}

void middleLegCal(){
  float midY = x * tan(angleDiffMax);
  float midX = (midY / 2) / tan(angleDiffMax / 2);
}

// function to map angle (0–180°) to PCA9685 pulse length
int angleToPulse(int angle) {
  return map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
}