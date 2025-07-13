#include <Servo.h>

Servo servo1;
Servo servo2;
Servo servo3;
float theta1;
float theta2 = 0;
float theta3;
float theta4;
float h = 45.0;
float x = 50.0;
float l1 = 40.0;
float l2 = 62.0;

// for servo1 rotation
int currentAngle = 90;
int targetAngle = 135;

void setup() {
  adjustThetas();
  Serial.begin(9600);
  Serial.print("theta1: ");
  Serial.print(theta1);
  Serial.println(" degrees");

  servo1.attach(13);
  servo2.attach(12);
  servo3.attach(11);
  servo1.write(90); 
  servo2.write(90); 
  servo3.write(90); 
  delay(1000);
  servo3.write(90-(90 - theta1));
  servo2.write(90 - 22.93);
  delay(1000);
}

void loop() {
  if (currentAngle < targetAngle) {
    for (int pos = currentAngle; pos <= targetAngle; pos++) {
      servo1.write(pos);
      theta2 = pos - 90;
      moveServos();
      delay(20); // Adjust this delay for speed (smaller = faster)
    }
  }
  else{
    for (int pos = currentAngle; pos >= targetAngle; pos--) {
      servo1.write(pos);
      theta2 = pos - 90;
      moveServos();
      delay(20); // Adjust this delay for speed (smaller = faster)
    }
  }

  currentAngle = targetAngle;

  // Set a new target to go back and forth
  targetAngle = (targetAngle == 135) ? 45 : 135;
  delay(1000); // Wait before changing direction
}

void adjustThetas(){
  float theta2Rad = theta2 / (180.0 / PI);
  float L = x / cos(theta2Rad);
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
  theta3 = angleRad * (180.0 / PI);
  
  theta4 = atan(L/h) * (180.0 / PI);;
}

void moveServos(){
  adjustThetas();
  float servo2Ang = theta3 + theta4 - 90;
  servo2.write(90 - servo2Ang);
  servo3.write(90-(90 - theta1));
}