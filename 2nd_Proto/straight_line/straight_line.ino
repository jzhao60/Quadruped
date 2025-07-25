#include <Servo.h>

Servo servo1;
Servo servo2;
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

// for servo1 rotation
int currentAngle = 90;
int targetAngle = 135;

void setup() {
  adjustThetas();
  Serial.begin(9600);
  /*
  Serial.print("theta1: ");
  Serial.print(theta1);
  Serial.println(" degrees");
  Serial.print("theta2: ");
  Serial.print(theta2);
  Serial.println(" degrees");
  Serial.print("theta3: ");
  Serial.print(theta3);
  Serial.println(" degrees");
  */
  
  servo1.attach(13);
  servo2.attach(12);
  servo3.attach(11);
  servo4.attach(10);
  servo1.write(90); 
  servo2.write(90); 
  servo3.write(90); 
  servo4.write(90); 
  delay(1000);
  servo3.write(90 - (theta3 + theta2 - 90)); 
  servo4.write(90 - (90 - theta1)); 
  delay(1000);
}

void loop() {
  // servo1 back and forth
  if (currentAngle < targetAngle) {
    for (int pos = currentAngle; pos <= targetAngle; pos++) {
      servo1.write(pos);
      servo2.write(180 - pos);
      theta = pos - 90;
      moveServos();
      delay(50); // speed (smaller = faster)
    }
  }
  else{
    for (int pos = currentAngle; pos >= targetAngle; pos--) {
      servo1.write(pos);
      servo2.write(180 - pos);
      theta = pos - 90;
      moveServos();
      delay(50); // speed (smaller = faster)
    }
  }

  currentAngle = targetAngle;

  //back and forth
  targetAngle = (targetAngle == 135) ? 90 : 135;
  delay(1000); // wait before changing direction
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