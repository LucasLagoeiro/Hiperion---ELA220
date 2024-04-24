#ifndef Pins.h
#define Pins.h

int motor1pin1 = 2;
int motor1pin2 = 3;

int motor2pin1 = 4;
int motor2pin2 = 5;

int WHEEL_DIST = 100; // mm
int WHEEL_RADIUS  = 100; // mm

void definePins(){
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(motor2pin1,   OUTPUT);
  pinMode(motor2pin2, OUTPUT);
  pinMode(9,   OUTPUT); 
  pinMode(10, OUTPUT);
}

float diffRobot(int velX, int velZ, int velMotors []){
  float vel = velX;
  float omega = velZ;

  float r_omega = ((vel + omega * WHEEL_DIST / 2) / WHEEL_RADIUS) * 60/(2*3.14159); // RPM 
  float l_omega = ((vel - omega * WHEEL_DIST / 2) / WHEEL_RADIUS) * 60/(2*3.14159); // RPM

  velMotors[0] = r_omega;
  velMotors [1] = l_omega;


}

void setupMotorVel(int velR, int  velL){
  analogWrite(9, 255); //ENA   pin
  analogWrite(10, 255); //ENB pin
  
}

void stopRobot(){

  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, LOW);

  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, LOW);

}


void goFoward(){

  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);

  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, HIGH);

}

void goBackward(){
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, HIGH);

  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);
}

void goLeft(){
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, HIGH);

  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, HIGH);
}

void goRight(){
  digitalWrite(motor1pin1,   HIGH);
  digitalWrite(motor1pin2, LOW);

  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);
}

#endif
