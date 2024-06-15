#ifndef MotorSpeedControl.h
#define MotorSpeedControl.h
#include <util/atomic.h>

//all pins must be PWM enabled pin with ~ printed beside them
const int D0=10;//~
const int D1=9;
const int D2=6;
const int D3=5;

int count = 0;

bool CW = true;
bool CW_2 = true;

bool CCW = false;
bool CCW_2 = false;

bool debug = false;



// // Pins
#define ENCA 2
#define ENCB 3
#define ENCA_2 19
#define ENCB_2 18
// #define PWM 5
// #define IN1 6
// #define IN2 7

// globals
long prevT = 0;
long prevT_2 = 0;

int posPrev = 0;
int posPrev_2 = 0;

// Use the "volatile" directive for variables
// used in an interrupt
volatile int pos_i = 0;
volatile int pos_i_2 = 0;

volatile float velocity_i = 0;
volatile float velocity_i_2 = 0;

volatile long prevT_i = 0;
volatile long prevT_i_2 = 0;

float v1Filt = 0;
float v1Filt_2 = 0;

float v1Prev = 0;
float v1Prev_2 = 0;

float v2Filt = 0;
float v2Filt_2 = 0;

float v2Prev = 0;
float v2Prev_2 = 0;

float eintegral = 0;
float eintegral_2 = 0;

void readEncoder(){
  // Read encoder B when ENCA rises
  int b = digitalRead(ENCB);
  int increment = 0;
  if(b>0){
    // If B is high, increment forward
    increment = 1;
  }
  else{
    // Otherwise, increment backward
    increment = -1;
  }
  pos_i = pos_i + increment;

  // Compute velocity with method 2
  long currT = micros();
  float deltaT = ((float) (currT - prevT_i))/1.0e6;
  velocity_i = increment/deltaT;
  prevT_i = currT;
}

void readEncoder_2(){
  // Read encoder B when ENCA rises
  int b_2 = digitalRead(ENCB_2);
  int increment_2 = 0;
  if(b_2>0){
    // If B is high, increment forward
    increment_2 = 1;
  }
  else{
    // Otherwise, increment backward
    increment_2 = -1;
  }
  pos_i_2 = pos_i_2 + increment_2;

  // Compute velocity with method 2
  long currT_2 = micros();
  float deltaT_2 = ((float) (currT_2 - prevT_i_2))/1.0e6;
  velocity_i_2 = increment_2/deltaT_2;
  prevT_i_2 = currT_2;
}

void enablePins() {
//  Serial.begin(115200);

  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);

  pinMode(ENCA_2,INPUT);
  pinMode(ENCB_2,INPUT);

  // pinMode(ENCA,INPUT);
  // pinMode(ENCB,INPUT);
  // pinMode(PWM,OUTPUT);
  // pinMode(IN1,OUTPUT);
  // pinMode(IN2,OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA_2),readEncoder_2,RISING);
}


void setMotorL(bool direction, int correction){
  int pwm = correction;
  
//  int pwm=map(speed, 0, 100, 0, 255);
  if(direction == CW_2)
  {
   analogWrite(D0,pwm);
   analogWrite(D1,LOW);   
  }else{
   analogWrite(D1,pwm);
   analogWrite(D0,LOW);     
  }
  
}

void setMotorR(bool direction, int correction){
  int pwmR = correction;
  
//  int pwm=map(speed, 0, 100, 0, 255);
  if(direction == CW)
  {
   analogWrite(D3,pwmR);
   analogWrite(D2,LOW);   
  }else{
   analogWrite(D2,pwmR);
   analogWrite(D3,LOW);     
  } 
}


void controlVelocity(float vL,float vR) {

  // read the position in an atomic block
  // to avoid potential misreads
  int pos = 0;
  float velocity2 = 0;

  int pos_2 = 0;
  float velocity2_2 = 0;

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    pos = pos_i;
    velocity2 = velocity_i;

    pos_2 = pos_i_2;
    velocity2_2 = velocity_i_2;
  }

  // Compute velocity with method 1
  long currT = micros();
  float deltaT = ((float) (currT-prevT))/1.0e6;
  float velocity1 = (pos - posPrev)/deltaT;
  posPrev = pos;
  prevT = currT;

  // Another encoder
  long currT_2 = micros();
  float deltaT_2 = ((float) (currT_2-prevT_2))/1.0e6;
  float velocity1_2 = (pos_2 - posPrev_2)/deltaT_2;
  posPrev_2 = pos_2;
  prevT_2 = currT_2;

  // Convert count/s to RPM
  float v1 = velocity1/600.0*60.0;
  float v2 = velocity2/600.0*60.0;

  // Another encoder
  float v1_2 = velocity1_2/600.0*60.0;
  float v2_2 = velocity2_2/600.0*60.0;

  // Low-pass filter (25 Hz cutoff)
  v1Filt = 0.854*v1Filt + 0.0728*v1 + 0.0728*v1Prev;
  v1Prev = v1;
  v2Filt = 0.854*v2Filt + 0.0728*v2 + 0.0728*v2Prev;
  v2Prev = v2;

  // Another encoder
  v1Filt_2 = 0.854*v1Filt_2 + 0.0728*v1_2 + 0.0728*v1Prev_2;
  v1Prev_2 = v1_2;
  v2Filt_2 = 0.854*v2Filt_2 + 0.0728*v2_2 + 0.0728*v2Prev_2;
  v2Prev_2 = v2_2;

  // Set a target
  // float vt = 25*(sin(currT/1e6)>0);
  float vt_R = vR;
  float vt_L = vL;
 

  // Compute the control signal u
  float kp = 5;
  float ki = 2;

  float e = vt_R-v1Filt;
  float e_2 = vt_L-v1Filt_2;

  eintegral = eintegral + e*deltaT;
  eintegral_2 = eintegral_2 + e_2*deltaT_2;

  
  float u = kp*e + ki*eintegral;
  float u_2 = kp*e_2 + ki*eintegral_2;

  // Set the motor speed and direction
  int dir = CW;
  if (u<0){
    dir = CCW;
  }

  int dir_2 = CW_2;
  if (u_2<0){
    dir_2 = CCW_2;
  }

  int pwr = (int) fabs(u);
  int pwr_2 = (int) fabs(u_2);

  if(pwr > 255){
    pwr = 255;
  }

  if(pwr_2 > 255){
    pwr_2 = 255;
  }


  setMotorR(dir,pwr);
  setMotorL(dir_2,pwr_2);

//  Serial.print(vR);
//  Serial.print(" ");
//  Serial.print(v1Filt);
//  Serial.print(" ");
//  Serial.print(vL);
//  Serial.print(" ");
//  Serial.print(v1Filt_2);
//  Serial.println();
//  delay(1);

}






#endif
