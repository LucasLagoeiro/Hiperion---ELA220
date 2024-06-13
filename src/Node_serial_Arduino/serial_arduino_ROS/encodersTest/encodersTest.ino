// This alternate version of the code does not require
// atomic.h. Instead, interrupts() and noInterrupts() 
// are used. Please use this code if your 
// platform does not support ATOMIC_BLOCK.
#include <Encoder.h> 

//all pins must be PWM enabled pin with ~ printed beside them
const int D0=10;//~
const int D1=9;
const int D2=6;
const int D3=5;

int count = 0;

bool CW = true;
bool CCW = false;

bool debug = false;

Encoder motorEncoder(3,2);
//
#define ENCA 2 // Green
#define ENCB 3 // Yellow
//
//
//#define ENCA 2 // YELLOW
//#define ENCB 3 // WHITE
//#define PWM 5
//#define IN2 6
//#define IN1 7

volatile int posi = 0; // specify posi as volatile: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
long prevT = 0;
float eprev = 0;
float eintegral = 0;

void setup() {
  Serial.begin(9600);
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
//  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
  
//  pinMode(PWM,OUTPUT);
//  pinMode(IN1,OUTPUT);
//  pinMode(IN2,OUTPUT);
  
  Serial.println("target pos");
}

void loop() {

  // set target position
  int target = 20000;
//  int target = 250*sin(prevT/1e6);

  // PID constants
  float kp = 1;
  float kd = 0.025;
  float ki = 0.00;

  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;

  // Read the position
  int pos = 0; 
  noInterrupts(); // disable interrupts temporarily while reading
  pos = motorEncoder.read();
  interrupts(); // turn interrupts back on
  
  // error
  int e = pos - target;

  // derivative
  float dedt = (e-eprev)/(deltaT);

  // integral
  eintegral = eintegral + e*deltaT;

  // control signal
  float u = kp*e + kd*dedt + ki*eintegral;

  // motor power
  float pwr = fabs(u);
  if( pwr > 255 ){
    pwr = 255;
  }

  // motor direction
  bool dir = CW;
  if(u<0){
    dir = CCW;
  }

  // signal the motor
  setMotor(dir,pwr);


  // store previous error
  eprev = e;

  Serial.print(target);
  Serial.print(" ");
  Serial.print(pos);
  Serial.print(" ");
  Serial.print(pwr);
  Serial.println();
}

void setMotor(bool direction, int pwr){
  int pwm = pwr;
  
//  int pwm=map(speed, 0, 100, 0, 255);
  if(direction == CW)
  {
   analogWrite(D0,pwm);
   analogWrite(D1,LOW);   
  }else{
   analogWrite(D1,pwm);
   analogWrite(D0,LOW);     
  }
  
}

//void readEncoder(){
//  int b = digitalRead(ENCB);
//  if(b > 0){
//    posi++;
//  }
//  else{
//    posi--;
//  }
//}
