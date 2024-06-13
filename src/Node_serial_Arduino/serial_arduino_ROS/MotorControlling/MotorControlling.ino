 /*
/*
 * Lesson 102: Using ZK-5DA to control 2 DC Motor each 4A
 * 
 * Full video details: https://youtu.be/W_Wm28nQAYA
 * Video timing
00:00 Introduction
03:16 Datashet viewed
06:34 Wiring Explained
08:55 Code Explained
14:28 Demonstration: Motor Control
17:22 Demonstration: Maximum current test
23:23 Voltage Drop test at 3A, 4A and 5A
26:28 Conclusion remarks

 * Written by Ahmad Shamshiri for Arduino Step by Step Course by Robojax
 * www.Robojax.com
 * on Mar 22, 2022 

 * 
 * This code is part of Arduino Step by Step Course which starts here:  https://youtu.be/-6qSrDUA5a8
 * 
 * for library of this code visit http://robojax.com/
 * 
If you found this tutorial helpful, please support me so I can continue creating 
content like this. Make a donation using PayPal by credit card https://bit.ly/donate-robojax 
 * This code is "AS IS" without warranty or liability. Free to be used as long as you keep this note intact.* 
 * This code has been download from Robojax.com
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

 
 //all pins must be PWM enabled pin with ~ printed beside them
const int D0=10;//~
const int D1=9;
const int D2=6;
const int D3=5;

bool CW = true;
bool CCW = false;


bool debug = false;

void setup() {
  Serial.begin(9600);
  Serial.println("Robojax TA6586 Motor Control");
 pinMode(D0, OUTPUT);
 pinMode(D1, OUTPUT);
 pinMode(D2, OUTPUT);
 pinMode(D3, OUTPUT);  
 
 
}

void loop() {
  // * Full video details: https://youtu.be/W_Wm28nQAYA
  M2(CW, 80);//motor 1 runs CW at 80% speed
  M1(CW, 100); //motor 1 runs CW at 100% speed
  delay(3000);//wait for 3 seconds
  brake (2);//apply brake to motor 2
  delay(1000);  
  
  M2(CCW, 60);//motor 2 runs CCW at 60% speed
  delay(3000);//wait for 3 seconds
  brake (2);//apply brake to motor 2
  brake (1);//apply brake to motor 1
  delay(3000);//wait for 3 seconds
  for(int i=0; i<=100; i++)
  {
    M1(CCW, i);//run motor 1 to CCW direction with i% speed
    delay(100);
  }
  delay(2000);
  brake (2);//apply brake to motor 2
  delay(3000);  delay(3000);//wait for 3 seconds}
}//loop ends

/*
 *  M1(bool direction,int speed)
 * @brief runs motor 1 
 * @param direction is CW or CCW, 
 * @param speed is integer between 0 to 100

 * @return returns non
 * Written by Ahmad Shamshiri for robojax.com
 * on Mar 22, 2022 
 * Full video details: https://youtu.be/W_Wm28nQAYA
 */
void M1(bool direction,int speed)
{
  int pwm=map(speed, 0, 100, 0, 255);
  if(direction == CW)
  {
   analogWrite(D0,pwm);
   analogWrite(D1,LOW);   
  }else{
   analogWrite(D1,pwm);
   analogWrite(D0,LOW);     
  }
  debugPrint(1, direction, speed, false); 
}//M1 end


/*
 *  M2(bool direction,int speed)
 * @brief runs motor 2 
 * @param direction is CW or CCW, 
 * @param speed is integer between 0 to 100

 * @return returns non
 * Written by Ahmad Shamshiri for robojax.com
 * on Mar 22, 2022 
 * Full video details: https://youtu.be/W_Wm28nQAYA
 */
void M2(bool direction,int speed)
{
  int pwm=map(speed, 0, 100, 0, 255);
  if(direction == CW)
  {
   analogWrite(D2,pwm);
   analogWrite(D3,LOW);   
  }else{
   analogWrite(D3,pwm);
   analogWrite(D2,LOW);     
  } 
  debugPrint(2, direction, speed, false);    
}//M2 ends


/*
 *  brake(int motor)
 * @brief applies brake to a motor
 * @param "motor" is integer 1 or 2

 * @return returns none
 * Written by Ahmad Shamshiri for robojax.com
 * on Mar 22, 2022 
 * Full video details: https://youtu.be/W_Wm28nQAYA
 */
void brake(int motor)
{
   if(motor == 1)
  {
   analogWrite(D0,HIGH);
   analogWrite(D1,HIGH);   
  }else{
   analogWrite(D2,HIGH);
   analogWrite(D3,HIGH);     
  }
  debugPrint(motor, true,  0, true);  
}//brake ends


/*
 * debugPrint(int motor, bool direction, int speed, bool stop)
 * @brief prints debugging information
 * @param "motor" is integer 1 or 2
 * @param "direction" is CW or CCW
 * @param "speed" is 0 to 100
 * @param "stop" is true or false, if true, the word "stop" is printed

 * @return returns none
 * Written by Ahmad Shamshiri for robojax.com
 * on Mar 22, 2022 
 * Full video details: https://youtu.be/W_Wm28nQAYA
 */
void debugPrint(int motor, bool direction, int speed, bool stop)
{
  if(debug)
  {
      Serial.print("Motor: ");
      Serial.print(motor);
    if(stop && motor>0)
    {
      Serial.println(" Stopped");
    }else{
      if(direction)
      {
      Serial.print(" CW at ");
      }else{
       Serial.print(" CCW at ");     
      }
      Serial.print(speed);       
      Serial.println(" %");    
    }
  }//debug
  
}

//not used but can be used to apply brake on both motors
void fullBrake()
{
 brake(1);
 brake(2); 

}
