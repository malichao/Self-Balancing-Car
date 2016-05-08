/*
 Copyright (c) <2013-2016> <Malcolm Ma>

 Permission is hereby granted, free of charge, to any person obtaining a copy 
 of this software and associated documentation files (the "Software"), to deal 
 in the Software without restriction, including without limitation the rights 
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell 
 copies of the Software, and to permit persons to whom the Software is 
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE 
 SOFTWARE.
 */

/*********************************************************
Code Warrior 5.0/1
Target : MC9S12XS128
Crystal: 16.000Mhz
busclock:80.000MHz
pllclock:160.000MHz  

Some development notes:
-Speed closed loop control,hit the wall at 1.5m/s with stable result
-Discard the direction judge since the car only runs forward on the track
-Segmentize the speed P coefficient

-Set the CCD view to 45 - 60 cm gets a pretty good result
-Current speed : 31 meter /22 - 28s
-Add track width calculation during running

function execution time:
CCD             | 8bit sampling             | 572us
Acc and Gyro    | sampling and computation  | 784us
PID controller  |                           | 208us

some good settings: 
40cm foresight view @ 68бу,38m-31.5s,stable
40cm foresight view @ 68бу,38m-26s,stable

-Add median filter to CCD data
-Add start line detect,start line has more than 6 edges

********************************************************/

#include "derivative.h"
#include "math.h"
#include "SCI.h"
#include "ADC.h"
#include "IMU.h"
#include "times"
#include "PWM.h"
#include "macros.h"
#include "filters.h"
#include "PIDControllers.h"

extern int16_t CCDDebugSwitch = 0;
extern int16_t CCDDebugSwitch2 = 2;
uint8_t StopCarAtFinish=true;
float DefaultAngle = -71.5;

void sendCCDData(int8_t arr[]) {
  for (int16_t i=0;i<CCD_PIXELS;i++) {
#if defined PRINT_AD          //Send out ADC values for debugging software
    if(arr[i]==0xFF)          //Do not send 0xFF as it's used as ending symbol
      arr[i] = 0xFE;            
    putchar(arr[i]);
#else                         //Or send out binarized values
    if (arr[i] > CCD_THRESHOLD)
      putchar(1);
    else
      putchar(0);
#endif     
  }
  putchar(0xFF);
}

void setInterruptPriorities() {
  INT_CFADDR = 0x70;
  INT_CFDATA5 = 0x05;
  INT_CFADDR = 0xD0;
  INT_CFDATA3 = 0x07;
}

void initHardwares(){
  setBuscClock80MHz();
  setInterruptPriorities();
  initPWM();
  initBMQ(); 
  initUART0();
  initADC();
  initPIT();
  DDRH = 0x00;  //PortH for detecting switches
  EnableInterrupts;
  delayMs(2000);
  AccGyroCalibration();
}

void printOut() {
  putf(getAccAngle());
  putf(getAngle());
  putf(64);               //A reference line
  putf(getLineCenter());
  putf(getSpeed());
  putstr(" \n\r");
}

void main() {
  initHardwares();

  //while(1){speedout ();getspeed();Dly_ms(5);}  //Test motors

  for (;;) {
    if (CCDDebugSwitch == 0)
      printout();
    if (CCDDebugSwitch == 1)
      sendCCDData(CCDBuf);
    //If cross the finish line,the car stop there to wait for pick up
    if (reachedEnd() == true) {
      if (StopCarAtFinish == true) {      
        motorOff();
        while (StopCarAtFinish == true);  
      }
    }
  }
}


void checkSwitch() {
  static uint8_t lastSwitch;
  uint8_t temp,switchValue;

  float setSpeedMM;

  //reading switches signal
  switchValue = PTH;
  if(switchValue==lastSwitch)  //If nothing new detected,return
    return;
  lastSwitch=switchValue;

  //bot0 for determine if the car stops after crossing the finish line
  StopCarAtFinish = switchValue & 0b00000001;

  //bit1 - bit3 for setting the speed
  temp1 = temp >> 3 & 0b00000111;
  if (temp1 == 0)
    setSpeedMM = 0;
  else
    setSpeedMM = -1400 + temp1 * 200 * -1;  //steady state :1800

  //bit6 for decreasing the car pitch angle
  temp1 = temp >> 6 & 0b0000001;
  if (temp1 != temp2) {
    DefaultAngle -= 1;
    temp2 = temp1;
  }
  //bit7 for increasing the car pitch angle
  temp1 = temp >> 7 & 0b0000001;
  if (temp1 != temp3) {
    DefaultAngle += 1;
    temp3 = temp1;
  }

  setTarget(setSpeedMM,DefaultAngle);
}

/*
 * Timer ISR
 * This ISR is called at 5ms interval.All sensors measurement,PID controller
 * calculation,and motor drive are called one by one in the timer ISR.All functions
 * execution time are tested to be smaller than 5ms.Also the linear CCD requires a
 * precise 5ms exposure time and the gyroscopes require short integral time,thus
 * they are called in the ISR to keep the timing neat.
 *
 * Note that although all the functions are called every time but not all of them
 * are performed.e.g.,speed PID controller executes every 50ms and direction PID
 * execute every 20ms.An internal counter takes care of this timing requirement.
 * But for other functions,they should be called as often as possible.e.g.,updateMotor()
 * checks if the car is tilting too much and if so it will turn off the motor to
 * protect the car.
 */
#pragma CODE_SEG __NEAR_SEG NON_BANKED
void interrupt 67 PIT1(){
  PITTF_PTF0 = 1;                   //clear interrupts flag
  PORTA_PA7=1;

  checkSwitch();                   //Test different setting using switches        
  updateMotor();                   //Read encoders,update PWM,check safety problem
  updateCCD();                    //Read CCD data,calculate track information
  updateIMU();                    //Read gyro and accelerometer data,fuse data
  
  int16_t outputL=0,outputR=0;
  PIDControl(&outputL,&outputR);  //Balance control,speed control,steering control
  setSpeed(outputL,outputR);      //Set the new speed for the car
  updateMotor();                  //Read encoders,update PWM,check safety problem
}
#pragma CODE_SEG DEFAULT         

