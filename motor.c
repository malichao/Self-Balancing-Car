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

#include "times"
#include "macros.h"
#include "PWM.h"
#include "motor.h"

 //Testing values of encoder R=1139 1119 1120.5  1121  L= 1118.5 1105 1119   avr=1120
 //Wheel diameter 50.5mm  perimeter=158.65mm
 //MMperPulse=158.65/1120=0.14165
 //PulseperMM=7.05965
static int16_t OutputLeft, OutputRight;
static int16_t MotorOffestL = 1900;          //Left Motor PWM output offset
static int16_t MotorOffestR = 1800;          //Right Motor PWM output offset
static int8_t MotorEnable=true;

//Initialize the encoder,IOC7 and PT7for generating clk
void initBMQ(){   
  TCNT = 0x00;
  PACTL= 0xC0;  //Enable PAC
  TIE  = 0x00;  //Disable ISR
  PACNT = 0;
  BMQR_RESET=LOW;
  DDRB = 0X00;  //PortB for external counter input
} 

//Since there's only one external counter in the chip so we
//use another counter to measure the encoder on the left motor.
//This function should be called as often as possible so that the
//external counter won't overflow.
void measureSpeed(){
  DisableInterrupts;
  int tempL,tempR;
  tempL=PORTB;       //Read the external encoder counter
  tempR=PACNT;       //Read the internal counter
  PACNT=0;
  BMQR_RESET=HIGH;
  delayUs(1);
  BMQR_RESET=HIGH;

  SpeedL=SpeedL+tempL;
  SpeedR=SpeedR+tempR;
  EnableInterrupts;
}

void getSpeed(int16_t *speedL,int16_t *speedR){
  *speedL=SpeedL;
  *speedR=SpeedR;
}

int16_t getSpeedAll(){
  return (SpeedR+SpeedL)/2;
}

void setSpeed(float myspeed) {
  SetSpeed = myspeed * PulseperMM * scPeriod * 5 / 1000;
  SetSpeed = 0;
}

void shutDown() {
    MotorEnable=false;
    PWMDTY01=0;
    PWMDTY23=0;
    PWMDTY67=0; 
    PWMDTY45=0;
}

void turnOn(){
   MotorEnable=true;
}


void speedOut(){   //speedout
  if(!MotorEnable)
  return;

 if(fabs(getAngle())>MAX_ANGLE) {
  OutputLeft=0;
  OutputRight=0;
 }
 
 if(OutputLeft==0){
    PWMDTY01=0;
    PWMDTY23=0;  
  } 
  else if(OutputLeft>0){ 
    PWMDTY01=0;
    PWMDTY23=OutputLeft+MotorOffestL;
  } else if(OutputLeft<0) { 
    PWMDTY23=0;
    PWMDTY01=MotorOffestL-OutputLeft;
    }

  if(OutputRight==0) {
    PWMDTY67=0; 
    PWMDTY45=0;
  } else if(OutputRight>0) {
    PWMDTY45=OutputRight+MotorOffestR;  
    PWMDTY67=0;
  }else if(OutputRight<0){
    PWMDTY45=0; 
    PWMDTY67=MotorOffestR-OutputRight;
  }
}