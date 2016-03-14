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

//Initialize the encoder,IOC7 and PT7for generating clk
void initBMQ(){   
  TCNT = 0x00;
  PACTL= 0xC0;  //Enable PAC
  TIE  = 0x00;  //Disable ISR
  PACNT = 0;
  BMQR_RESET=LOW;
} 

//Since there's only one external counter in the chip so we
//use another counter to measure the encoder on the left motor
void getSpeed(){
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

void calculateSpeedOutput() { //SpeedOutCalculate
  int16_t i;
  float temp=0;
  speed=Output-SpeedControlOut;
  
  //DirectionControlOut=0;//for ccd test only
  LOUT=-(speed+DirectionControlOut);
  ROUT=-(speed-DirectionControlOut);
}

void speedOut(){   //speedout
  if(!MotorEnable)
  return;

 if(fabs(Setpoint-angleFilter2)>45) {
  LOUT=0;
  ROUT=0;
 }
 
 if(LOUT==0){
    PWMDTY01=0;
    PWMDTY23=0;  
  } 
  else if(LOUT>0){ 
    PWMDTY01=0;
    PWMDTY23=LOUT+MotorOffestL;
  } else if(LOUT<0) { 
    PWMDTY23=0;
    PWMDTY01=MotorOffestL-LOUT;
    }

  if(ROUT==0) {
    PWMDTY67=0; 
    PWMDTY45=0;
  } else if(ROUT>0) {
    PWMDTY45=ROUT+MotorOffestR;  
    PWMDTY67=0;
  }else if(ROUT<0){
    PWMDTY45=0; 
    PWMDTY67=MotorOffestR-ROUT;
  }
}