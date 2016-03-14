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
 
 
 速度闭环，测试1.5m/s撞墙不倒
 去掉了速度前后方向判断，只能前进不能后退
 原来官方的方向判断方法有问题，会导致速度不稳定
 速度控制分段P

 前后式CCD测试
 小前瞻45，大前瞻60
 小前瞻保底程序
 31M/22秒~28秒

 小车跑动时路径检查测试

 各函数运行时间：
 CCD 8bit 采样         572us
 AccGyro  采样+计算    784us
 方向、速度控制、输出  208us

 速度测试
 前瞻40cm@68°
 38m--31.5s
 稳定性不错

 38m--26s

 非线性PID参数，直立效果超好，但是加速困难？？
 CCD数据中值滤波
 加入起跑线检测，6个跳变沿

 P=36 setSpeedMM=1800

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
    if (switchChange == true) {
      Dly_ms(2000);
      switchChange = false;
    }
    if (CCDDebugSwitch == 1)
      sendCCDData(CCDBuf);
    sendSign = 0;
    if (reachedEnd() == true) {
      if (StopCarAtFinish == true) {
        shutdown();
        while (StopCarAtFinish == true);  //wait for start signal
      }
    }
  }
}


void checkSwitch() {
  static uint8_t lastSwitch, switchChange;
  uint8_t temp,temp1;

  float setSpeedMM;

  //reading switches signal
  temp = PTH;
  StopCarAtFinish = temp & 0b00000001;
  //temp1=temp&0b00000111;
  //SpeedControlP=temp1*12;
  temp1 = temp >> 3 & 0b00000111;
  if (temp1 == 0)
    setSpeedMM = 0;
  else
    setSpeedMM = -1400 + temp1 * 200 * -1;  //steady state :1800

  //setSpeed(setSpeedMM);
  temp1 = temp >> 6 & 0b0000001;
  if (temp1 != temp2) {
    DefaultAngle -= 1;
    temp2 = temp1;
  }
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
  PITTF_PTF0 = 1;         //clear interrupts flag
  PORTA_PA7=1;

  checkSwitch();
  measureSpeed();
  updateCCD();
  updateIMU();
  
  int16_t speedL,speedR;
  int16_t *pL,*pR;
  pL=&speedL;
  pR=&speedR;
  
  PIDControl(pL,pR);
  setSpeed(*pL,*pR);
  updateMotor();
}
#pragma CODE_SEG DEFAULT         

