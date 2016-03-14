/*
 Copyright (c) <2013-2016> <Malcolm Ma>

 Permission is hereby granted, free of int8_tge, to any person obtaining a copy 
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

/*    
 -------------------------------------------
 Code Warrior 5.0/1
 Target : MC9S12XS128
 Crystal: 16.000Mhz
 busclock:80.000MHz
 pllclock:160.000MHz  
 
 互补滤波
 双陀螺仪采样取平均
 定时器计时
 滑动权值
 当加速度计震动小时
 0.99<weight<0.998
 当震动太大时
 weight=1
 
 轮子转动一圈，脉冲数 R=1139 1119 1120.5  1121  L= 1118.5 1105 1119   avr=1120
 轮子直径50.5mm  周长=158.65mm
 MMperPulse=158.65/1120=0.14165
 PulseperMM=7.05965
 
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

 单路陀螺仪


 非线性PID参数，直立效果超好，但是加速困难？？
 CCD数据中值滤波
 加入起跑线检测，6个跳变沿

 P=36 SetSpeedMM=1800

 ********************************************************/

#include "derivative.h"
#include "stdio.h"
#include "math.h"
#include "SCI.h"
#include "ADC.h"
#include "times"
#include "PWM.h"
#include "macros.h"
#include "filters.h"
#include <IMU.h>

//CCD debugging settings
int8_t debug = 1;
float timer1, time1, timer2, time2, timer3, time3; //Some timer profiler to speed up the code

//============= Definition and parameters of gyros and Accelerometers=============
float gyroOffset = 121;
float gyroZ, accX, accY, accZ;
float angleG = 0;
//Four gyros: Z1 and Z2 for pitch angle,X1 and X2 for yaw angle 
float GyroOffsetZ1 = 1950;      //Gyro offset value,set larger to tilt forward
float GyroOffsetZ2 = 1950;
float GyroOffsetX1 = 1950;
float GyroOffsetX2 = 1950;
float GyroCoef = 0.536;         //Gyro coefficient,see datasheet for detail     
float AccOffse = 1365;  //Accelerometers offset value,set larger to tilt forward
float KP = 700.0;
float KD = 20.5;
float Tg = 2.0;
float kjifen = 150.0;

float AngleAZ = 0;
float AngleAX = 0;
float AngleAY = 0;
float AngleA = 0;
float AngleFilter = 0;
float gyro, Aangle;
float gyro, angle, jifen;
//Balancing control parameters
uint32_t gyroTimer = 0;
float weightFlag = 0;
float GyroSense = 0.01;
float GyroSense2 = 0.67;
float AccSense = 800;               //mV
float gravity = 0, gravityG = 1, gravityError;
float gravityGate = 0.06, gravityVibrationGate = 0.2;
float weight = 0.99;
float weight1 = 0.995, weight2 = 0.98;
float Setpoint = -67, OriginPoint = -71.5, Input = 0, Output = 0; //前倾角度增大，后仰减小
float kp = 1300, kd = 14; //kp=1250,kd=18;
float ka = 0.1;
float Error, dErr, LastErr = 0;
float val_kp, val_kd;

//============= Definition and parameters of Speed PID Control ==============
float SpeedControlP = 32.0;
float SpeedControlI = 0;
float SpeedControlD = 0.0;
int16_t MotorOffestL = 1900;          //Left Motor PWM output offset
int16_t MotorOffestR = 1800;          //Right Motor PWM output offset
int16_t CAR_SPEED_SET = 0;
float MMperPulse = 0.14165, PulseperMM = 7.05965;
float SetSpeed = 0, SetSpeedMM = -1350, fspeed, sp = 1, si = 0;
float CarSpeed;
float SpeedControlIntegral=0,
float SpeedControlOutOld=0,
float SpeedControlOutNew=0,
float SpeedControlOut=0;
int16_t SpeedControlCount = 0;
int16_t SpeedControlPeriod = 0;
int16_t scPeriod = 10;
int16_t ControlFlag = 0;
int16_t SpeedL = 0, SpeedR = 0;

//============= Definition and parameters of Linear CCD Sensor==============
int16_t CCDTime = 0;
int16_t ccdMultiple=1,
int16_t CCDBL=3;
uint8_t CCDRAW[2][128] = { 0 };     //Buffer to store the raw CCD sampling data
uint8_t CCDLine[128] = { 0 };
int16_t CCDDebugSwitch = 0;
int16_t CCDDebugSwitch2 = 2;
float FZBL = 0.5;                //0.35
float FZBL1 = 0.6;
int16_t CCDt;
long CCDa;
int16_t FZ, CCDAvr0, Rblack, LastRblack, Lblack, LastLblack, LineCenter = 64;
int16_t LastC1 = 64, LastC2 = 64, LastC3 = 64;
;
int16_t trackWidth = 73;
float sWeight = 0.6;
float dWeight = 0.4;
int8_t CCDFirstTime = true;
int16_t Lspeed, Rspeed, LspeedJF, RspeedJF;
int16_t speed, AngleControlOut;

//============= Definition and parameters of Steering PID Control ==============
float DirectionControlP = 126;
float DirectionControlD = -13.8;        //转向比例系数
int16_t DirectionControlPeriod = 1;
float DError, DLastError, DDError;
;
float DirectionControlOutNew;
float DirectionControlOutOld;
float DirectionControlOut;
int16_t DirectionControlCount;
int16_t LOUT, ROUT;

long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void SendCCDData(int8_t arr[]) {
  int16_t i;
  for (i = (64 - WINDOW_WIDTH / 2); i < (64 + WINDOW_WIDTH / 2); i++) {
#if defined PRINT_AD          //串口发送AD值，可用于线性CCD调试助手
    if(arr[i]==0xFF)
    arr[i] = 0xFE;            //遇到FF用FE替换即可
    putchar(arr[i]);
#else                         //串口发送而值量，方便用串口调试
    if (arr[i] > CCD_THRESHOLD)
      putchar(1);
    else
      putchar(0);
#endif     
  }
  putchar(0xFF);
}

void initPort() {
  DDRA = 0XFF;  //PortA for PWM output
  DDRB = 0X00;  //PortB for external counter input
  DDRH = 0x00;  //PortH for detecting swithes
}

void setInterruptPriorities() {
  INT_CFADDR = 0x70;
  INT_CFDATA5 = 0x05;
  INT_CFADDR = 0xD0;
  INT_CFDATA3 = 0x07;
}

void printOut() {
  putf (angleA);
  putf (angleFilter2);
  putf(64);
  putf(LineCenter);
  putf (LineCenter1);

  putf(fspeed);
  putf(CarSpeed * MMperPulse * 1000 / (5 * scPeriod));

  putstr(" \n\r");
}

void PID() {
  int16_t k = 1;
  Error = Setpoint - Input;
  Error = FIR(FIRPar[2], Error);
  ValueK = k;

  dErr = testGyroZ1 - GyroOffsetZ1;   //kp=1900,kd=20;//kp=1600,kd=4000;
  Output = kp * Error * k - kd * dErr * k;

  val_kp = kp * Error;
  val_kd = kd * dErr;

  if (fabs(Error) > 45)
    Output = 0;

  LastErr = Error;
}

void SpeedControl() {
  float fP, fDelta;
  float fI;
  float fD;
  float P;

  static lastErr=0;

  CarSpeed = (SpeedL + SpeedR) / 2;
  SpeedL = SpeedR = 0;

  CarSpeed *= 1.0;
  CarSpeed = FIR(FIRPar[1], CarSpeed);

  P = SpeedControlP;
  fDelta = SetSpeed - CarSpeed;
  fDelta = FIR(FIRPar[4], fDelta);

  if (fDelta >= 0)
    k = map(fDelta, 0, 1000, 0, 45);
  else
    k = map(fDelta, 0, -1000, 0, 45);
  speedK = tangent[k];

  fP = fDelta * P * speedK;
  fI = fDelta * SpeedControlI * speedK;
  fD = (fDelta - lastErr) * SpeedControlD * speedK;
  lastErr = fDelta;
  SpeedControlIntegral += fI;

  SpeedControlOutOld = SpeedControlOutNew;

  SpeedControlOutNew = fP + SpeedControlIntegral - fD;

  if (SpeedControlOutNew > SPEED_CONTROL_OUT_MAX)
    SpeedControlOutNew = SPEED_CONTROL_OUT_MAX;
  if (SpeedControlOutNew < SPEED_CONTROL_OUT_MIN)
    SpeedControlOutNew = SPEED_CONTROL_OUT_MIN;
}

void setSpeed(float myspeed) {
  SetSpeed = myspeed * PulseperMM * scPeriod * 5 / 1000;
  SetSpeed = 0;
}

void SpeedControlOutput() {
  float fValue;
  fValue = SpeedControlOutNew - SpeedControlOutOld;
  SpeedControlOut = fValue * (SpeedControlPeriod + 1) / scPeriod
      + SpeedControlOutOld;
}

void DirectionControl() {
  //float DError1;
  int16_t kk;
  float DErrorMult = 1;
  DirectionControlOutOld = DirectionControlOutNew;
  DError = LineCenter - (64 - 3);

  DDError = testGyroX1 - GyroOffsetX1;
  DirectionControlOutNew = DError * DirectionControlP * DErrorMult
      - DDError * DirectionControlD * DErrorMult;
  DLastError = DError;
}

void DirectionControlOutput() {
  float fValue;
  fValue = DirectionControlOutNew - DirectionControlOutOld;
  DirectionControlOut = fValue * (DirectionControlPeriod + 1)
      / DirectionControlPeriod + DirectionControlOutOld;
}

int8_t sendSign = 0, updateSign = 0;
uint32_t speedTimer = 0;
uint32_t tt1, ttr1;
uint8_t lastSwith, swithChange;
void main() {
  //int8_t c,cc; 
  int8_t temp;
  int8_t read;
  int16_t i;
  setbusclock_80M();
  Interrupt_Priority_Set();
  Port_Init();
  PWM_Init();
  BMQ_Init();
  UART0_Init();
  ADC_Init();
  setADC12bit();
  PIT_Init();
  EnableInterrupts;
  DisableControlPIT EnableTimerPIT;

  Dly_ms(2000);
  CCDCalibration();
  AccGyroCalibration();

  //gyroTimer=micros();//large delay will effect the gyro integration
  //while(1){speedout ();getspeed();Dly_ms(5);}  //死区、编码器测试
  speedTimer = millis();

  EnableControlPIT;
  for (;;) {
    //LOUT=-speed;
    //ROUT=-speed;
    //speedout (); 

    time3 = micros();
    /*DisableInterrupts;
     for(i=0;i<128;i++)
     CCDBuf[i]=CCDRAW[i];
     EnableInterrupts;
     SendCCDData(CCDBuf);
     //printout ();
     */
    if (CCDDebugSwitch == 0)
      printout();

    if (swithChange == true) {
      Dly_ms(2000);
      swithChange = false;
    }

    //if(sendSign){
    if (CCDDebugSwitch == 1)
      SendCCDData (CCDBuf);
    sendSign = 0;
    //} 
    if (scratchLine == true) {
      if (StopCarOn == true) {
        Dly_ms(400);
        shutdown();
        while (StopCarOn == true)
          ;
        Dly_ms(1000);
      }
      scratchLine = false;
    }
    timer3 = micros() - time3;
    timer3 = micros() - time3;
    //speedout (); 
  }
}

void checkSwith(){
//reading switches signal
  temp=PTH;
  StopCarOn=temp&0b00000001;
  //temp1=temp&0b00000111;
  //SpeedControlP=temp1*12;
  temp1=temp>>3&0b00000111;
  if(temp1==0)
  SetSpeedMM=0;
  else
  SetSpeedMM=-1400+temp1*200*-1;//steady state :1800
  fspeed=SetSpeedMM;
  //setSpeed(SetSpeedMM);
  temp1=temp>>6&0b0000001;
  if(temp1!=temp2) {
    OriginPoint-=1;
    temp2=temp1;
  }
  temp1=temp>>7&0b0000001;
  if(temp1!=temp3) {
    OriginPoint+=1;
    temp3=temp1;
  }

  temp=PTH;
  temp1=temp>>1&0b00000001; //accelerate

  //Determin what to do according to the switch state.
  switch(temp1) {

    case 0:
    tempPoint=OriginPoint;
    break;
    case 1:
    if(abs(LineCenter-64)<8) {
      tempSpeed= CarSpeed*MMperPulse*1000/(5*scPeriod);
      if(tempSpeed-900>SetSpeedMM)
      tempPoint=OriginPoint+5;
      else if(tempSpeed-800>SetSpeedMM)
      tempPoint=OriginPoint+4;
      else if(tempSpeed-700>SetSpeedMM)
      tempPoint=OriginPoint+3;
      else if(tempSpeed-600>SetSpeedMM)
      tempPoint=OriginPoint+2;
      else if(tempSpeed-500>SetSpeedMM)
      tempPoint=OriginPoint+1;
      else if(tempSpeed-400>SetSpeedMM)
      tempPoint=OriginPoint+0.5;
      else
      tempPoint=OriginPoint;
    }
    else {
      if(tempSpeed<SetSpeedMM)
      tempPoint=OriginPoint-1;
      else
      tempPoint=OriginPoint;
    }
    break;
  }

  //Decide whether we're going to deal with obstacles or not.
  temp1=temp>>2&0b00000001;
  if(temp1) {

    if(obstacleSign==true) {
      obstacleCounter++;
      if(obstacleCounter>=100) {
        obstacleCounter=0;
        obstacleSign=false;
      }
      if(tempSpeed<-2500)
      tempPoint=OriginPoint-4;
      else if(tempSpeed<-2200)
      tempPoint=OriginPoint-3;
      else if(tempSpeed<-2000)
      tempPoint=OriginPoint-2;
      else if(tempSpeed<-1800)
      tempPoint=OriginPoint-1;
    }
  }

  if(lastSwith!=temp) {
    swithChange=true;
    Setpoint=OriginPoint;
    setSpeed(-50);
    scratchLineCount=0;
    scratchLineTimer=millis();
    scratchLine=false;
  }
  if(swithChange==false) {
    if(abs(LineCenter-64)>12)
    SetSpeedMM=-1800;
    Setpoint=tempPoint;
    setSpeed(SetSpeedMM);
  }
  lastSwith=temp;
}

#pragma CODE_SEG __NEAR_SEG NON_BANKED
float ccdSum = 0;
float time, timer;
float t1, tr1, t2, tr2, t3, tr3, tr4, t4, tr5, t5;
uint8_t temp, temp1, temp2 = 0, temp3 = 0;
void interrupt 67 PIT1(){
  int16_t i,j;

  int16_t tempL,tempR;
  int16_t tempEdge;
  int16_t tempCCD;
  float tempPoint,tempSpeed;
  time=micros();
  PITTF_PTF0 = 1; //clear interrupts flag
  PORTA_PA7=1;

  checkSwith();


  CCDTime++;
  if(CCDTime>=ccdMultiple) {
    CCDTime=0;
    RD_CCD(0);
    CCDRAW[0][0]=CCDRAW[0][1]; //CCDRAW[0][0] is a bad point
    CalculateCCD0();

  }

  getAccGyrovalues ();
  calculateAngle();
  tr2=micros()-t2;

  t3=micros();
  getspeed();
  Input=angleFilter2;
  PID();
  SpeedControlPeriod ++;
  SpeedControlOutput();
  DirectionControlOutput();
  SpeedControlCount ++;
  if(SpeedControlCount >=scPeriod){
    SpeedControl();
    SpeedControlCount = 0;
    SpeedControlPeriod = 0;
  }

  DirectionControlCount++;
  if(DirectionControlCount >=DirectionControlPeriod){
    DirectionControl();
    DirectionControlCount = 0;
    DirectionControlPeriod = 0;
  }
  SpeedOutCalculate ();
  //LOUT=-(Output);
  //ROUT=-(Output);
  speedout ();
  tr3=micros()-t3;

  timer=micros()-time;
}

#pragma CODE_SEG DEFAULT         

