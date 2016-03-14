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

#include "math.h"
#include "times"
#include "macros.h"
#include "filters.h"
#include "IMU.h"

//============= Definition and parameters of Speed PID Control ==============
static float SpeedP = 32.0;
static float SpeedI = 0;
static float SpeedD = 0.0;
int16_t CAR_SPEED_SET = 0;
float MMperPulse = 0.14165, PulseperMM = 7.05965;
float TargetSpeed = 0, TargetSpeedMM = -1350, fspeed, sp = 1, si = 0;
float carSpeed;
float SpeedIntegral=0,
float SpeedControlOutOld=0,
float SpeedControlOutNew=0,
float SpeedControlOut=0;
int16_t SpeedControlCount = 0;
int16_t SpeedPeriod = 0;
int16_t scPeriod = 10;
int16_t ControlFlag = 0;
int16_t SpeedL = 0, SpeedR = 0;

//============= Definition and parameters of Steering PID Control ==============
float DirectionControlP = 126;
float DirectionControlD = -13.8;        //转向比例系数

//Balancing control parameters
static float BalanceAngle = -67;
static float OriginalAngle = -71.5;
static float TargetSpeed;
static floatInput = 0, Output = 0; //前倾角度增大，后仰减小
float kp = 1300, kd = 14; //kp=1250,kd=18;

float val_kp, val_kd;

void PIDControl(){
  static uint8_t periodCount=0;
  periodCount++;
  balancePID();
  if(periodCount%SPEED_CONTROL_PERIOD==0)
    SpeedPID();
  if(periodCount%DIRECTION_CONTROL_PERIOD==0)
    DirectionPID();

}

static void balancePID(int16_t *speedL,int16_t *speedR) {
  static float error, dErr;
  float output;
  error = BalanceAngle - getAngle();
  error = FIR(2, error);

  //The gyro output can be used as differential error
  dErr = getAngularSpeed();       
  output = kp * error- kd * dErr;

  *speedL+=(int16_t)output;
  *speedR+=(int16_t)output;
}

static void speedPID(int16_t *speedL,int16_t *speedR) {
  float outputP,outputI,outputD;
  float output;

  static float lastError=0;
  float error;
  int16_t *speedL=NULL,*speedR=NULL;

  int16_t carSpeed = getSpeedAll();
  carSpeed = FIR(1, carSpeed);
  error = TargetSpeed - carSpeed;

  outputP = error * SpeedP;
  outputI = error * SpeedI * SpeedI;
  outputD = (error - lastError) * SpeedD;
  lastError = error;
  SpeedIntegral += error;

  output=outputP+outputI+outputD;

  if (output > SPEED_CONTROL_OUT_MAX)
    output = SPEED_CONTROL_OUT_MAX;
  if (output < SPEED_CONTROL_OUT_MIN)
    output = SPEED_CONTROL_OUT_MIN;

  *speedL+=(int16_t)output;
  *speedR+=(int16_t)output;
}

static void directionPID(int16_t *speedL,int16_t *speedR) {
  float error, DLasterror, DDerror;
  Derror = LineCenter - (64 - 3);

  DDerror = testGyroX1 - GyroOffsetX1;
  DirectionControlOutNew = Derror * DirectionControlP
      - DDerror * DirectionControlD;
  DLasterror = Derror;
}

static void speedControlOutput() {
  float temp;
  temp = SpeedControlOutNew - SpeedControlOutOld;
  SpeedControlOut = temp * (SpeedPeriod + 1) / scPeriod
      + SpeedControlOutOld;
}



void DirectionControlOutput() {
  float fValue;
  fValue = DirectionControlOutNew - DirectionControlOutOld;
  DirectionControlOut = fValue * (DirectionControlPeriod + 1)
      / DirectionControlPeriod + DirectionControlOutOld;
}

void calculateSpeedOutput() { //SpeedOutCalculate
  int16_t i;
  float temp=0;
  speed=Output-SpeedControOutputLeft;
  
  //DirectionControOutputLeft=0;//for ccd test only
  OutputLeft=-(speed+DirectionControOutputLeft);
  OutputRight=-(speed-DirectionControOutputLeft);
}