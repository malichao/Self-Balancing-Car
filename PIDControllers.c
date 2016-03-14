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

#include "filters.h"
#include "IMU.h"
#include "CCD.h"
#include "motor.h"


static float BalanceAngle = -67;
static float OriginalAngle = -71.5;
float BalanceP = 1300, BalanceD = 14; //BalanceP=1250,BalanceD=18;

static float DirectionP = 126;
static float DirectionD = -13.8;       
static float LineCenter=CCD_PIXELS/2;

static float TargetSpeed;
static float SpeedP = 32.0;
static float SpeedI = 0;
static float SpeedD = 0.0;

void PIDControl(int16_t *speedL,int16_t *speedR){
  static uint8_t periodCount=0;

  periodCount++;
  balancePID(speedL,speedR);
  if(periodCount%SPEED_CONTROL_PERIOD==0)
    SpeedPID(speedL,speedR);
  if(periodCount%DIRECTION_CONTROL_PERIOD==0)
    DirectionPID(speedL,speedR);
}

static void balancePID(int16_t *speedL,int16_t *speedR){
  static float error, dError;
  float output;
  error = BalanceAngle - getAngle();
  error = FIR(2, error);

  //The gyro output can be used as differential error
  dError = getAngularSpeed(ANGULAR_SPEED_PITCH);      
  output = BalanceP * error - BalanceD * dError;

  *speedL+=(int16_t)output;
  *speedR+=(int16_t)output;
}

static void speedPID(int16_t *speedL,int16_t *speedR){
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

static void directionPID(int16_t *speedL,int16_t *speedR){
  float error, dErroror;
  float output;
  error = LineCenter - getLineCenter();

  dErroror = getAngularSpeed(ANGULAR_SPEED_YAW);
  output = error * DirectionP - dErroror * DirectionD;

  if (output > DIRECTION_CONTROL_OUT_MAX)
    output = DIRECTION_CONTROL_OUT_MAX;
  if (output < DIRECTION_CONTROL_OUT_MIN)
    output = DIRECTION_CONTROL_OUT_MIN;

  //Notice the speed output is differential value
  *speedL+=(int16_t)output;
  *speedR-=(int16_t)output;
}
