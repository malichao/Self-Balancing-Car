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


//========== Note: Why putting all the global values here in the header? ========
//It's not a good practice to put all the local/global variables in the header file
//but this is a special case.The freescale debugger(BDM) I use didn't support reading
//the variables defined in a function for some reason.But there are a lots of parameters
//need to be tuned at runtime to get the optimal performance.So I put all the variables
//here in the header files.


#ifndef INCLUDES_H
#define INCLUDES_H

#define PI 3.14159265358979323846 
#define true 1
#define false 0

//Definition of gyros
#define gyroPin_Z1 5
#define gyroPin_Z2 7
#define gyroPin_X1 6
#define gyroPin_X2 15
#define accPin_Z 4
#define accPin_Y 3
#define accPin_X 2
float gyroOffset=121;
float gyroZ,accX,accY,accZ;
float angleG=0;

//Definition of IO pins
#define BMQR  PORTA_PA4  		//Encoder Reset
#define TSL_SI  PORTA_PA1    	//CCD sensor pin SI
#define TSL_CLK PORTA_PA0    	//CCD sensor pin CLK 
#define TSL_SI1  PORTA_PA3    	//CCD sensor pin SI
#define TSL_CLK1 PORTA_PA2    	//CCD sensor pin CLK 

//CCD debugging settings
#define PRINT_AD   					//toggle if print the ADC values or binarized values
#define CCD_THRESHOLD 100    	//binarization threshold
#define WINDOW_WIDTH 128     //Set how many pixels to print 0~128

//Balancing control parameters
float GyroOffsetZ1=1950;
float GyroOffsetZ2=1950;
float TLYLDX1=1950;
float TLYLDX2=1950;
//#define TLYLD 1950           //陀螺仪零点，前倾则增加
float TLYBL=0.536;          //陀螺仪比例
float TLYBLZ2=0.536;          //陀螺仪比例
float TLYBLX1=0.536;         //陀螺仪比例
float TLYBLX2=0.536;          //陀螺仪比例

#define JSDLD 1365           //加速度计零点,往前跑则增加
float KP=700.0;
float KD=20.5;  
float Tg =2.0;
float kjifen=150.0;



//速度控制
float SPEED_CONTROL_P=32.0;
float SPEED_CONTROL_I=0;
float SPEED_CONTROL_D=0.0;
#define M_SQ 0   //电机死区
int M_SQL=1900;   //电机死区
int M_SQR=1800;   //电机死区
#define CAR_SPEED_SET_MAX -500
int CAR_SPEED_SET =0;
#define SPEED_CONTROL_OUT_MAX 8000
#define SPEED_CONTROL_OUT_MIN -8000

#define gyroPin_Z 5
#define accPin_Z 4
#define accPin_Y 3
#define accPin_X 2

int accOffset=1365;
float angleAZ=0,angleAX=0,angleAY=0,angleA=0,angleFilter=0;

//定义全局变量
int CCDTime=0,ccdMultiple=1,CCDBL=3;
byte ADV[2][128]={0,0};         //声明数组，用于存放采集的线性数值 
byte CCDLine[128]={0};
int CCDDebugSwitch=0,CCDDebugSwitch2=2;
float FZBL= 0.5;//0.35
float FZBL1=0.6;
int CCDt;
long CCDa;
#define DCCD 8
int FZ,CCDAvr,Rblack,LastRblack,Lblack,LastLblack,LineCenter=64;
int FZ1,CCDAvr1,Rblack1,LastRblack1,Lblack1,LastLblack1,LineCenter1=64;
int LastC1=64,LastC2=64,LastC3=64;;
int trackWidth=73; 
float sWeight=0.6;
float dWeight=0.4;
char CCDFirstTime=true;
float gyro,Aangle;
int Lspeed,Rspeed,LspeedJF,RspeedJF;
float gyro,angle,jifen; 
int speed,AngleControlOut;  

float KDIR=126,DDIR=-13.8;      //转向比例系数
int dcPeriod=1;
float DError,DLastError,DDError;;
float g_fDirectionControlOutNew,g_fDirectionControlOutOld,g_fDirectionControlOut,ZX;
int g_nDirectionControlPeriod;
int g_nDirectionControlCount;
int LOUT,ROUT;
float g_fCarSpeed,g_fSpeedControlIntegral=0,g_fSpeedControlOutOld=0,g_fSpeedControlOutNew=0,g_fSpeedControlOut=0;
int g_nSpeedControlCount = 0;
int g_nSpeedControlPeriod = 0;
int scPeriod=10;  
int ControlFlag=0;
int Lspeed,Rspeed,LspeedJF,RspeedJF;
float gyro,Aangle;
#define CMD_TURN_LEFT '!'
#define CMD_TURN_RIGHT '@'
#define CMD_FORWARD '#'
#define CMD_BACKWARD '$'
#define CMD_RUN '%'
#define CMD_STOP '^'    
#define CMD_SET_SPEED '&'
#define CMD_SET_GYROOFFSET '&'

char debug=1;
unsigned long gyroTimer=0;
float weightFlag=0;
float GyroSense=0.01;
float GyroSense2=0.67;
float AccSense=800;//mV
float gravity=0,gravityG=1,gravityError;
float gravityGate=0.06,gravityVibrationGate=0.2;
float weight=0.99; 
float weight1=0.995,weight2=0.98;
float Setpoint=-67,OriginPoint=-71.5,Input=0,Output=0; //前倾角度增大，后仰减小
float kp=1300,kd=14;//kp=1250,kd=18;
float ka=0.1;
float Error,dErr,LastErr=0;
float val_kp,val_kd;

float MMperPulse=0.14165,PulseperMM=7.05965;
float SetSpeed=0,SetSpeedMM=-1350,fspeed,sp=1,si=0;
float SError=0,SErrI=0;
float SOutput=0;
#endif