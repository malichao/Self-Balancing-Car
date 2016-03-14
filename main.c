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
#include "Time_Function.h"
#include "PWM.h"
#include "macros.h"
#include "filters.h"

//CCD debugging settings
char debug=1;
float timer1,time1,timer2,time2,timer3,time3; //Some timer profiler to speed up the code

//============= Definition and parameters of gyros and Accelerometers=============
float gyroOffset=121;
float gyroZ,accX,accY,accZ;
float angleG=0;
//Four gyros: Z1 and Z2 for pitch angle,X1 and X2 for yaw angle 
float GyroOffsetZ1=1950;      //Gyro offset value,set larger to tilt forward
float GyroOffsetZ2=1950;
float GyroOffsetX1=1950;
float GyroOffsetX2=1950;
float GyroCoef=0.536;             //Gyro coefficient,see datasheet for detail     
float AccOffse= 1365;               //Accelerometers offset value,set larger to tilt forward
float KP=700.0;
float KD=20.5;  
float Tg =2.0;
float kjifen=150.0;
int    AccOffset=1365;
float AngleAZ=0;
float AngleAX=0;
float AngleAY=0;
float AngleA=0;
float AngleFilter=0;
float gyro,Aangle;
float gyro,angle,jifen; 
//Balancing control parameters
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


//============= Definition and parameters of Speed PID Control ==============
float SpeedControlP=32.0;
float SpeedControlI=0;
float SpeedControlD=0.0;
int MotorOffestL=1900;          //Left Motor PWM output offset
int MotorOffestR=1800;          //Right Motor PWM output offset
int CAR_SPEED_SET =0;
float MMperPulse=0.14165,PulseperMM=7.05965;
float SetSpeed=0,SetSpeedMM=-1350,fspeed,sp=1,si=0;
float CarSpeed;
float SpeedControlIntegral=0,
float SpeedControlOutOld=0,
float SpeedControlOutNew=0,
float SpeedControlOut=0;
int SpeedControlCount = 0;
int SpeedControlPeriod = 0;
int scPeriod=10;  
int ControlFlag=0;
int SpeedL=0,SpeedR=0;

//============= Definition and parameters of Linear CCD Sensor==============
int CCDTime=0;
int ccdMultiple=1,
int CCDBL=3;
unsigned char CCDRAW[2][128]={0};     //Buffer to store the raw CCD sampling data
unsigned char CCDLine[128]={0};
int CCDDebugSwitch=0;
int CCDDebugSwitch2=2;
float FZBL= 0.5;                //0.35
float FZBL1=0.6;
int CCDt;
long CCDa;
int FZ,CCDAvr0,Rblack,LastRblack,Lblack,LastLblack,LineCenter=64;
int LastC1=64,LastC2=64,LastC3=64;;
int trackWidth=73; 
float sWeight=0.6;
float dWeight=0.4;
char CCDFirstTime=true;
int Lspeed,Rspeed,LspeedJF,RspeedJF;
int speed,AngleControlOut;  


//============= Definition and parameters of Steering PID Control ==============
float DirectionControlP=126;
float DirectionControlD=-13.8;        //转向比例系数
int DirectionControlPeriod=1;
float DError,DLastError,DDError;;
float DirectionControlOutNew;
float DirectionControlOutOld;
float DirectionControlOut;
int DirectionControlCount;
int LOUT,ROUT;



long map(long x, long in_min, long in_max, long out_min, long out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void CalculateCCD0 (void) {
    int tempEdge;
    if(CCDDebugSwitch2==1) {           //Enable median value filter
     for(int i=1;i<127;i++){
      CCDBuf[i]=mid(&CCDRAW[0][i-1]);
      CCDBuf2[i]=CCDBuf[i]; 
      } 
    }
    else{    
     for(i=0;i<128;i++){
      CCDBuf[i]=CCDRAW[0][i];  
      CCDBuf2[i]=CCDRAW[0][i]; 
     }
    }

    for(CCDt=0,CCDAvr0=0;CCDt<128;CCDt++) {
      CCDAvr0=CCDa+CCDBuf2[CCDt];
    }  
    CCDAvr0=CCDAvr0/128;
    FZ=CCDa*FZBL;
     
     if(angleFilter2>OriginPoint-10&&angleFilter2<OriginPoint+15) //-74~-49
        if(CCDAvr0<Threshold*2/3)
           obstacleSign=true;
       
       
       for(CCDt=0,tempEdge=128,CCDEdge=0;CCDt+DCCD<128;CCDt++) {
         if(abs(CCDBuf2[CCDt]-CCDBuf2[CCDt+DCCD])>FZ) 
          {
          if(abs(tempEdge-CCDt)>DCCD*2) {
            tempEdge=CCDt;
            edge[CCDEdge]=CCDt;
            CCDEdge++;
          }
          }
       }
      if(CCDEdge>5&&(millis()-scratchLineTimer)>10000) {
        scratchLineCount++;
        scratchLine=true;
      }
      
      /*
      if(CCDBuf2[LineCenter]>CCDa*0.71)
        if(CCDBuf2[LineCenter+trackWidth/4]<CCDa*0.71)
           if(CCDBuf2[LineCenter-trackWidth/4]<CCDa*0.71)
              scratchLine=true;
      */ 
        for(CCDt=LineCenter;CCDt+DCCD<128;CCDt++)  //Rblack
        {
          if(CCDBuf2[CCDt]-CCDBuf2[CCDt+DCCD]>FZ) 
          {
            Rblack=CCDt+DCCD;
             for(CCDt=Rblack;CCDt<128;CCDt++) 
            {
              CCDBuf2[CCDt]=2;
            }  
          }
        }
    for(CCDt=LineCenter;CCDt-DCCD>=0;CCDt--)  //Lblack
        {
          if(CCDBuf2[CCDt]-CCDBuf2[CCDt-DCCD]>FZ) 
          {
            Lblack=CCDt-DCCD;
            for(CCDt=Lblack;CCDt>=0;CCDt--) 
            {
              CCDBuf2[CCDt]=2;
            }  
          }
        }
        
        if(Lblack>5&&Rblack<123)
         trackWidth=Rblack-Lblack;
        
        if(Lblack>50)
         LineCenter=Lblack+trackWidth/2;
        else if(Rblack<78)
         LineCenter=Rblack-trackWidth/2;
        else
         LineCenter=(Lblack+Rblack)/2; 
        
        
        if(LineCenter+DCCD>125)
          LineCenter=125;
        else if(LineCenter<5)
          LineCenter=5;
        LastC1=LineCenter;

        LastC3=LastC2;
        LastC2=LastC1;

        
        LineCenter=FIR(FIRPar[3],LineCenter);

}


 void SendCCDData(byte arr[]) 
{
  int i;
 for(i=(64-WINDOW_WIDTH/2); i<(64+WINDOW_WIDTH/2); i++)
    {
#if defined PRINT_AD          //串口发送AD值，可用于线性CCD调试助手
      if(arr[i]==0xFF) 
        arr[i] = 0xFE;        //遇到FF用FE替换即可
      uart0_putchar(arr[i]);
#else                         //串口发送而值量，方便用串口调试
      if(arr[i]>CCD_THRESHOLD)
        uart0_putchar(1);
      else
        uart0_putchar(0);
#endif     
    }
    
    uart0_putchar(0xFF); 
}
  

void initPort(void){ 
  DDRA=0XFF;  //PortA for PWM output
  DDRB=0X00;  //PortB for external counter input
  DDRH=0x00;  //PortH for detecting swithes
}


//Initialize the encoder,IOC7 and PT7for generating clk
void initBMQ(void){   
  TCNT = 0x00;
  PACTL= 0xC0;  //Enable PAC
  TIE  = 0x00;  //Disable ISR
  PACNT = 0;
  BMQR_RESET=LOW;
} 

//Since there's only one external counter in the chip so we
//use another counter to measure the encoder on the left motor
void getspeed (void) {
  DisableInterrupts;
  int tempL,tempR;
  tempL=PORTB;       //Read the external encoder counter
  tempR=PACNT;       //Read the internal counter
  PACNT=0;
  BMQR_RESET=HIGH;
  Dly_us(1);
  BMQR_RESET=HIGH;

  SpeedL=SpeedL+tempL;
  SpeedR=SpeedR+tempR;
  EnableInterrupts;
}

//Loop unrolling to speed up
float testGyroZ1=0,testGyroX1=0;
void getADCValues (void) {
    setADC12bit(); 
    time2=micros();

    float temp=0;
    temp=readADC(GYRO_PIN_Z1)+readADC(GYRO_PIN_Z1)+readADC(GYRO_PIN_Z1)
            +readADC(GYRO_PIN_Z1)+readADC(GYRO_PIN_Z1)+readADC(GYRO_PIN_Z1)
            +readADC(GYRO_PIN_Z1)+readADC(GYRO_PIN_Z1)+readADC(GYRO_PIN_Z1)
            +readADC(GYRO_PIN_Z1);
    temp=temp/10.0;
    testGyroZ1=temp;
    gyro=(temp-GyroOffsetZ1)*GyroCoef; 
     
    temp=readADC(GYRO_PIN_X1)+readADC(GYRO_PIN_X1)+readADC(GYRO_PIN_X1)
      +readADC(GYRO_PIN_X1)+readADC(GYRO_PIN_X1)+readADC(GYRO_PIN_X1)
      +readADC(GYRO_PIN_X1)+readADC(GYRO_PIN_X1)+readADC(GYRO_PIN_X1)
      +readADC(GYRO_PIN_X1);  
    temp=temp/10.0;
    testGyroX1=temp;
    
    temp=readADC(ACC_PIN_Z)+readADC(ACC_PIN_Z)+readADC(ACC_PIN_Z)
          +readADC(ACC_PIN_Z)+readADC(ACC_PIN_Z)+readADC(ACC_PIN_Z)
          +readADC(ACC_PIN_Z)+readADC(ACC_PIN_Z)+readADC(ACC_PIN_Z)
          +readADC(ACC_PIN_Z);           
 
    AngleAZ=temp/10.0-AccOffset;
    
    temp=readADC(ACC_PIN_X)+readADC(ACC_PIN_X)+readADC(ACC_PIN_X)
          +readADC(ACC_PIN_X)+readADC(ACC_PIN_X)+readADC(ACC_PIN_X)
          +readADC(ACC_PIN_X)+readADC(ACC_PIN_X)+readADC(ACC_PIN_X)
          +readADC(ACC_PIN_X);  
    AngleAX=temp/10.0-AccOffset;  
    
    temp=readADC(ACC_PIN_Y)+readADC(ACC_PIN_Y)+readADC(ACC_PIN_Y)
          +readADC(ACC_PIN_Y)+readADC(ACC_PIN_Y)+readADC(ACC_PIN_Y)
          +readADC(ACC_PIN_Y)+readADC(ACC_PIN_Y)+readADC(ACC_PIN_Y)
          +readADC(ACC_PIN_Y);  
    AngleAY=temp/10.0-AccOffset;
    
    timer2=micros()-time2;
}

//Move these local variables up front so that we can see the values
//in the debugger.
float tempX,tempY,tempZ;
void calculateAngle(void) 
{
    unsigned long time=0;   
    
    time1=micros();
    
    tempX=AngleAX*5/4096*1000/AccSense;
    tempY=AngleAY*5/4096*1000/AccSense;
    tempZ=AngleAZ*5/4096*1000/AccSense;
    
    
    angleA=-atan2(AngleAX,AngleAZ)*180/PI;
    gravity=sqrt(tempX*tempX+tempY*tempY+tempZ*tempZ);
     //angleA2=AngleAZ/AccSense;
    angleA=FIR(FIRPar[0],angleA); 
    if(debug) {
    gravityError=fabs(gravity-gravityG);

    if(gravityError>gravityVibrationGate)// avoid vibration
    {
     weightFlag=10; 
     weight=1;
    } else{
      weight=weight2+(weight1-weight2)*gravityError/gravityVibrationGate;
      weightFlag=(weight-weight2)*100;
      }
    }
    
    
    time=micros()-gyroTimer;
    gyroTimeTest=time;
    gyroTimer=micros();
    if(time>50000)  //50ms,timeout
     angleG2=0;     //large delay will effect the gyro integration
    else
     angleG2=(testGyroZ1-GyroOffsetZ1)*5/4096*1.5/5.1/GyroSense2*time/1000;
    
    angleFilter2=angleA*(1-weight)+(angleG2+angleFilter2)*weight;

    timer1=micros()-time1;
}



void shutdown() {

    MotorEnable=false;
    PWMDTY01=0;
    PWMDTY23=0;
    PWMDTY67=0; 
    PWMDTY45=0;
}

void turnon(){
   MotorEnable=true;
}
void speedout (void) {
  if(!MotorEnable)
  return;

 if(fabs(Setpoint-angleFilter2)>45) {
  LOUT=0;
  ROUT=0;
 }
 
 if(LOUT==0)
  {
    PWMDTY01=0;
    PWMDTY23=0;  
  } 
  else if(LOUT>0) 
    { 
    PWMDTY01=0;
    PWMDTY23=LOUT+MotorOffestL;  //L
    } 
  else if(LOUT<0) 
    { 
    PWMDTY23=0;
    PWMDTY01=MotorOffestL-LOUT;
    }
  if(ROUT==0) 
  {
    PWMDTY67=0; 
    PWMDTY45=0;
  } 
  else if(ROUT>0) 
  {
    PWMDTY45=ROUT+MotorOffestR;  
    PWMDTY67=0;
  }
  
  else if(ROUT<0) 
  {
    PWMDTY45=0; 
    PWMDTY67=MotorOffestR-ROUT;
  }
}


void Interrupt_Priority_Set(void){
    INT_CFADDR=0x70;
    INT_CFDATA5=0x05;
    INT_CFADDR=0xD0;
    INT_CFDATA3=0x07;
}

float FIROutput;

void printout (void) { 
  putf(angleA);
  putf(angleFilter2);
  putf(64);
  putf(LineCenter);
  putf(LineCenter1);
  
  putf(fspeed);
  putf(CarSpeed*MMperPulse*1000/(5*scPeriod));
  
  putstr(" \n\r");  
}


 void PID() {
  int k=1;
  Error=Setpoint-Input;
  Error=FIR(FIRPar[2],Error);
  ValueK=k;
  // dErr=angleG2;      //kp=1600,kd=4000;
  dErr=testGyroZ1-GyroOffsetZ1;   //kp=1900,kd=20;

  Output=kp*Error*k-kd*dErr*k;
  //Output=kp*Error*30-kd*dErr*30;
  val_kp=kp*Error;
  val_kd=kd*dErr;

  if(fabs(Error)>45)
    Output=0;

  LastErr=Error;
}

void SpeedControl (void) {
  float fP, fDelta;
  float fI;
  float fD;
  float P;
  
  static lastErr=0;

  CarSpeed = (SpeedL+SpeedR) / 2;
  SpeedL=SpeedR=0;
  
  CarSpeed*=1.0;
  CarSpeed=FIR(FIRPar[1],CarSpeed);

  P=SpeedControlP;
  fDelta = SetSpeed- CarSpeed;
  fDelta=FIR(FIRPar[4],fDelta);
  
  if(fDelta>=0)
    k=map(fDelta,0,1000,0,45);
  else
    k=map(fDelta,0,-1000,0,45);
  speedK=tangent[k];  
   
  fP = fDelta * P*speedK;
  fI = fDelta * SpeedControlI*speedK;
  fD = (fDelta-lastErr) * SpeedControlD*speedK;
  lastErr=fDelta;
  SpeedControlIntegral += fI;
  
  SpeedControlOutOld = SpeedControlOutNew;

  SpeedControlOutNew = fP + SpeedControlIntegral-fD;
  
  if(SpeedControlOutNew > SPEED_CONTROL_OUT_MAX) 
    SpeedControlOutNew = SPEED_CONTROL_OUT_MAX;
  if(SpeedControlOutNew < SPEED_CONTROL_OUT_MIN)   
    SpeedControlOutNew = SPEED_CONTROL_OUT_MIN;
  
  
  }
  
void setSpeed(float myspeed) {
 SetSpeed=myspeed*PulseperMM*scPeriod*5/1000; 
 SetSpeed=0;
}


void SpeedControlOutput(void) {
  float fValue;
  fValue = SpeedControlOutNew - SpeedControlOutOld;
  SpeedControlOut=fValue * (SpeedControlPeriod + 1) /scPeriod + SpeedControlOutOld;
}


void DirectionControl(void) {
  //float DError1;
  int kk;
  float DErrorMult=1;
  DirectionControlOutOld = DirectionControlOutNew;
  DError=LineCenter-(64-3);
  
  DDError=testGyroX1-GyroOffsetX1;
  DirectionControlOutNew=DError*DirectionControlP*DErrorMult-DDError*DirectionControlD*DErrorMult;
  DLastError=DError; 
}


void DirectionControlOutput(void) {
  float fValue;
  fValue = DirectionControlOutNew - DirectionControlOutOld;
  DirectionControlOut=fValue * (DirectionControlPeriod + 1) /DirectionControlPeriod + DirectionControlOutOld;
}

void SpeedOutCalculate (void) {
  int i;
  float temp=0;
  speed=Output-SpeedControlOut;
  
  //DirectionControlOut=0;//for ccd test only
  LOUT=-(speed+DirectionControlOut);
  ROUT=-(speed-DirectionControlOut);
}


int samplecounter=0,sampletime=10, sampleSign=1;
float  tempGyroOffsetZ1=0,tempGyroOffsetZ2=0,tempGyroOffsetX1=0,tempGyroOffsetX2=0;
float tempAccX=0,tempAccY=0,tempAccZ=0;

void AccGyroCalibration() {
  for(;samplecounter<sampletime;samplecounter++) {
    getADCValues();
    tempGyroOffsetZ1+=testGyroZ1;
    tempGyroOffsetX1+=testGyroX1;
    tempAccX+=AngleAX;
    tempAccY+=AngleAY;
    tempAccZ+=AngleAZ;
    //Dly_ms(1);
  }
  gyroOffset=tempGyroOffsetZ1/sampletime;
  
  GyroOffsetZ1=tempGyroOffsetZ1/sampletime;
  GyroOffsetZ2=tempGyroOffsetZ2/sampletime;
  GyroOffsetX1=tempGyroOffsetX1/sampletime;
  GyroOffsetX2=tempGyroOffsetX2/sampletime;
  
  tempAccX/=sampletime;
  tempAccY/=sampletime;
  tempAccZ/=sampletime;
  tempAccX=tempAccX*5/4096*1000/AccSense;
  tempAccY=tempAccY*5/4096*1000/AccSense;
  tempAccZ=tempAccZ*5/4096*1000/AccSense;
  gravityG=sqrt(tempAccX*tempAccX+tempAccY*tempAccY+tempAccZ*tempAccZ);
  

  putstr("\ngravityG=");
  putf(gravityG);
  uart0_putchar('\n');
  getADCValues ();
  angleFilter2=-atan2(AngleAX,AngleAZ)*180/PI;
  putstr("\n Acc Gyro Init Done\n");
  
}


void CCDCalibration() {
  int i;
  //There are actually two cameras on the car for experiment,
  //in this code,only one is used.
  RD_CCD(0);
  for(i=0;i<10;i++) {
    //Delay some time between two sampling
      Dly_ms(ccdMultiple*5-1);
      RD_CCD(0);
      CalculateCCD0();
      Threshold+=CCDAvr0;
  }
  //Calculate the average value
  Threshold/=10;
  obstacleSign=false;
}



char sendSign=0,updateSign=0;
unsigned long speedTimer=0;
unsigned long tt1,ttr1;
unsigned char lastSwith,swithChange;
void main(){ 
  //char c,cc; 
  char temp;
  char read;
  int i;
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
  DisableControlPIT
  EnableTimerPIT;
  
  Dly_ms(2000);
  CCDCalibration();
  AccGyroCalibration();
  
  //gyroTimer=micros();//large delay will effect the gyro integration
  //while(1){speedout ();getspeed();Dly_ms(5);}  //死区、编码器测试
  speedTimer=millis();
 
  
  EnableControlPIT;
  for(;;)
  {

    //LOUT=-speed;
    //ROUT=-speed;
    //speedout (); 
    
    time3=micros();
    /*DisableInterrupts;
    for(i=0;i<128;i++)
     CCDBuf[i]=CCDRAW[i];
    EnableInterrupts;
    SendCCDData(CCDBuf);
    //printout ();
    */
    if(CCDDebugSwitch==0)
      printout ();
     
    if(swithChange==true){
      Dly_ms(2000);
      swithChange=false;
    }
    
    
    //if(sendSign){
      if(CCDDebugSwitch==1)
      SendCCDData(CCDBuf);
      sendSign=0;
    //} 
    if(scratchLine==true) {
      if(StopCarOn==true) {
        Dly_ms(400);
        shutdown();
        while(StopCarOn==true);
        Dly_ms(1000);
      }
      scratchLine=false;
    }
    timer3=micros()-time3;
    timer3=micros()-time3;
     //speedout (); 
  }
}


#pragma CODE_SEG __NEAR_SEG NON_BANKED
float ccdSum=0;
float time,timer;
float t1,tr1,t2,tr2,t3,tr3,tr4,t4,tr5,t5;
unsigned char temp,temp1,temp2=0,temp3=0;
void interrupt 67 PIT1(void)
{

   int i,j;
   
   int tempL,tempR;
   int tempEdge;
   int tempCCD;
   float tempPoint,tempSpeed;
   time=micros();
   PITTF_PTF0 = 1; //clear interrupts flag
   PORTA_PA7=1;
   
   //reading switches signal
   temp=PTH;
    StopCarOn=temp&0b00000001;
    //temp1=temp&0b00000111;
    //SpeedControlP=temp1*12;
    temp1=temp>>3&0b00000111;
    if(temp1==0)
     SetSpeedMM=0;
    else
      SetSpeedMM=-1400+temp1*200*-1;  //steady state :1800
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
  switch(temp1){
    
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
  if(temp1){
      
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
  if(swithChange==false){
    if(abs(LineCenter-64)>12)
     SetSpeedMM=-1800; 
    Setpoint=tempPoint;
    setSpeed(SetSpeedMM);
  }
  lastSwith=temp;
  
  
  t1=micros();
  
   CCDTime++;
   if(CCDTime>=ccdMultiple) {
     CCDTime=0; 
     RD_CCD(0);   
     CCDRAW[0][0]=CCDRAW[0][1];//CCDRAW[0][0] is a bad point
     CalculateCCD0();
   
   }


   
   getADCValues ();
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
   if(SpeedControlCount >=scPeriod) 
   { 
        SpeedControl();
        SpeedControlCount = 0;
        SpeedControlPeriod = 0;        
    }
    
    DirectionControlCount++;   
      if(DirectionControlCount >=DirectionControlPeriod) 
      {
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

