/*
Copyright (c) <2013> <Malcolm Ma>

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

********************************************************/

#include "derivative.h"
#include "stdio.h"
#include "math.h"
#include "SCI.h"
#include "ADC.h"
#include "Time_Function.h"
#include "PWM.h"
#include "includes.h"






//LOW_PASS,Fs=200,RECTANGLE,wp=2,ws=3

//LOW_PASS,Fs=200,HAMMING,wp=3,ws=9
#define FILTERTAPS 5
float coef[FILTERTAPS] = {0.001193,0.010720,0.026164,0.026164,0.010720};
float gain=13.340246;    
float values[FILTERTAPS] = {0}; 

float FIR(float in){
   static byte k;
   byte i = 0; 
   float out = 0;
   values[k] = in; 
   for (i=0; i<FILTERTAPS; i++) {            
     out += coef[i] * values[(i + k) % FILTERTAPS];                          
    }
    out *= gain;                        
    k = (k+1) % FILTERTAPS;                  
    return out;                        
}

//LOW_PASS,Fs=40,RECTANGLE,wp=10,ws=11
#define FILTERTAPS2 5
float coef2[FILTERTAPS2] = {0.000000,0.318310,0.500000,0.500000,0.318310};
float gain2=0.611015;


float values2[FILTERTAPS2] = {0}; 

float FIR2(float in){
   static byte k;
   byte i = 0; 
   float out = 0;
   values2[k] = in; 
   for (i=0; i<FILTERTAPS2; i++) {            
     out += coef2[i] * values2[(i + k) % FILTERTAPS2];                          
    }
    out *= gain2;                        
    k = (k+1) % FILTERTAPS2;                  
    return out;                        
}



//LOW_PASS,Fs=20,HAMMING,wp=3,ws=9
#define FILTERTAPS3 5
float coef3[FILTERTAPS3] = {0.006055,0.092153,0.261644,0.261644,0.092153};
float gain3=1.401247;
float values3[FILTERTAPS3] = {0}; 

float FIR3(float in){
   static byte k;
   byte i = 0; 
   float out = 0;
   values3[k] = in; 
   for (i=0; i<FILTERTAPS3; i++) {            
     out += coef3[i] * values3[(i + k) % FILTERTAPS3];                          
    }
    out *= gain3;                        
    k = (k+1) % FILTERTAPS3;                  
    return out;                        
}


//LOW_PASS,Fs=20,HAMMING,wp=3,ws=9
#define FILTERTAPS4 5
float coef4[FILTERTAPS4] = {0.006055,0.092153,0.261644,0.261644,0.092153};
float gain4=1.401247;
float values4[FILTERTAPS4] = {0}; 

float FIR4(float in){
   static byte k;
   byte i = 0; 
   float out = 0;
   values4[k] = in; 
   for (i=0; i<FILTERTAPS4; i++) {            
     out += coef4[i] * values4[(i + k) % FILTERTAPS4];                          
    }
    out *= gain4;                        
    k = (k+1) % FILTERTAPS4;                  
    return out;                        
}


 void SendCCDData(byte arr[]) 
{
  int i;
 for(i=(64-WINDOW_WIDTH/2); i<(64+WINDOW_WIDTH/2); i++)
    {
#if(PRINT_AD==1)          //串口发送AD值，可用于线性CCD调试助手
      if(arr[i]==0xFF) 
        arr[i] = 0xFE; //遇到FF用FE替换即可
      uart0_putchar(arr[i]);
#else                     //串口发送而值量，方便用串口调试
      if(arr[i]>THRESHOLD)
        uart0_putchar(1);
      else
        uart0_putchar(0);
#endif     
    }
    
    uart0_putchar(0xFF); 
}
  

void Port_Init(void)  
{ 
  //DDRA=0XFF;  
  //DDRB = 0XFF;      //原版
  //PORTB = 0X00;
  
  
  DDRA=0XFF;  
  DDRB = 0X00;      //测速实验
  DDRK=0xFF;
}



//IOC7/PT7用于计算右编码器产生的脉冲数
void BMQ_Init(void)
{   
  TCNT = 0x00;
  PACTL= 0xC0;//允许PAC
  TIE  = 0x00;//每一位对应相应通道禁止中断
  PACNT = 0;
  BMQR=0;
} 


void getspeed (void) 
{
DisableInterrupts;
Lspeed=PORTB;
Rspeed=PACNT;
PACNT=0;
BMQR=1;
Dly_us(1);
BMQR=0;
//if(speed<0) 
  {
    Lspeed=-Lspeed;
    Rspeed=-Rspeed;
  }

  LspeedJF=LspeedJF+Lspeed;
  RspeedJF=RspeedJF+Rspeed;
  EnableInterrupts;
}

float timer1,time1,timer2,time2,timer3,time3;
float testGyroZ1=0,testGyroZ2=0,testGyroX1=0,testGyroX2=0;;
void getAD (void) 
{
    unsigned int tlycs,jiao;
    float temp=0;
    time2=micros();
    //DisableInterrupts;
    tlycs=ADChannelx(5)+ADChannelx(5)+ADChannelx(5)+ADChannelx(5)+ADChannelx(5) 
	       +ADChannelx(5)+ADChannelx(5)+ADChannelx(5)+ADChannelx(5)+ADChannelx(5);
	  temp=tlycs/10.0;
	  testGyroZ1=temp;
	  gyro=(temp-TLYLDZ1)*TLYBL; 
	   
	   tlycs=ADChannelx(7)+ADChannelx(7)+ADChannelx(7)+ADChannelx(7)+ADChannelx(7) 
	       +ADChannelx(7)+ADChannelx(7)+ADChannelx(7)+ADChannelx(7)+ADChannelx(7);
	  temp=tlycs/10.0;
	  testGyroZ2=temp;
	  
	  tlycs=ADChannelx(6)+ADChannelx(6)+ADChannelx(6)+ADChannelx(6)+ADChannelx(6) 
	       +ADChannelx(6)+ADChannelx(6)+ADChannelx(6)+ADChannelx(6)+ADChannelx(6);
	  temp=tlycs/10.0;
	  testGyroX1=temp;
	  
	  tlycs=ADChannelx(15)+ADChannelx(15)+ADChannelx(15)+ADChannelx(15)+ADChannelx(15) 
	       +ADChannelx(15)+ADChannelx(15)+ADChannelx(15)+ADChannelx(15)+ADChannelx(15);
	  temp=tlycs/10.0;
	  testGyroX2=temp;
		 
		 
		 jiao=ADChannelx(4)+ADChannelx(4)+ADChannelx(4) + ADChannelx(4)+ADChannelx(4)
          +ADChannelx(4)+ADChannelx(4)+ADChannelx(4) + ADChannelx(4)+ADChannelx(4);           
    Aangle=jiao/20.0;              	 
		Aangle=(Aangle-JSDLD)*0.13;//0.12;	
		angleAZ=jiao/10.0-accOffset;
		
		jiao=ADChannelx(2)+ADChannelx(2)+ADChannelx(2) + ADChannelx(2)+ADChannelx(2)
          +ADChannelx(2)+ADChannelx(2)+ADChannelx(2) + ADChannelx(2)+ADChannelx(2); 
    angleAX=jiao/10.0-accOffset;	
    
    jiao=ADChannelx(3)+ADChannelx(3)+ADChannelx(3) + ADChannelx(3)+ADChannelx(3)
          +ADChannelx(3)+ADChannelx(3)+ADChannelx(3) + ADChannelx(3)+ADChannelx(3); 
    angleAY=jiao/10.0-accOffset;
    
    timer2=micros()-time2;
    //EnableInterrupts;
}

	
	

float angleG2=0,angleFilter2=0;
float angleGY=0,angleFilterY=0;
float angleA2=0;
float TurnSpeed=0;
float gyroTimeTest=0;	
float tempX,tempY,tempZ;

void AngleCalculate (void) 
{
    unsigned long time=0;   
    
    time1=micros();
    
    tempX=angleAX*5/4096*1000/AccSense;
    tempY=angleAY*5/4096*1000/AccSense;
    tempZ=angleAZ*5/4096*1000/AccSense;
    
    
    angleA=-atan2(angleAX,angleAZ)*180/PI;
    gravity=sqrt(tempX*tempX+tempY*tempY+tempZ*tempZ);
     //angleA2=angleAZ/AccSense;
    angleA=FIR(angleA); 
    if(debug) {
    gravityError=fabs(gravity-gravityG);
    /*
    if(gravityError>gravityVibrationGate)// avoid vibration
    {
     weightFlag=5; 
     weight=1;
    } else if(gravityError<gravityGate)  //trust acc more
    {
     weightFlag=0;
     weight=weight2;
    }
    else {
     weightFlag=1; 
     weight=weight1;
    }
    */
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
     angleG2=(testGyroZ1-TLYLDZ1+testGyroZ2-TLYLDZ2)*5/4096*1.5/5.1/GyroSense2/2.0*time/1000;
    
    //TurnSpeed=(testGyroX1-TLYLDX1+testGyroX2-TLYLDX2)*GyroSense/2.0;
    
    
    //angleG=(testGyro-TLYLD)*0.00911;
    //angleFilter=angleA*(1-weight)+(angleG+angleFilter)*weight;
    angleFilter2=angleA*(1-weight)+(angleG2+angleFilter2)*weight;
    //angleFilter=angleG+angleFilter;
    
    timer1=micros()-time1;
}




void speedout (void) 
{ 

 if(fabs(Setpoint-angleFilter2)>35) {
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
		PWMDTY23=LOUT+M_SQL;  //L
		} 
	else if(LOUT<0) 
		{	
		PWMDTY23=0;
		PWMDTY01=M_SQL-LOUT;
    }
  if(ROUT==0) 
  {
    PWMDTY67=0;	
		PWMDTY45=0;
  } 
  else if(ROUT>0) 
  {
    PWMDTY45=ROUT+M_SQR;	
		PWMDTY67=0;
  }
  
  else if(ROUT<0) 
  {
    PWMDTY45=0;	
		PWMDTY67=M_SQR-ROUT;
  }
}


   

void Interrupt_Priority_Set(void){
    INT_CFADDR=0x70;
    INT_CFDATA5=0x05;
    INT_CFADDR=0xD0;
    INT_CFDATA3=0x07;
}

float FIROutput;

void printout (void) 
{ 
  
  /*
  uart0_putf(angleA);
  uart0_putf(angleFilter2);
  uart0_putf(Output);
  uart0_putf(FIROutput);
  uart0_putf(testGyroZ1);
  uart0_putf(testGyroZ2);
   */
  uart0_putf(angleA);
  uart0_putf(angleFilter2);
  //uart0_putf(g_fSpeedControlOutNew);
  uart0_putf(64);
  uart0_putf(LineCenter);
  uart0_putf(LineCenter1);
  
  uart0_putf(fspeed);
  uart0_putf(g_fCarSpeed*MMperPulse*1000/(5*scPeriod));
  
  
	uart0_putstr(" \n\r");	
}


 void PID() {

 Error=Setpoint-Input;
// dErr=angleG2;      //kp=1600,kd=4000;
 dErr=(testGyroZ1-TLYLDZ1+testGyroZ2-TLYLDZ2)/2;   //kp=1900,kd=20;
  {
    /*
    if(fabs(Setpoint-Input)<1) 
      Output=kp*Error*0.3-kd*dErr*0.2;
    else if(fabs(Setpoint-Input)<3) 
      Output=kp*Error*0.8-kd*dErr*0.5;
    else if(fabs(Setpoint-Input)>5)
      Output=kp*Error*1.5-kd*dErr*1.2;
    else   */ 
      Output=kp*Error-kd*dErr;
    val_kp=kp*Error;
    val_kd=kd*dErr;
  }
if(fabs(Error)>45)
 Output=0;

LastErr=Error;
}

void SpeedControl (void) 
{
  float fP, fDelta;
	float fI;
	float fD;
	float P;
	
	g_fCarSpeed = (LspeedJF+RspeedJF) / 2;
	LspeedJF=RspeedJF=0;
	
	g_fCarSpeed*=1.0;
	g_fCarSpeed=FIR2(g_fCarSpeed);

	
	if(fabs(g_fCarSpeed*MMperPulse*scPeriod/1000-SetSpeedMM)<300)//300
	 P=SPEED_CONTROL_P*0.3;
	else if(fabs(g_fCarSpeed*MMperPulse*scPeriod/1000-SetSpeedMM)<600) //500
	 P=SPEED_CONTROL_P*0.6;
	else 
	 P=SPEED_CONTROL_P;
	
	fDelta = SetSpeed;
	fDelta -= g_fCarSpeed;
	
	fP = fDelta * P;
	fI = fDelta * SPEED_CONTROL_I;
	fD = fDelta * SPEED_CONTROL_D;
	g_fSpeedControlIntegral += fI;
	
	if(g_fSpeedControlIntegral > SPEED_CONTROL_OUT_MAX)	
		g_fSpeedControlIntegral = SPEED_CONTROL_OUT_MAX;
	if(g_fSpeedControlIntegral < SPEED_CONTROL_OUT_MIN)  	
		g_fSpeedControlIntegral = SPEED_CONTROL_OUT_MIN;

	g_fSpeedControlOutOld = g_fSpeedControlOutNew;

	g_fSpeedControlOutNew = fP + g_fSpeedControlIntegral-fD;
	
	}
	
 void setSpeed(float myspeed) {
 
 SetSpeed=myspeed*PulseperMM*scPeriod*5/1000; 
}


 void SpeedControlOutput(void) 
{
	float fValue;
	fValue = g_fSpeedControlOutNew - g_fSpeedControlOutOld;
  g_fSpeedControlOut=fValue * (g_nSpeedControlPeriod + 1) /scPeriod + g_fSpeedControlOutOld;
}


void DirectionControl(void) 
{
  //float DError1;
  g_fDirectionControlOutOld = g_fDirectionControlOutNew;
  DError=LineCenter-64;
  //DError1=LineCenter1-64;
  
  DDError=(testGyroX1-TLYLDX1+testGyroX2-TLYLDX2)/2;
  g_fDirectionControlOutNew=DError*KDIR-DDError*DDIR;
  DLastError=DError; 
  
  fspeed=SetSpeedMM+fabs(DError*8);
  if(fspeed>-1200)
   fspeed=-1200;
  setSpeed(fspeed); 
  
}


void DirectionControlOutput(void) {
	float fValue;
	fValue = g_fDirectionControlOutNew - g_fDirectionControlOutOld;
	g_fDirectionControlOut=fValue * (g_nDirectionControlPeriod + 1) /dcPeriod + g_fDirectionControlOutOld;
  //g_fDirectionControlOut=0; 	
}

void SpeedOutCalculate (void) 
{
  int i;
  float temp=0;
  speed=Output-g_fSpeedControlOut;
  
  //g_fDirectionControlOut=0;//for ccd test only
  LOUT=-(speed+g_fDirectionControlOut);
  ROUT=-(speed-g_fDirectionControlOut);
}


int samplecounter=0,sampletime=10, sampleSign=1;
float  tempGyroOffsetZ1=0,tempGyroOffsetZ2=0,tempGyroOffsetX1=0,tempGyroOffsetX2=0;
float tempAccX=0,tempAccY=0,tempAccZ=0;

void AccGyroCalibration() {
   Dly_ms(3000);
  for(;samplecounter<sampletime;samplecounter++) {
    getAD();
    tempGyroOffsetZ1+=testGyroZ1;
    tempGyroOffsetZ2+=testGyroZ2;
    tempGyroOffsetX1+=testGyroX1;
    tempGyroOffsetX2+=testGyroX2;
    tempAccX+=angleAX;
    tempAccY+=angleAY;
    tempAccZ+=angleAZ;
    //Dly_ms(1);
  }
  gyroOffset=tempGyroOffsetZ1/sampletime;
  
  TLYLDZ1=tempGyroOffsetZ1/sampletime;
  TLYLDZ2=tempGyroOffsetZ2/sampletime;
  TLYLDX1=tempGyroOffsetX1/sampletime;
  TLYLDX2=tempGyroOffsetX2/sampletime;
  
  tempAccX/=sampletime;
  tempAccY/=sampletime;
  tempAccZ/=sampletime;
  tempAccX=tempAccX*5/4096*1000/AccSense;
  tempAccY=tempAccY*5/4096*1000/AccSense;
  tempAccZ=tempAccZ*5/4096*1000/AccSense;
  gravityG=sqrt(tempAccX*tempAccX+tempAccY*tempAccY+tempAccZ*tempAccZ);
  

  uart0_putstr("\ngravityG=");
  uart0_putf(gravityG);
  uart0_putchar('\n');
  getAD ();
  angleFilter2=-atan2(angleAX,angleAZ)*180/PI;
  uart0_putstr("\n Acc Gyro Init Done\n");
  
}

int mid(int a,int b,int c) {
  
 int i,j,t;
 int arr[3];
 arr[0]=a;
 arr[1]=b;
 arr[2]=c;
 for(i=0;i<3;i++)
  for(j=i+1;j<3;j++)
   if(arr[i]>arr[j]) {
    t=arr[i];
    arr[i]=arr[j];
    arr[j]=t;
   }
 return arr[1]; 
  
}


char sendSign=0,updateSign=0;
unsigned long speedTimer=0;
unsigned char  CCDBuf[128]=0,CCDBuf2[128]=0,CCDBuf3[128]=0;
  void main()
{ 
  char c; 
  int i;
	setbusclock_80M();
	Interrupt_Priority_Set();    
  Port_Init();     
  PWM_Init();        
  BMQ_Init(); 
  UART0_Init();
  ADC_Init0();
  setADC12bit();
  
  AccGyroCalibration();
  //gyroTimer=micros();//large delay will effect the gyro integration
  while(1){speedout ();getspeed();Dly_ms(5);}  //死区、编码器测试
  PIT_Init();
  speedTimer=millis();
  EnableInterrupts;
	for(;;)
	{

    //LOUT=-speed;
    //ROUT=-speed;
	  //speedout (); 
	  
	  time3=micros();
	  /*DisableInterrupts;
	  for(i=0;i<128;i++)
	   CCDBuf[i]=ADV[i];
	  EnableInterrupts;
	  SendCCDData(CCDBuf);
	  //printout ();
	  */
	  if(CCDDebugSwitch==0)
	    printout ();
	   
	  
	  if(sendSign){
	    if(CCDDebugSwitch==1)
	    SendCCDData(CCDBuf);
	    sendSign=0;
	  } 
	  
    timer3=micros()-time3;
	   //speedout (); 
	}
}


#pragma CODE_SEG __NEAR_SEG NON_BANKED
float ccdSum=0;
float time,timer;
float t1,tr1,t2,tr2,t3,tr3,tr4,t4,tr5,t5;
void interrupt 67 PIT1(void)
{

   int i,j;
   int tempL,tempR;
   time=micros();
   PITTF_PTF0 = 1; //clear interrupts flag
   PORTA_PA7=1;
   if(millis()-speedTimer<500)
    SetSpeed=-50;
   //else
    //SetSpeed=SetSpeedMM*PulseperMM*(scPeriod*5)/1000;
  
  
  t1=micros();
  
   CCDTime++;
   if(CCDTime==1*ccdMultiple) {
   setADC8bit(); 
   
   //t4=micros();
	 RD_CCD(0);   
	 //tr4=micros()-t4;
	 
	 
   } else if(CCDTime>=2*ccdMultiple) {
     setADC8bit(); 
  	 RD_CCD(1); 
  	 //CalculateCCD(); 
  	 CCDTime=0;
  	 
  	 if(sendSign==0){
  	 
  	   t5=micros();
    	//for(i=0;i<128;i++)
    	 //CCDBuf[i]=CCDLine[(int)(i*BL)]; 
	 
      	 
       if(CCDDebugSwitch2==0) 
         for(i=0;i<128;i++)
      	 CCDBuf[i]=ADV[0][i];
      else if(CCDDebugSwitch2==1) 
         for(i=0;i<128;i++)
      	 CCDBuf[i]=ADV[1][i]; 
      else if(CCDDebugSwitch2==2||CCDDebugSwitch2==4) {
       for(i=0;i<128;i++)
      	  CCDBuf2[i]=ADV[0][i];   
         
      } 
       else if(CCDDebugSwitch2==3) {
      	 for(i=0;i<128;i++)
      	  CCDBuf2[i]=ADV[1][i]; 
        
       }
       
       
      if( CCDDebugSwitch2==3||CCDDebugSwitch2==2||CCDDebugSwitch2==4){
        
        
      for(CCDt=0,CCDa=0;CCDt<128;CCDt++) 
  	      {
  	        CCDa=CCDa+CCDBuf2[CCDt];
  	      }  
    	CCDa=CCDa/128;
    	FZ=CCDa*FZBL;
        
        for(CCDt=LineCenter;CCDt<120;CCDt++)  //Rblack
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
	  for(CCDt=LineCenter;CCDt>=0;CCDt--)  //Lblack
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
	      
	      if(Lblack>18&&Rblack<110)
	       trackWidth=Rblack-Lblack;
	      
	      if(Lblack>50)
	       LineCenter=Lblack+trackWidth/2;
	      else if(Rblack<78)
	       LineCenter=Rblack-trackWidth/2;
	      else
         LineCenter=(Lblack+Rblack)/2; 
	      
	      
        if(LineCenter>120)
          LineCenter=120;
        else if(LineCenter<0)
          LineCenter=0;
        //if(abs(LineCenter-(LastC1+LastC2+LastC3)/3)>20)
         //LineCenter=(LastC1+LastC2+LastC3)/3;    
        //LineCenter=LineCenter*0.65+LastC1*0.25+LastC2*0.1;
        LastC1=LineCenter;
        //LineCenter=mid(LastC1,LastC2,LastC3);
        LastC3=LastC2;
        LastC2=LastC1;
        //LastC1=LineCenter;
        
        
        
       if(CCDDebugSwitch2==2) {
        
        	for(i=0;i<Lblack;i++)
      	 	CCDBuf[i]=0;
      	for(;i<Rblack;i++)
      	 	CCDBuf[i]=100;
      	for(;i<128;i++)
      	 	CCDBuf[i]=0;
       } else if(CCDDebugSwitch2==4) {
        
        	for(i=0;i<LineCenter-trackWidth/2;i++)
      	 	CCDBuf[i]=0;
      	for(;i<LineCenter+trackWidth/2;i++)
      	 	CCDBuf[i]=100;
      	for(;i<128;i++)
      	 	CCDBuf[i]=0;
        
       }
       
       ////////////ccd1//////////
        t4=micros();
        for(i=0;i<128;i++)
      	  CCDBuf3[i]=ADV[1][i]; 
        for(CCDt=0,CCDa=0;CCDt<128;CCDt++) 
  	      {
  	        CCDa=CCDa+CCDBuf3[CCDt];
  	      }  
    	CCDa=CCDa/128;
    	FZ1=CCDa*FZBL1;
        
        for(CCDt=LineCenter1;CCDt<120;CCDt++)  //Rblack
	      {
	        if(CCDBuf3[CCDt]-CCDBuf3[CCDt+DCCD]>FZ1) 
	        {
	          Rblack1=CCDt+DCCD; 
	        }
	      }
	  for(CCDt=LineCenter1;CCDt>=0;CCDt--)  //Lblack
	      {
	        if(CCDBuf3[CCDt]-CCDBuf3[CCDt-DCCD]>FZ1) 
	        {
	          Lblack1=CCDt-DCCD; 
	        }
	      }
       LineCenter1=(Lblack1+Rblack1)/2; 
         if(LineCenter1>120||LineCenter1<0)
         LineCenter1=64;

        tr4=micros()-t4;
       }  
  	  sendSign=1;
  	  tr5=micros()-t5;
    }
   }
	 
	 tr1=micros()-t1;
	 
   t2=micros();
	 setADC12bit(); 
   getAD ();
	 AngleCalculate();
	 tr2=micros()-t2;
	 
	 t3=micros();
   getspeed();
   Input=angleFilter2;
   PID();
	 g_nSpeedControlPeriod ++;
	 SpeedControlOutput();	
	 DirectionControlOutput();
	 g_nSpeedControlCount ++;
   if(g_nSpeedControlCount >=scPeriod) 
   { 
        SpeedControl();
  			g_nSpeedControlCount = 0;
  			g_nSpeedControlPeriod = 0;  			
  	}
  	
  	g_nDirectionControlCount++;		
      if(g_nDirectionControlCount >=dcPeriod) 
      {
	  		DirectionControl();
	  		g_nDirectionControlCount = 0;
	  		g_nDirectionControlPeriod = 0;
	  	}
	  SpeedOutCalculate (); 	
   //LOUT=-(Output-speedOutput);
   //ROUT=-(Output-speedOutput);
	 speedout ();   
	 tr3=micros()-t3;
	 
	 timer=micros()-time;
}

#pragma CODE_SEG DEFAULT         

