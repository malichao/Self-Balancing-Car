#include "IMU.h"
#inlcude "ADC.h"
#include "times"
#include "macros.h"
#include "filters"

//============= Definition and parameters of gyros and Accelerometers=============
float GyroZ1=0,GyroX1=0;
float angleAccZ=0;
float angleAccX=0;
float angleAccY=0;

//Four gyros: Z1 and Z2 for pitch angle,X1 and X2 for yaw angle 
float GyroOffsetZ1=1950;      //Gyro offset value,set larger to tilt forward
float GyroOffsetZ2=1950;
float GyroOffsetX1=1950;
float GyroOffsetX2=1950;
float GyroCoef=0.536;               //Gyro coefficient,see datasheet for detail     
float AccOffset= 1365;               //Accelerometers offset value,set larger to tilt forward

//The follow values come from the datasheet
float GyroSense=0.67;
float AccSense=800;     //mV

float gravity=0,gravityG=1,gravityError;

//Use complementary filter to fuse the acc and gyro data
void calculateAngle() {
    uint16_t32_t time=micros();

    float tempX,tempY,tempZ;
    tempX=angleAccX*5/4096*1000/AccSense;
    tempY=angleAccY*5/4096*1000/AccSense;
    tempZ=angleAccZ*5/4096*1000/AccSense;
    
    float gravityGate=0.06,gravityVibrationGate=0.2;
    angleAcc=-atan2(angleAccX,angleAccZ)*180/PI;
    gravity=sqrt(tempX*tempX+tempY*tempY+tempZ*tempZ);
    angleAcc=FIR(0,angleAcc); 

    float weight=0.99; 
    float weightMax=0.995,weightMin=0.98;
    //didn't use #if defined because we want to evaluate both method online
    if(debug) {    
      float weightFlag=0;                       
      gravityError=fabs(gravity-gravityG);

      if(gravityError>gravityVibrationGate){  // avoid vibration
        weightFlag=10; 
        weight=1;
      } else{
        weight=weightMin+(weightMax-weightMin)*gravityError/gravityVibrationGate;
        weightFlag=(weight-weightMin)*100;
        }
    }
    
    time=micros()-time;
    if(time>50000)                   //50ms,timeout
     angleGyroDelta=0;               //large delay will effect the gyro int16_tegration
    else
     angleGyroDelta=(GyroZ1-GyroOffsetZ1)*5/4096*1.5/5.1/GyroSense*time/1000;
    
    angleFinal=angleAcc*(1-weight)+(angleGyroDelta+angleFinal)*weight;
}

//Loop unrolling to speed up
void getAccGyroValues () {
    uint16_t32_t timer;
    setADC12bit(); 
    timer=micros();

    float temp=0;
    temp=readADC(GYRO_PIN_Z1)+readADC(GYRO_PIN_Z1)+readADC(GYRO_PIN_Z1)
            +readADC(GYRO_PIN_Z1)+readADC(GYRO_PIN_Z1)+readADC(GYRO_PIN_Z1)
            +readADC(GYRO_PIN_Z1)+readADC(GYRO_PIN_Z1)+readADC(GYRO_PIN_Z1)
            +readADC(GYRO_PIN_Z1);
    temp=temp/10.0;
    GyroZ1=(temp-GyroOffsetZ1)*GyroCoef; 
     
    temp=readADC(GYRO_PIN_X1)+readADC(GYRO_PIN_X1)+readADC(GYRO_PIN_X1)
      +readADC(GYRO_PIN_X1)+readADC(GYRO_PIN_X1)+readADC(GYRO_PIN_X1)
      +readADC(GYRO_PIN_X1)+readADC(GYRO_PIN_X1)+readADC(GYRO_PIN_X1)
      +readADC(GYRO_PIN_X1);  
    temp=temp/10.0;
    GyroX1=(temp-GyroOffsetZ1)*GyroCoef; 
    
    temp=readADC(ACC_PIN_Z)+readADC(ACC_PIN_Z)+readADC(ACC_PIN_Z)
          +readADC(ACC_PIN_Z)+readADC(ACC_PIN_Z)+readADC(ACC_PIN_Z)
          +readADC(ACC_PIN_Z)+readADC(ACC_PIN_Z)+readADC(ACC_PIN_Z)
          +readADC(ACC_PIN_Z);           
    angleAccZ=temp/10.0-AccOffset;
    
    temp=readADC(ACC_PIN_X)+readADC(ACC_PIN_X)+readADC(ACC_PIN_X)
          +readADC(ACC_PIN_X)+readADC(ACC_PIN_X)+readADC(ACC_PIN_X)
          +readADC(ACC_PIN_X)+readADC(ACC_PIN_X)+readADC(ACC_PIN_X)
          +readADC(ACC_PIN_X);  
    angleAccX=temp/10.0-AccOffset;  
    
    temp=readADC(ACC_PIN_Y)+readADC(ACC_PIN_Y)+readADC(ACC_PIN_Y)
          +readADC(ACC_PIN_Y)+readADC(ACC_PIN_Y)+readADC(ACC_PIN_Y)
          +readADC(ACC_PIN_Y)+readADC(ACC_PIN_Y)+readADC(ACC_PIN_Y)
          +readADC(ACC_PIN_Y);  
    angleAccY=temp/10.0-AccOffset;
    
    timer=micros()-timer;
}

void AccGyroCalibration() {
	int16_t samplecounter=0,sampletime=10, sampleSign=1;
  float tempAccX=0,tempAccY=0,tempAccZ=0;
  float  tempGyroOffsetZ1=0,tempGyroOffsetZ2=0,tempGyroOffsetX1=0,tempGyroOffsetX2=0;
	for(;samplecounter<sampletime;samplecounter++) {
		getAccGyroValues();
		tempGyroOffsetZ1+=GyroZ1;
		tempGyroOffsetX1+=GyroX1;
		tempAccX+=angleAccX;
		tempAccY+=angleAccY;
		tempAccZ+=angleAccZ;
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
   
}