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
 * Note:This file is stripped from main.c to make it more readable,there might
 * be some definition problem and won't pass the compliler.
 * */

#include"Time_Function.h"
//#include <hidef.h>             /* common defines and macros */
//#include "derivative.h"        /* derivative-specific definitions */
#include "macros.h"
#include "ADC.h"



static inline void setCLKLow(const uint16_t time){
  TSL_CLK=LOW;    //Set the clk to low
  delayUs(time);  //delay for 1us
}
static inline void setCLKHigh(const uint16_t time){
  TSL_CLK=LOW;    //Set the clk to low
  delayUs(time);  //delay for 1us
}

/*
 * Before sampling the ADC value of the CCD sensor,we must
 * set the SI signal to inform the CCD sensor to get ready.
 * See datasheet for timing detail.
 */
static void startReadingCCD(){
    TSL_SI=LOW;     
    setCLKLow(1);
    
    TSL_SI=HIGH;     
    delayUs(1);   
    
    TSL_SI=LOW;     
    setCLKHigh(1);
}

/*
 * Read the CCD sensor.Currently there are at most only two sensors on
 * the car.
 */
void readCCD(uint16_t num){
  DisableInterrupts;
  startReadingCCD();
  for(int i=0;i<128;i++){
    setCLKLow(1);
    ADV[num][i]=ADC_Read(num);
    setCLKHigh(1);
  }
  EnableInterrupts;
}

void calculateCCD(void){
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