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

#include"Time_Function.h"
#include <hidef.h>      /* common defines and macros */
#include "derivative.h"      /* derivative-specific definitions */
//#include "includes.h"
#include "ADC.h"
//#include "CCD.h"
/*void CalculateFZ (void) 
{
  for(CCDt=0;CCDt<128;CCDt++) 
	      {
	        CCDa=CCDa+ADV[CCDt];
	      }
	CCDa=CCDa/128;
	FZ=CCDa*0.35;
}

void GetBlack (void) 
{
    for(CCDt=64;CCDt<120;CCDt++)  //Rblack
	      {
	        if(ADV[CCDt]-ADV[CCDt+DCCD]>FZ) 
	        {
	          Rblack=CCDt+DCCD;
	          for(CCDt=Rblack;CCDt<128;CCDt++) 
	          {
	            ADV[CCDt]=4;
	          }
	        }
	      }
	  for(CCDt=64;CCDt>=0;CCDt--)  //Lblack
	      {
	        if(ADV[CCDt]-ADV[CCDt-DCCD]>FZ) 
	        {
	          Lblack=CCDt-DCCD;
	          for(CCDt=Lblack;CCDt>=0;CCDt--) 
	          {
	            ADV[CCDt]=4;
	          }
	        }
	      }
}


void RD_TSL(void) 
{
  byte i=0,tslp=0;
  
  TSL_CLK=1;//起始电平高 
  TSL_SI=0; //起始电平低
  Dly_us(1); //合理的延时
      
  TSL_SI=1; //上升沿
  TSL_CLK=0;//下降沿
  Dly_us(1); //合理延时
      
  TSL_CLK=1;//上升沿
  TSL_SI=0; //下降沿
  Dly_us(1); //合理延时      
  for(i=0;i<128;i++)
  { 
    TSL_CLK=0;//下降沿    
    Dly_us(10); //合理延时
    ADV[tslp]=(byte)(ADCValue(0)>>4);  //AD采集
    ++tslp;
    TSL_CLK=1;//上升沿 
    Dly_us(10); //合理延时    
  }   
}
*/