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