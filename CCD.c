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
#include <hidef.h>             /* common defines and macros */
#include "derivative.h"        /* derivative-specific definitions */
//#include "includes.h"
#include "ADC.h"



inline void setCLKLow(int time){
  TSL_CLK=0;    //Set the clk to low
  delayUs(time);  //delay for 1us
}
inline void setCLKHigh(int time){
  TSL_CLK=0;    //Set the clk to low
  delayUs(time);  //delay for 1us
}

/*
 * Before sampling the ADC value of the CCD sensor,we must
 * set the SI signal to inform the CCD sensor to get ready.
 * See datasheet for timing detail.
 */
void initCCD(){

    TSL_SI=0;     
    setCLKLow(1);
    
    TSL_SI=1;     
    delayUs(1);   
    
    TSL_SI=0;     
    setCLKHigh(1);
}

/*
 * Read the CCD sensor.Currently there are at most only two sensors on
 * the car.
 */
void readCCD(int num){
    DisableInterrupts;
    InitCCD();
  for(int i=0;i<128;i++){
    setCLKLow(1);
    ADV[num][i]=ADC_Read(num);
    setCLKHigh(1);
  }
  EnableInterrupts;
}