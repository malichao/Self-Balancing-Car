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

#include <hidef.h>            /* common defines and macros */
#include "derivative.h"      /* derivative-specific definitions */


void initPWM(void)
{   
  //Cascade the PWM module 0 and 1 to 16bits for motor on the right
  PWMCTL_CON01 = 1;
  PWME_PWME1 = 0;
  PWMPRCLK = 0X00;    //set clock to 80Mhz
  PWMPOL_PPOL1 = 1;   //Set active voltage level to low
  PWMCLK_PCLK1 = 0;   //set clock source to SA
  PWMCAE_CAE1 = 0;    //set alignment to left
  PWMPER01 = 10000;   //PWM Freq = A/10000 = 8KHz
  PWMDTY01 = 0;   
  PWME_PWME1 = 1; 
 
  //Cascade the PWM module 4 and 5 to 16bits for motor on the right
  PWMCTL_CON45 = 1;
  PWME_PWME5 = 0;  
  PWMPOL_PPOL5 = 1;   //Set active voltage level to low
  PWMCLK_PCLK5 = 0;   //set clock source to SA
  PWMCAE_CAE5 = 0;    //set alignment to left
  PWMPER45 = 10000;   //PWM Freq = A/10000 = 8KHz
  PWMDTY45 = 0;   
  PWME_PWME5 = 1;
  
  //Cascade the PWM module 2 and 3 to 16bits for motor on the right
  PWMCTL_CON23 = 1;
  PWME_PWME3 = 0;
  PWMPOL_PPOL3 = 1;   //Set active voltage level to low
  PWMCLK_PCLK3 = 0;   //set clock source to SB
  PWMCAE_CAE3 = 0;    //set alignment to left
  PWMPER23 = 10000;   //PWM Freq = A/10000 = 8KHz
  PWMDTY23 = 0;   
  PWME_PWME3 = 1; 

  //Cascade the PWM module 6 and 7 to 16bits for motor on the right
  PWMCTL_CON67 = 1;
  PWME_PWME7 = 0;  
  PWMPOL_PPOL7 = 1;   //Set active voltage level to low
  PWMCLK_PCLK7 = 0;   //set clock source to SB
  PWMCAE_CAE7 = 0;    //set alignment to left
  PWMPER67 = 10000;   //PWM Freq = A/10000 = 8KHz
  PWMDTY67 = 0;   
  PWME_PWME7 = 1; 

}