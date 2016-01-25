#include <hidef.h>            /* common defines and macros */
#include "derivative.h"      /* derivative-specific definitions */
#include "stdio.h"


void PWM_Init(void)     //原版四通道PWM#################################################
{   
  //*****右电机 0、1级联********
  PWMCTL_CON01 = 1;
  PWME_PWME1 = 0;
  PWMPRCLK = 0X00;    //clockA 1分频:80Mhz
  //PWMSCLA = 5;        //对clock SA 进行2*PWMSCLA = 10分频；pwm clock = clockA/10; 
  PWMPOL_PPOL1 = 1;   //脉冲有效期间为低电平
  PWMCLK_PCLK1 = 0;   //时钟为SA
  PWMCAE_CAE1 = 0;    //左对齐
  PWMPER01 = 10000;   //PWM频率 = A/5000 = 16KHz
  PWMDTY01 = 0;   
  PWME_PWME1 = 1; 
  //******************************
 
  //*****右电机 4、5级联********
  PWMCTL_CON45 = 1;
  PWME_PWME5 = 0;  
  PWMPOL_PPOL5 = 1;   //脉冲有效期间为低电平
  PWMCLK_PCLK5 = 0;   //时钟为A
  PWMCAE_CAE5 = 0;    //左对齐
  PWMPER45 = 10000;   //PWM频率 = A/5000 = 16KHz
  PWMDTY45 = 0;   
  PWME_PWME5 = 1;
  
  //******************************
      //*****左电机 2、3级联********
  PWMCTL_CON23 = 1;
  PWME_PWME3 = 0;
  PWMPOL_PPOL3 = 1;   //脉冲有效期间为低电平
  PWMCLK_PCLK3 = 0;   //时钟为SB
  PWMCAE_CAE3 = 0;    //左对齐
  PWMPER23 = 10000;   //PWM频率 = A/5000 = 16KHz
  PWMDTY23 = 0;   
  PWME_PWME3 = 1; 
  //******************************

   //*****左电机 6、7级联********
  PWMCTL_CON67 = 1;
  PWME_PWME7 = 0;  
  PWMPOL_PPOL7 = 1;   //脉冲有效期间为低电平
  PWMCLK_PCLK7 = 0;   //时钟为B
  PWMCAE_CAE7 = 0;    //左对齐
  PWMPER67 = 10000;   //PWM频率 = A/5000 = 16KHz
  PWMDTY67 = 0;   
  PWME_PWME7 = 1; 
  //******************************
}