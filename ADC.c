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

#include <hidef.h>            /* common defines and macros */
#include "derivative.h"      /* derivative-specific definitions */

//---------------------------------------------------------------------
//函 数 名:ADCInit                                                      
//功    能:A/D转换初始化,设置A/D转换时钟频率为1MHz                      
//参    数:无                                                           
//返    回:无                                                           
//---------------------------------------------------------------------
void ADC_Init(void)
{

  //ATD0CTL4: SMP2=1,SMP1=1,SMP0=1,PRS4=0,PRS3=1,PRS2=1,PRS1=1,PRS0=1 
  //ATD0CTL4 = 0b11101111; //设置采样时间和频率,fATDCLK=fBUS/(2 ×(PRS + 1))
  ATD0CTL4 = 0b11100100; //设置采样时间和频率,fATDCLK=fBUS/(2 ×(PRS + 1)) 
  
  //ATD0CTL3: DJM=1,S8C=0,S4C=0,S2C=0,S1C=1,FIFO=0,FRZ1=0,FRZ0=0 
  ATD0CTL3 = 0b10001000; //采样结果右对齐，每个序列的转换个数为1
   
  //ATD0CTL0: ??=0,??=0,??=0,??=0,WRAP3=1,WRAP2=1,WRAP1=1,WRAP0=1 
  ATD0CTL0 = 0b00000001; //AD循环采集到AN1后即可  
  
  //ATD0CTL1: ETRIGSEL=0,SRES1=0,SRES0=0,SMP_DIS=0,ETRIGCH3=1,ETRIGCH2=1
  //,ETRIGCH1=1,ETRIGCH0=1 
  ATD0CTL1 = 0b00001111; //8位分辨率，采样前不卸载内部采样电容   
  
  //ATD0CTL2: ??=0,AFFC=1,ICLKSTP=0,ETRIGLE=0,ETRIGP=0,ETRIGE=0,ASCIE=0,
  //ACMPIE=0 
  ATD0CTL2 = 0b01000000; //下降沿触发，不接受外部信号，禁用ATD比较中断请求      

}

void setADC8bit() {
  
  ATD0CTL1 = 0b00001111; //8位分辨率，采样前不卸载内部采样电容
  
}

void setADC12bit() {
  
  ATD0CTL1 = 0X4F; //12位分辨率,禁用外部触发,采样前不卸载内部采样电容
  
}

void ADC_Init0(void)
{
  ATD0CTL0 = 0X00; //AD循环采集到AN0 (ATD0CTL5_MULT=1)  
  ATD0CTL1 = 0X4F; //12位分辨率,禁用外部触发,采样前不卸载内部采样电容
  ATD0CTL2 = 0b01000000; //快速清零,外部信号下降沿触发，禁止外部触发，禁止AD中断, 禁用ATD比较中断
  ATD0CTL3 = 0b10001000; //采样结果右对齐，转换1个通道
  ATD0CTL4 = 0b00100001; //设置采样时间和频率,fATDCLK=fBUS/(2 ×(PRS + 1))
}
int ADChannelx(unsigned char Channel)
{  
  ATD0CTL5_Cx = Channel;
  while(!ATD0STAT0_SCF);
  return ATD0DR0;
}

//---------------------------------------------------------------------
//函 数 名:ADCValue                                                     
//功    能:1路A/D转换函数,获取通道channel的A/D转换结果                  
//参    数:channel = 通道号                                             
//返    回:该通道的12位A/D转换结果                                      
//---------------------------------------------------------------------
uint ADCValue(uchar channel)
{
  //暂存A/D转换的结果
  uint temp;                        
	ATD0CTL5 = channel;
	//取A/D转换结果                                  
  while(!ATD0STAT0_SCF);
  temp = ATD0DR0;
	return  temp;
}