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
#include "stdio.h"
#include "math.h"
#include "SCI.h"
//---------------------------------------------------------------------
// 函数功能：UART0_Init初始化
// 形式参数：  无
// 函数返回值：无   
//---------------------------------------------------------------------
int rx_buffer_head = 0;
int rx_buffer_tail = 0;
char rx_buffer[RX_BUFFER_SIZE]={0};


void UART0_Init(void)
{
  SCI0CR1 = 0x00; 
  SCI0CR2 = 0x2C;     //接收中断使能，发送接收使能
  SCI0BD  = 130;       //521--9600
  //SCI0BD  = 130;     //波特率配置成38400,fbus=80M
                      //When IREN = 0 then 
                      //SCI baud rate = SCI bus clock / (16 x SBR[12:0])
}



//---------------------------------------------------------------------
// 函数功能：SCI0发送一个字节数据
// 形式参数：  byte ch：发送的一个字节数据
// 函数返回值：无   
//---------------------------------------------------------------------


void uart0_putchar(unsigned char ch)
{
 	if (ch == '\n')  
  {
      while(!(SCI0SR1&0x80)) ;     
      SCI0DRL= 0x0d;       				 //output'CR'
	    return;
  }
  while(!(SCI0SR1&0x80)) ; 		    //keep waiting when not empty  
  SCI0DRL=ch;	
}

//---------------------------------------------------------------------
// 函数功能：SCI0发送字符串数据
// 形式参数：   byte *pBuff     发送缓冲区
//              int Length 发送字节的长度 
// 函数返回值：无   
//---------------------------------------------------------------------
void uart0_sendpacket(byte *pBuf,int pBuf_Length) 
{
  int i;
  for(i=0;i<pBuf_Length;i++)
  {
    while(!(SCI0SR1&0x80));
    SCI0DRL=*(pBuf+i); 
  }
}
 

void uart0_putstr(char ch[])
{
  unsigned char ptr=0;
  while(ch[ptr]){
      uart0_putchar((unsigned char)ch[ptr++]);
  }     
  
}
void uart0_putf(float num)
{
  char buf[20]={0};
  (void)far_sprintf(buf,"%.3f,",num);
  uart0_putstr(buf);
}



char uart0_read()
{
  char data  ;
  if (rx_buffer_head == rx_buffer_tail) {
    return SERIAL_NO_DATA;
  } else {
    data = rx_buffer[rx_buffer_tail];
    rx_buffer_tail++;
    if (rx_buffer_tail == RX_BUFFER_SIZE) { rx_buffer_tail = 0; }
    return data;
  }
}
char uart0_available() {
    if (rx_buffer_head == rx_buffer_tail) {
    return SERIAL_NO_DATA;
    return 1;
  }
}
unsigned char SciRead(){
    
    if(SCI0SR1_RDRF==1)     //表明数据从位移寄存器传输到SCI数据寄存器
      {      SCI0SR1_RDRF=1;     //读取数据寄存器会将RDRF清除  重新置位 
           return SCI0DRL;        //返回数据寄存器的数据
        }
        
}


void SciWrite2(unsigned char sendchar){
    while (!(SCI0SR1&0x80));    
    SCI0DRH=0;    
    SCI0DRL=sendchar;
    }


#pragma CODE_SEG NON_BANKED

interrupt 20 void SCI_RX_IRS(void){
byte RxData,RX;
byte next_head;
DisableInterrupts;
RX=SCI0SR1;
//读状态寄存器，为清零作准备
RxData=(byte)SCI0DRL; //读接收寄存器的值
//uart0_putchar(RxData);
next_head = rx_buffer_head + 1;
if (next_head == RX_BUFFER_SIZE) { next_head = 0; }
if (next_head != rx_buffer_tail) {
        rx_buffer[rx_buffer_head] = RxData;
        rx_buffer_head = next_head;  
}
EnableInterrupts;
}  




