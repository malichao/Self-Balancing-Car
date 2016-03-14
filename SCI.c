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
#include "stdio.h"
#include "math.h"
#include "SCI.h"


int rxBufferHead = 0;
int rxBufferTail = 0;
char rx_buffer[RX_BUFFER_SIZE]={0};


void initUART0(void){
  SCI0CR1 = 0x00; 
  SCI0CR2 = 0x2C;     //enable RX ISR
  SCI0BD  = 130;       //521--9600
  //SCI0BD  = 130;     //set buadrate to 38400 @ fbus=80M
                      //When IREN = 0 then 
                      //SCI baud rate = SCI bus clock / (16 x SBR[12:0])
}

//Send a char through UART0,sending character is using polling mode
//since sending values are not as important as other tasks and it should
//give its priority to other tasks.
void putchar(unsigned char ch){
  while(!(SCI0SR1&0x80)) ; 		    //Simple polling mode
  SCI0DRL=ch;	
}

//Send a string through the UART0,this function calls the putchar function 
//to do the job.
void putstr(unsigned char ch[]){
  int i=0;
  while(ch[i]){
      uart0_putchar((unsigned char)ch[i++]);
  }      
}

//A function to print float type through UART,the function calls the built-in
//far_sprintf to convert the number.
void putf(float num){
  unsigned char buf[20]={0};
  (void)far_sprintf(buf,"%.3f,",num);
  uart0_putstr(buf);
}

//Read the data from RX buffer,which is a cyclic array.It returns 0 if the buffer
//is empty.And it stop receiving if the buffer is full.
char readUART0(){
  char data  ;
  if (rxBufferHead == rxBufferTail) {
    return SERIAL_NO_DATA;
  } else {
    data = rx_buffer[rxBufferTail];
    rxBufferTail++;
    if (rxBufferTail == RX_BUFFER_SIZE) { rxBufferTail = 0; }
    return data;
  }
}

//RX ISR
#pragma CODE_SEG NON_BANKED
interrupt 20 void SCI_RX_IRS(void){
  unsigned char rxData;
  unsigned char nextHead;
  DisableInterrupts;

  rxData=(byte)SCI0DRL;             //Read the RX value in register

  nextHead = rxBufferHead + 1;
  if (nextHead == RX_BUFFER_SIZE) { //If reach the end of buffer,set to 0
    nextHead = 0; 
 }
  if (nextHead != rxBufferTail) {   //If buffer is empty then put it in
    rx_buffer[rxBufferHead] = RxData;
    rxBufferHead = nextHead;  
  }
  EnableInterrupts;
}  