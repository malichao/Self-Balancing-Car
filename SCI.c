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


int16_t rxBufferHead = 0;
int16_t rxBufferTail = 0;
int16_t rx_buffer[RX_BUFFER_SIZE]={0};


void initUART0(void){
  SCI0CR1 = 0x00; 
  SCI0CR2 = 0x2C;     //enable RX ISR
  SCI0BD  = 130;       //set buadrate to 38400 @ fbus=80M
  //SCI0BD  = 130;     
}

//Send a char through UART0,sending character is using polling mode
//since sending values are not as important as other tasks and it should
//give its priority to other tasks.
void putchar(uint8_t ch){
  while(!(SCI0SR1&0x80)) ; 		    //Simple polling mode
  SCI0DRL=ch;	
}

//Send a string through the UART0,this function calls the putchar function 
//to do the job.
void putstr(uint8_t ch[]){
  int16_t i=0;
  while(ch[i]){
      putchar((uint8_t)ch[i++]);
  }      
}

//A function to print16_t float type through UART,the function calls the built-in
//far_sprint16_tf to convert the number.
void putf(float num){
  uint8_t buf[20]={0};
  (void)far_sprint16_tf(buf,"%.3f,",num);
  putstr(buf);
}

//Read the data from RX buffer,which is a cyclic array.It returns 0 if the buffer
//is empty.And it stop receiving if the buffer is full.
int8_t readUART0(){
  int8_t data  ;
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
int16_terrupt 20 void SCI_RX_IRS(void){
  uint8_t rxData;
  uint8_t nextHead;
  Disableint16_terrupts;

  rxData=(byte)SCI0DRL;             //Read the RX value in register

  nextHead = rxBufferHead + 1;
  if (nextHead == RX_BUFFER_SIZE) { //If reach the end of buffer,set to 0
    nextHead = 0; 
 }
  if (nextHead != rxBufferTail) {   //If buffer is empty then put it in
    rx_buffer[rxBufferHead] = RxData;
    rxBufferHead = nextHead;  
  }
  Enableint16_terrupts;
}  