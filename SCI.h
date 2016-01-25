#ifndef   SCI_H
#define   SCI_H

/*
*********************************************************************************************************
* Description: No
*********************************************************************************************************
*/

void UART0_Init(void);
void uart0_putchar(unsigned char ch);//uart0打印字符
void uart0_putstr(char ch[]);//uart0打印字符串
void uart0_putf(float num);//uart0打印浮点数
char uart0_read();
#define RX_BUFFER_SIZE 128
#define SERIAL_NO_DATA 0
#endif