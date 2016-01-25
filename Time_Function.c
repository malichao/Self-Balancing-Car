#include"Time_Function.h"
#include <hidef.h>      /* common defines and macros */
#include "derivative.h"      /* derivative-specific definitions */


void setbusclock_80M(void)
{       
  CLKSEL = 0X00;    //disengage PLL to system
  PLLCTL_PLLON = 1;   //turn on PLL
  SYNR = 0xc0 | 0x09;                        
  REFDV = 0x80 | 0x01; 
  POSTDIV = 0x00;       //pllclock=2*osc*(1+SYNR)/(1+REFDV)=160MHz;
  _asm(nop);          //BUS CLOCK=80M
  _asm(nop);
  while(!(CRGFLG_LOCK==1));   //when pll is steady ,then use it;
  CLKSEL_PLLSEL = 1;          //engage PLL to system; 
}

void Dly_ms(unsigned int ms)
{
   unsigned int i,j;
   if (ms<1) ms=1;
   for(i=1;i<=ms;i++)
   for(j=0;j<1000;j++) {
    _asm(nop);_asm(nop);_asm(nop);_asm(nop);
      _asm(nop);_asm(nop);_asm(nop);_asm(nop);
      _asm(nop);_asm(nop);_asm(nop);_asm(nop);
      _asm(nop);_asm(nop);_asm(nop);_asm(nop);
      _asm(nop);_asm(nop);_asm(nop);_asm(nop);
      _asm(nop);_asm(nop);_asm(nop);_asm(nop);
      _asm(nop);_asm(nop);_asm(nop);_asm(nop);
      _asm(nop);_asm(nop);_asm(nop);_asm(nop);  
      _asm(nop);_asm(nop);_asm(nop);_asm(nop);
      _asm(nop);_asm(nop);_asm(nop);_asm(nop);
      
      _asm(nop);_asm(nop);_asm(nop);_asm(nop);
      _asm(nop);_asm(nop);_asm(nop);_asm(nop);
      _asm(nop);_asm(nop);_asm(nop);_asm(nop);
      _asm(nop);_asm(nop);_asm(nop);_asm(nop);
      _asm(nop);_asm(nop);_asm(nop);_asm(nop);
      _asm(nop);_asm(nop);_asm(nop);_asm(nop);  
      _asm(nop);_asm(nop);_asm(nop);_asm(nop);
   }
    
}

void Dly_us(unsigned int us)
{
   unsigned int ii;    
   for(ii=0;ii<us;ii++)
   {
      
#ifdef BUSCLOCK32M
      _asm(nop);_asm(nop);_asm(nop);_asm(nop);
      _asm(nop);_asm(nop);_asm(nop);_asm(nop);
      _asm(nop);_asm(nop);_asm(nop);_asm(nop);
      _asm(nop);_asm(nop);_asm(nop);_asm(nop); 
      
      _asm(nop);_asm(nop);_asm(nop);_asm(nop);
      _asm(nop);_asm(nop);_asm(nop);_asm(nop);
      _asm(nop);_asm(nop);_asm(nop);_asm(nop);
      _asm(nop);_asm(nop);_asm(nop);_asm(nop);  
#endif

#ifdef BUSCLOCK64M
      _asm(nop);_asm(nop);_asm(nop);_asm(nop);
      _asm(nop);_asm(nop);_asm(nop);_asm(nop);
      _asm(nop);_asm(nop);_asm(nop);_asm(nop);
      _asm(nop);_asm(nop);_asm(nop);_asm(nop); 
      
      _asm(nop);_asm(nop);_asm(nop);_asm(nop);
      _asm(nop);_asm(nop);_asm(nop);_asm(nop);
      _asm(nop);_asm(nop);_asm(nop);_asm(nop);
      _asm(nop);_asm(nop);_asm(nop);_asm(nop);
      
      _asm(nop);_asm(nop);_asm(nop);_asm(nop);
      _asm(nop);_asm(nop);_asm(nop);_asm(nop);
      _asm(nop);_asm(nop);_asm(nop);_asm(nop);
      _asm(nop);_asm(nop);_asm(nop);_asm(nop); 
      
      _asm(nop);_asm(nop);_asm(nop);_asm(nop);
      _asm(nop);_asm(nop);_asm(nop);_asm(nop);
      _asm(nop);_asm(nop);_asm(nop);_asm(nop);
      _asm(nop);_asm(nop);_asm(nop);_asm(nop);  
#endif

#ifdef BUSCLOCK80M
      _asm(nop);_asm(nop);_asm(nop);_asm(nop);
      _asm(nop);_asm(nop);_asm(nop);_asm(nop);
      _asm(nop);_asm(nop);_asm(nop);_asm(nop);
      _asm(nop);_asm(nop);_asm(nop);_asm(nop);
      _asm(nop);_asm(nop);_asm(nop);_asm(nop);
      _asm(nop);_asm(nop);_asm(nop);_asm(nop);
      _asm(nop);_asm(nop);_asm(nop);_asm(nop);
      _asm(nop);_asm(nop);_asm(nop);_asm(nop);  
      _asm(nop);_asm(nop);_asm(nop);_asm(nop);
      _asm(nop);_asm(nop);_asm(nop);_asm(nop);
      
      _asm(nop);_asm(nop);_asm(nop);_asm(nop);
      _asm(nop);_asm(nop);_asm(nop);_asm(nop);
      _asm(nop);_asm(nop);_asm(nop);_asm(nop);
      _asm(nop);_asm(nop);_asm(nop);_asm(nop);
      _asm(nop);_asm(nop);_asm(nop);_asm(nop);
      _asm(nop);_asm(nop);_asm(nop);_asm(nop);  
      _asm(nop);_asm(nop);_asm(nop);_asm(nop);
      //_asm(nop);_asm(nop);_asm(nop);_asm(nop);     
#endif
      
   }
} //end of Dly_us()



void Dly_25ns(unsigned int ns)
{
   unsigned int ii;    
   for(ii=0;ii<ns;ii++)
   {
#ifdef BUSCLOCK80M
  
      _asm(nop);
      //_asm(nop);_asm(nop);_asm(nop);_asm(nop);     
#endif
      
   }
} //end of Dly_us()



  
  


 /*   for micros(),millis()
void PIT_Init(void) {
  PITCFLMT_PITE=0; //定时中断通道0 关
  PITCE_PCE0=1;//定时器通道0 使能
  PITMTLD0=80-1;//8 位定时器初值设定。80 分频，在80MHzBusClock 下，为1MHz。即1us.
  PITLD0=50000-1;//16 位定时器初值设定。50ms溢出
  PITINTE_PINTE0=1;//定时器中断通道0 中断使能
  PITCFLMT_PITE=1;//定时器通道0 使能
}
*/

void PIT_Init(void) {
  PITCFLMT_PITE=0; //定时中断通道0 关
  PITCE_PCE0=1;//定时器通道0 使能
  PITMTLD0=80-1;//8 位定时器初值设定。80 分频，在80MHzBusClock 下，为1MHz。即1us.
  PITLD0=54321-1;//16 位定时器初值设定。50ms溢出
  PITINTE_PINTE0=1;//定时器中断通道0 中断使能
  PITCFLMT_PITE=1;//定时器通道0 使能
  
  PITLD1=5000-1;//16 位定时器初值设定。2ms溢出
  PITCE_PCE1=1;//定时器通道1 使能
  PITINTE_PINTE1=1; //定时器中断通道1 中断使能
}

 /*
void setTimer0(unsigned long)//定时中断初始化函数50MS 定时中断设置
{
  
}
 */
 
unsigned long micros() { 
 return time_us+(54321-PITCNT0);
}

unsigned long millis() {
unsigned long mm=micros();
if(mm>500)
 return (54321-PITCNT0)/1000+time_ms+1;
return (54321-PITCNT0)/1000+time_ms;

}


     
#pragma CODE_SEG __NEAR_SEG NON_BANKED

void interrupt 66 PIT0(void)
{

    PITTF_PTF0 = 1; //clear interrupts flag
    time_ms_mod+=321;
    if(time_ms_mod>=1000){
     time_ms_mod-=1000;
     time_ms+=1;
    }
    time_ms+=54;
    time_us+=54321;
      
}

#pragma CODE_SEG DEFAULT      


