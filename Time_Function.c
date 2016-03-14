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

#include"Time_Function.h"
#include <hidef.h>           /* common defines and macros */
#include "derivative.h"      /* derivative-specific definitions */


void setBuscClock80MHz(void){       
  CLKSEL = 0X00;            //disengage PLL to system
  PLLCTL_PLLON = 1;         //turn on PLL
  SYNR = 0xc0 | 0x09;                        
  REFDV = 0x80 | 0x01; 
  POSTDIV = 0x00;           //pllclock=2*osc*(1+SYNR)/(1+REFDV)=160MHz;
  _asm(nop);                //BUS CLOCK=80M
  _asm(nop);
  while(!(CRGFLG_LOCK==1)); //when pll is steady ,then use it;
  CLKSEL_PLLSEL = 1;        //engage PLL to system; 
}

//Delay 1ms
void delayMs(unsigned int ms){
   for(unsigned int i=0;i<ms;i++){
    delayUs(1000);
   }
}

//Use loop unrolling to reduce for statement overhead and achieve
//precise timming.
void delayUs(unsigned int us){   
   for(unsigned int i=0;i<us;i++){ 
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
} 


void initPIT(void) {
  PITCFLMT_PITE = 0;            //Turn off timer 
  PITCE_PCE0 = 1;               //Enable timer 0
  PITMTLD0 = 80 - 1;            //Set timer count value,80 = 1us @80MHz
  PITLD0 = TIMER0_COUNT - 1;    //Set timer overfloat time
  PITINTE_PINTE0 = 1;           //Enable timer ISR
  PITCFLMT_PITE = 1;            //Turn on timer

  PITLD1 = TIMER1_COUNT - 1;    //Set 16 bits timer interval to 5ms
  PITCE_PCE1 = 1;               //Enable timer 1
  PITINTE_PINTE1 = 1;           //Enable timer 1 ISR
}

//Profiling function,read the time in micros
unsigned long micros() { 
  return TimeUs+(TIMER0_COUNT-PITCNT0);
}

//Profiling function,read the time in millis
unsigned long millis() {
  unsigned long time=micros();
  if(time>500)
    return (TIMER0_COUNT-PITCNT0)/1000+TimeMS+1;
  return (TIMER0_COUNT-PITCNT0)/1000+TimeMS;
}

//Timer ISR
#pragma CODE_SEG __NEAR_SEG NON_BANKED
void interrupt 66 PIT0(void){
  PITTF_PTF0 = 1;       //clear interrupts flag
  TimeMSMod+=TIMER0_COUNT%1000;
  if(TimeMSMod>=1000){
   TimeMSMod-=1000;
   TimeMS+=1;
  }
  TimeMS+=54;
  TimeUs+=TIMER0_COUNT;
}
#pragma CODE_SEG DEFAULT      


