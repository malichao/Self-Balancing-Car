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
#include <filters.h>

float coef1[FIR_NUM][FIR_TAPS] = {
  {0.001193,0.010720,0.026164,0.026164,0.010720}, //LOW_PASS,Fs=40,RECTANGLE,wp=10,ws=11
  {0.000000,0.318310,0.500000,0.500000,0.318310}, //LOW_PASS,Fs=20,HAMMING,wp=3,ws=9
  {0.006055,0.092153,0.261644,0.261644,0.092153}, //LOW_PASS,Fs=20,HAMMING,wp=3,ws=9
  {0.006055,0.092153,0.261644,0.261644,0.092153}, //LOW_PASS,Fs=20,HAMMING,wp=3,ws=9
  {0.001193,0.010720,0.026164,0.026164,0.010720}  //LOW_PASS,Fs=200,HAMMING,wp=3,ws=9
};
float gain[FIR_NUM]={13.340246,611015,1.401247,1.401247,13.340246};    
float values[FIR_NUM][FIR_TAPS] = {0}; 

float FIR(int16_t index,float input){
  float output = 0;
  int16_t k=k;
  int16_t taps=FIRPar[index].taps;
  FIRPar[index].[k] = input;
  for (int16_t i=0; i<taps; i++) {
     output += FIRPar[index].coef[i] * FIRPar[index].>values[(i + k) % taps];
  }
  output *= FIRPar[index]gain;
  k = (k+1) % taps;
  FIRPar[index].k=k;
  return output;
}

void initFIR(){
  for(int16_t i=0;i<5;i++){
    FIRPar[i].gain=gain[i];
    FIRPar[i].taps=sizeof(coef)/sizeof(coef[0]);
    FIRPar[i].k=0;
    for(int16_t j=0;j<5;j++){
      FIRPar[i].coef[j]=coef[i][j];
      FIRPar[i].values[j]=0;
    }
  }
}

//A simple bubble sort to calculate the median value
int16_t mid(int16_t arr[]) {
 for(int16_t i=0;i<3;i++)
  for(int16_t j=i+1;j<3;j++)
   if(arr[i]>arr[j]) {
    t=arr[i];
    arr[i]=arr[j];
    arr[j]=t;
   }
 return arr[1]; 
}