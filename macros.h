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


#ifndef MARCROS_H
#define MARCROS_H

#define PI 3.14159265358979323846 
#define true 1
#define false 0
#define HIGH 1
#define LOW 0

//=== Definition of IO pins === 
#define BMQR_RESET  PORTA_PA4  		//Encoder Reset
#define TSL_SI  PORTA_PA1    	//CCD sensor pin SI
#define TSL_CLK PORTA_PA0    	//CCD sensor pin CLK 
#define TSL_SI1  PORTA_PA3    	//CCD sensor pin SI
#define TSL_CLK1 PORTA_PA2    	//CCD sensor pin CLK 

//=== CCD debugging settings === 
#define PRINT_AD   					//toggle if print the ADC values or binarized values
#define CCD_THRESHOLD 100    	//binarization threshold
#define WINDOW_WIDTH 128     //Set how many pixels to print 0~128
#define CMD_TURN_LEFT '!'
#define CMD_TURN_RIGHT '@'
#define CMD_FORWARD '#'
#define CMD_BACKWARD '$'
#define CMD_RUN '%'
#define CMD_STOP '^'    
#define CMD_SET_SPEED '&'
#define CMD_SET_GYROOFFSET '&'

//=== Definition of Gyros and Accelerometers === 
#define GYRO_PIN_Z1 5
#define GYRO_PIN_X1 6
#define ACC_PIN_X 2
#define ACC_PIN_Y 3
#define ACC_PIN_Z 4

//=== Definition of Speed PID Control === 
#define CAR_SPEED_SET_MAX -500
#define SPEED_CONTROL_OUT_MAX 8000
#define SPEED_CONTROL_OUT_MIN -8000

//=== Definition of Linear CCD Sensor === 
#define DCCD 8



#endif