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
#ifnded PIDCONTROLLER_H
#define PIDCONTROLLER_H

//=== Definition of Speed PID Control === 
#define SPEED_CONTROL_OUT_MAX 8000
#define SPEED_CONTROL_OUT_MIN -8000

#define DIRECTION_CONTROL_OUT_MAX 5000
#define DIRECTION_CONTROL_OUT_MIN -5000

#define SPEED_CONTROL_PERIOD 10		//50ms @ 5ms timer ISR
#define DIRECTION_CONTROL_PERIOD 4	//20ms @ 5ms timer ISR

void setTarget(const float speed,const float angle);
void PIDControl(int16_t *speedL,int16_t *speedR);
static void balancePID(int16_t *speedL,int16_t *speedR);
static void speedPID(int16_t *speedL,int16_t *speedR);
static void directionPID(int16_t *speedL,int16_t *speedR);

#endif