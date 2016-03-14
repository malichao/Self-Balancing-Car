#ifndef IMU_H
#define IMU_H
//=== Definition of Gyros and Accelerometers === 
#define GYRO_PIN_Z1 5
#define GYRO_PIN_X1 6
#define ACC_PIN_X 2
#define ACC_PIN_Y 3
#define ACC_PIN_Z 4

void getAccGyrovalues();
void AccGyroCalibration();
#endif