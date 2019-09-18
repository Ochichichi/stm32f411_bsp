#ifndef __ALGO_H_
#define __ALGO_H_

#include "IMU.h"

void IMU_HWSetup(void);
void Acc_UpdateXYZ(float accData[3]);
void Gyro_UpdateXYZ(float gyroData[3]);
void Mag_UpdateXYZ(float magData[3]);

void Acc_GetOffset(void);
void Gyro_GetOffset(void);

void Complementary_Filter(float accData[3], float gyroData[3], float *pitch, float *roll);
#endif // __ALGO_H_