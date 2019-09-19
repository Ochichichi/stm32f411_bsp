#ifndef __ALGO_H_
#define __ALGO_H_

#include "IMU.h"
#include "IMU_Utils.h"

void IMU_HWSetup(void);
void IMU_Calibration(void);

void Acc_UpdateXYZ(float accData[3]);
void Gyro_UpdateXYZ(float gyroData[3]);
void Mag_UpdateXYZ(float magData[3]);

void Acc_GetOffset(void);
void Gyro_GetOffset(void);
void Mag_GetOffset(void);

void Update_PitchRollYaw(float accData[3], float gyroData[3], \
            float magData[3], float *pitch, float *roll, float *yaw);
#endif // __ALGO_H_