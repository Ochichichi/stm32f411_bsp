/*
 * algo.c
 *
 *  Created on: Sep 17, 2019
 *      Author: HoaHiep
 */

#include "algo.h"
#include "log.h"
#include <math.h>

float AccX_offset = 0, AccY_offset = 0, AccZ_offset = 0;
float GyroX_offset = 0, GyroY_offset = 0, GyroZ_offset = 0;

float elapsedTime, time, timePrev;

void IMU_HWSetup(void)
{
    /* Init on-board IMU */
    log_info("Initializing Accelerometer ...\r\n");
    BSP_Accelero_Init();
    BSP_Magneto_Init();
    log_info("Configured LSM303DLHC sensor\r\n");

    log_info("Initializing Gyroscope ...\r\n");
    if(BSP_Gyro_Init() != GYRO_OK)
    {
        log_error("Failed to configure L3GD20 Sensor\r\n");
    }
    time = HAL_GetTick();
    log_info("Configured L3GD20 Sensor\r\n");
}

void Acc_GetOffset(void)
{
    float accData[3];
    for(uint8_t i=0; i<200; i++)
    {
        BSP_Accelero_GetXYZ(accData);
        AccX_offset += accData[0];
        AccY_offset += accData[1];
        AccZ_offset += accData[2];
    }
    // Calculate Acc offset
    AccX_offset = AccZ_offset / 200;
    AccY_offset = AccZ_offset / 200;
    AccZ_offset = AccZ_offset / 200;
}

void Gyro_GetOffset(void)
{
    float gyroData[3];
    for(uint8_t i=0; i<200; i++)
    {
        BSP_Gyro_GetXYZ(gyroData);
        GyroX_offset += gyroData[0];
        GyroY_offset += gyroData[1];
        GyroZ_offset += gyroData[2];
    }
    // Calculate Gyro offset
    GyroX_offset = GyroX_offset / 200;
    GyroY_offset = GyroY_offset / 200;
    GyroZ_offset = GyroZ_offset / 200;
}

void Acc_UpdateXYZ(float accData[3])
{
    BSP_Accelero_GetXYZ(accData);
    accData[0] -= AccX_offset;
    accData[1] -= AccY_offset;
    accData[2] -= AccZ_offset;
}

void Gyro_UpdateXYZ(float gyroData[3])
{
    timePrev = time;                        // the previous time is stored before the actual time read
    time = HAL_GetTick();                   // actual time read
    elapsedTime = (time - timePrev) / 1000; //divide by 1000 in order to obtain seconds

    BSP_Gyro_GetXYZ(gyroData);

    gyroData[0] -= GyroX_offset;
    gyroData[1] -= GyroY_offset;
    gyroData[2] -= GyroZ_offset;
}


void Complementary_Filter(float accData[3], float gyroData[3], float *pitch, float *roll)
{
    float pitchAcc, rollAcc, pitchGyro, rollGyro;

    // Update value from Accelerometer
    Acc_UpdateXYZ(accData);
    // Update value from Gyroscope
    Gyro_UpdateXYZ(gyroData);

    // Calculate the angles from the gyro
    pitchGyro = gyroData[0]*elapsedTime; // Angle round the X-Axis
    rollGyro  = gyroData[1]*elapsedTime; // Angle round the y-Axis

    // Calculate the angles from the acc
    pitchAcc = (atan2(accData[1], accData[2]) + PI) * RAD_TO_DEG;
    rollAcc  = (atan2(accData[2], accData[0]) + PI) * RAD_TO_DEG;

    //If IMU is up the correct way, use these lines
    pitchAcc -= (float)180.0;
    if (rollAcc > 90)
        rollAcc -= (float)270;
    else {
        rollAcc += (float)90;
    }
    
    //Complementary filter used to combine the 
    // accelerometer and gyro values.
    *pitch = 0.97*(*pitch + pitchGyro) + 0.03*pitchAcc;
    *roll  = 0.97*(*roll  + rollGyro) + 0.03*rollAcc;
}
