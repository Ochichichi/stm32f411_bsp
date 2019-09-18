/*
 * algo.c
 *
 *  Created on: Sep 17, 2019
 *      Author: HoaHiep
 */

#include "algo.h"
#include "log.h"

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
        Error_Handler();
    }
    time = HAL_GetTick();
    log_info("Configured L3GD20 Sensor\r\n");
}
void Acc_GetOffset(void)
{
    float accData[3];
    for(uint8_t i=0; i<100; i++)
    {
        BSP_Accelero_GetXYZ(accData);
        AccX_offset += accData[0];
        AccY_offset += accData[1];
        AccZ_offset += accData[2];
    }
    AccX_offset = AccZ_offset / 100;
    AccY_offset = AccZ_offset / 100;
    AccZ_offset = AccZ_offset / 100;
}

void Gyro_UpdateXYZ(float gyroData[3])
{
    timePrev = time;                        // the previous time is stored before the actual time read
    time = HAL_GetTick();                   // actual time read
    elapsedTime = (time - timePrev) / 1000; //divide by 1000 in order to obtain seconds

    BSP_Gyro_GetXYZ(gyroData);
}

void Acc_UpdateXYZ(float accData[3])
{
    BSP_Accelero_GetXYZ(accData);
    accData[0] -= AccX_offset;
    accData[1] -= AccY_offset;
    accData[2] -= AccZ_offset;
}

void Complementary_Filter(float accData[3], float gyroData[3], float *pitch, float *roll)
{
    float pitchAcc, rollAcc;

    // Integrate the gyroscope data -> int(angularSpeed) = angle
    
}
