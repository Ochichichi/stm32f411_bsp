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
float MagX_min = 0, MagY_min = 0, MagZ_min = 0;
float MagX_max = 0, MagY_max = 0, MagZ_max = 0;

float elapsedTime, time, timePrev;
float old_MagData[3] = {0,0,0};

void IMU_Calibration(void);

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
    log_info("Configured L3GD20 Sensor\r\n");
    // IMU_Calibration();
    time = HAL_GetTick();
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

void Mag_GetOffset(void)
{
    float magData[3];

    for(uint8_t i=0; i<200; i++)
    {
        BSP_Magneto_GetXYZ(magData);
        if (magData[0] < MagX_min) MagX_min = magData[0];
        if (magData[0] > MagX_max) MagX_max = magData[0];
        if (magData[1] < MagY_min) MagY_min = magData[1];
        if (magData[1] > MagY_max) MagY_max = magData[1];
        if (magData[2] < MagZ_min) MagZ_min = magData[2];
        if (magData[2] > MagZ_max) MagZ_max = magData[2];
    }
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

void Mag_UpdateXYZ(float magData[3])
{
    BSP_Gyro_GetXYZ(magData);
    magData[0] -= (MagX_min + MagX_max) / 2 ;
    magData[1] -= (MagY_min + MagY_max) / 2 ;
    magData[2] -= (MagZ_min + MagZ_max) / 2 ;
}

void IMU_Calibration(void)
{
    Acc_GetOffset();
    Gyro_GetOffset();
    Mag_GetOffset();
}

void Update_PitchRollYaw(float accData[3], float gyroData[3], float magData[3], float *pitch, float *roll, float *yaw)
{
    float pitchAcc, rollAcc, pitchGyro, rollGyro;
    static float yawGyro = 0;

    // Update value from Accelerometer
    Acc_UpdateXYZ(accData);

    // Update value from Gyroscope
    Gyro_UpdateXYZ(gyroData);

    // Calculate the angles from the gyro
    pitchGyro = gyroData[0] * elapsedTime; // Angle round the X-Axis
    rollGyro  = gyroData[1] * elapsedTime; // Angle round the y-Axis
    yawGyro   = yawGyro + gyroData[2] * elapsedTime;

    // Calculate the angles from the acc
    pitchAcc = (atan2(accData[1], accData[2]) + PI) * RAD_TO_DEG;
    rollAcc  = (atan2(accData[2], accData[0]) + PI) * RAD_TO_DEG;

    // If IMU is up the correct way, use these lines
    pitchAcc -= (float)180.0;
    if (rollAcc > 90)
        rollAcc -= (float)270;
    else {
        rollAcc += (float)90;
    }

    // Complementary filter used to combine the accelerometer and gyro values.
    *pitch = 0.97*(*pitch + pitchGyro) + 0.03*pitchAcc;
    *roll  = 0.97*(*roll  + rollGyro) + 0.03*rollAcc;
    *yaw   =  yawGyro * RAD_TO_DEG;
}
