#ifndef __IMU_H
#define __IMU_H

/* Includes ------------------------------------------------------------------*/
#include "stm32_accelero_gyro.h"
#include "lsm303dlhc.h"
#include "l3gd20.h"

typedef enum 
{
    ACCELERO_OK = 0,
    ACCELERO_ERROR = 1,
    ACCELERO_TIMEOUT = 2
} ACCELERO_StatusTypeDef;

typedef enum 
{
    GYRO_OK = 0,
    GYRO_ERROR = 1,
    GYRO_TIMEOUT = 2
} GYRO_StatusTypeDef;

// Accelerometer functions
uint8_t BSP_Accelero_Init(void);
void    BSP_Accelero_Reset(void);
void    BSP_Accelero_Click_ITConfig(void);
void    BSP_Accelero_GetXYZ(int16_t *pDataXYZ);

// Gyroscope functions
uint8_t BSP_Gyro_Init(void);
void    BSP_Gyro_Reset(void);
uint8_t BSP_Gyro_ReadID(void);
void    BSP_Gyro_ITConfig(GYRO_InterruptConfigTypeDef *pIntConfigStruct);
void    BSP_Gyro_EnableIT(uint8_t IntPin);
void    BSP_Gyro_DisableIT(uint8_t IntPin);
void    BSP_Gyro_GetXYZ(float *pfData);

#endif // __IMU_H