#ifndef __IMU_H
#define __IMU_H
#include "lsm303dlhc.h"
#include "stm32_accelero_gyro.h"

typedef enum 
{
  ACCELERO_OK = 0,
  ACCELERO_ERROR = 1,
  ACCELERO_TIMEOUT = 2
} ACCELERO_StatusTypeDef;

/* Accelerometer functions */   
uint8_t BSP_Accelero_Init(void);
void    BSP_Accelero_Reset(void);
void    BSP_Accelero_Click_ITConfig(void);
void    BSP_Accelero_GetXYZ(int16_t *pDataXYZ);

#endif // __IMU_H