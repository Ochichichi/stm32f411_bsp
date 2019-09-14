/*
 * l3gd20.c
 *
 *  Created on: Sep 14, 2019
 *      Author: HoaHiep
 */

#include "l3gd20.h"

// Mapping Function Pointer
GYRO_DrvTypeDef L3gd20Drv =
{
    L3GD20_Init,
    L3GD20_DeInit,
    L3GD20_ReadID,
    L3GD20_RebootCmd,
    L3GD20_LowPower,
    L3GD20_INT1InterruptConfig,
    L3GD20_EnableIT,
    L3GD20_DisableIT,
    0,
    0,
    L3GD20_FilterConfig,
    L3GD20_FilterCmd,
    L3GD20_ReadXYZAngRate
}

