/*
 * IMU.c
 *
 *  Created on: Sep 14, 2019
 *      Author: HoaHiep
 */

#include "IMU.h"

static ACCELERO_DrvTypeDef *AccelerometerDrv;

/**
  * @brief  Set Accelerometer Initialization.
  * @retval ACCELERO_OK if no problem during initialization
  */

uint8_t BSP_Accelero_Init(void)
{
    uint8_t ret = ACCELERO_ERROR;
    uint16_t ctrl = 0x0000;

    ACCELERO_InitTypeDef            LSM303DLHC_InitStruct;
    ACCELERO_FilterConfigTypeDef    LSM303DLHC_FilterStruct;

    if(Lsm303dlhcDrv.ReadID() == I_AM_LMS303DLHC)
    {
        // Initialize the Accelerometer driver structure
        AccelerometerDrv = &Lsm303dlhcDrv;

        // MEMS configuration
        // Fill the Accelerometer structure
        LSM303DLHC_InitStruct.Power_Mode = LSM303DLHC_NORMAL_MODE;
        LSM303DLHC_InitStruct.AccOutput_DataRate = LSM303DLHC_ODR_50_HZ;
        LSM303DLHC_InitStruct.Axes_Enable = LSM303DLHC_AXES_ENABLE;
        LSM303DLHC_InitStruct.AccFull_Scale = LSM303DLHC_FULLSCALE_2G;
        LSM303DLHC_InitStruct.BlockData_Update = LSM303DLHC_BlockUpdate_Continous;
        LSM303DLHC_InitStruct.Endianness = LSM303DLHC_BLE_LSB;
        LSM303DLHC_InitStruct.High_Resolution = LSM303DLHC_HR_ENABLE;

        // Configure MEMS: data rate, power mode, full scale and axes
        ctrl |= (LSM303DLHC_InitStruct.Power_Mode | \
                LSM303DLHC_InitStruct.AccOutput_DataRate | \
                LSM303DLHC_InitStruct.Axes_Enable);
        
        ctrl |= ((LSM303DLHC_InitStruct.BlockData_Update | \
                LSM303DLHC_InitStruct.Endianness | \
                LSM303DLHC_InitStruct.AccFull_Scale | \
                LSM303DLHC_InitStruct.High_Resolution) << 8);
        
        // Configure the Accelerometer main parameter
        AccelerometerDrv->Init(ctrl);

        // Fill the Accelerometer LPF structure
        LSM303DLHC_FilterStruct.HighPassFilter_Mode_Selection =LSM303DLHC_HPM_NORMAL_MODE;
        LSM303DLHC_FilterStruct.HighPassFilter_CutOff_Frequency = LSM303DLHC_HPFCF_16;
        LSM303DLHC_FilterStruct.HighPassFilter_AOI1 = LSM303DLHC_HPF_AOI1_DISABLE;
        LSM303DLHC_FilterStruct.HighPassFilter_AOI2 = LSM303DLHC_HPF_AOI2_DISABLE;

        // Configure MEMS: mode, cutoff frquency, Filter status, Click, AOI1 and AOI2
        ctrl = (uint8_t) (LSM303DLHC_FilterStruct.HighPassFilter_Mode_Selection |\
                            LSM303DLHC_FilterStruct.HighPassFilter_CutOff_Frequency|\
                            LSM303DLHC_FilterStruct.HighPassFilter_AOI1|\
                            LSM303DLHC_FilterStruct.HighPassFilter_AOI2);
        
        // Configure the Accelerometer main LPF parameter
        AccelerometerDrv->FilterConfig(ctrl);

        ret = ACCELERO_OK;
    }
    else {
        ret = ACCELERO_ERROR;
    }

    return ret;
}

/**
  * @brief  Reboot memory content of Accelerometer.
  */
void BSP_Accelero_Reset(void)
{
    if(AccelerometerDrv->Reset != NULL)
    {
        AccelerometerDrv->Reset();
    }  
}

/**
  * @brief  Configure Accelerometer click IT. 
  */
void BSP_Accelero_Click_ITConfig(void)
{
    if(AccelerometerDrv->ConfigIT!= NULL)
    {
        AccelerometerDrv->ConfigIT();
    }
}

/**
  * @brief  Get XYZ axes acceleration.
  * @param  pDataXYZ: Pointer to 3 angular acceleration axes.  
  *                   pDataXYZ[0] = X axis, pDataXYZ[1] = Y axis, pDataXYZ[2] = Z axis
  */
void BSP_Accelero_GetXYZ(int16_t *pDataXYZ)
{
    int16_t SwitchXY = 0;

    if(AccelerometerDrv->GetXYZ!= NULL)
    {
        AccelerometerDrv->GetXYZ(pDataXYZ);
    
        /* Switch X and Y Axes in case of LSM303DLHC MEMS */
        if(AccelerometerDrv == &Lsm303dlhcDrv)
        { 
            SwitchXY  = pDataXYZ[0];
            pDataXYZ[0] = pDataXYZ[1];

            /* Invert Y Axis to be conpliant with LIS3DSH */
            pDataXYZ[1] = -SwitchXY;
        } 
    }
}