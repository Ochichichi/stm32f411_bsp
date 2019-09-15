/*
 * IMU.c
 *
 *  Created on: Sep 14, 2019
 *      Author: HiepNguyen
 */

#include "IMU.h"

static ACCELERO_DrvTypeDef  *AccelerometerDrv;
static GYRO_DrvTypeDef      *GyroscopeDrv;
static MAGNETO_DrvTypeDef   *MagnetometerDrv;
/* ########################### ACCELEROMETER ########################### */
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

    if(Lsm303dlhcAccDrv.ReadID() == I_AM_LMS303DLHC_ACC)
    {
        // Initialize the Accelerometer driver structure
        AccelerometerDrv = &Lsm303dlhcAccDrv;

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
        ctrl = (uint8_t) (LSM303DLHC_FilterStruct.HighPassFilter_Mode_Selection | \
                            LSM303DLHC_FilterStruct.HighPassFilter_CutOff_Frequency | \
                            LSM303DLHC_FilterStruct.HighPassFilter_AOI1 |\
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
    // int16_t SwitchXY = 0;

    if(AccelerometerDrv->GetXYZ!= NULL)
    {
        AccelerometerDrv->GetXYZ(pDataXYZ);
    
        /* Switch X and Y Axes in case of LSM303DLHC MEMS */
        // if(AccelerometerDrv == &Lsm303dlhcAccDrv)
        // { 
        //     SwitchXY  = pDataXYZ[0];
        //     pDataXYZ[0] = pDataXYZ[1];

        //     /* Invert Y Axis to be conpliant with LIS3DSH */
        //     pDataXYZ[1] = -SwitchXY;
        // } 
    }
}
/* ########################### GYROSCOPE ########################### */
/**
  * @brief  Set Gyroscope Initialization.
  * @retval GYRO_OK if no problem during initialization
  */
uint8_t BSP_Gyro_Init(void)
{
    uint8_t ret = GYRO_ERROR;
    uint16_t ctrl = 0x0000;

    GYRO_InitTypeDef            L3GD20_InitStruct;
    GYRO_FilterConfigTypeDef    L3GD20_FilterStruct;

    if((L3gd20Drv.ReadId() == I_AM_L3GD20) || \
        (L3gd20Drv.ReadId() == I_AM_L3GD20_TR))
    {
        // Initialize the GyroScope driver structure
        GyroscopeDrv = &L3gd20Drv;

        // MEMS Configuration
        // Fill the Gyro Init structure
        L3GD20_InitStruct.Power_Mode        = L3GD20_MODE_ACTIVE;
        L3GD20_InitStruct.Output_DataRate   = L3GD20_OUTPUT_DATARATE_1;
        L3GD20_InitStruct.Axes_Enable       = L3GD20_AXES_ENABLE;
        L3GD20_InitStruct.Band_Width        = L3GD20_BANDWIDTH_4;
        L3GD20_InitStruct.BlockData_Update  = L3GD20_BlockDataUpdate_Continous;
        L3GD20_InitStruct.Endianness        = L3GD20_BLE_LSB;
        L3GD20_InitStruct.Full_Scale        = L3GD20_FULLSCALE_500; 

        // Configure MEMS: data rate, power mode, full scale and axes
        ctrl =  (uint16_t)(L3GD20_InitStruct.Power_Mode | \
                            L3GD20_InitStruct.Output_DataRate | \
                            L3GD20_InitStruct.Axes_Enable | \
                            L3GD20_InitStruct.Band_Width);

        ctrl |= (uint16_t)((L3GD20_InitStruct.BlockData_Update | \
                            L3GD20_InitStruct.Endianness | \
                            L3GD20_InitStruct.Full_Scale) << 8);
        // Configure the GYRO main parameter
        GyroscopeDrv->Init(ctrl);
        
        // Fill the Gyro Filter Structure
        L3GD20_FilterStruct.HighPassFilter_Mode_Selection   = L3GD20_HPM_NORMAL_MODE_RES;
        L3GD20_FilterStruct.HighPassFilter_CutOff_Frequency = L3GD20_HPFCF_0;

        ctrl = (uint8_t) (L3GD20_FilterStruct.HighPassFilter_Mode_Selection | \
                            L3GD20_FilterStruct.HighPassFilter_CutOff_Frequency);
        // Configure the GYRO filter parameter
        GyroscopeDrv->FilterConfig(ctrl);
        GyroscopeDrv->FilterCmd(L3GD20_HIGHPASSFILTER_ENABLE);

        ret = GYRO_OK;
    }

    return ret;
}

/**
  * @brief  Read ID of Gyroscope component.
  * @retval ID
  */
uint8_t BSP_Gyro_ReadID(void)
{
    uint8_t id = 0x00;

    if(GyroscopeDrv->ReadId != NULL)
    {
        id = GyroscopeDrv ->ReadId();
    }

    return id;
}

/**
  * @brief  Reboot memory content of Gyroscope.
  */
void BSP_Gyro_Reset(void)
{
    if(GyroscopeDrv->Reset != NULL)
    {
        GyroscopeDrv->Reset();
    }
}

/**
  * @brief  Configures INT1 interrupt.
  * @param  pIntConfig: pointer to a L3GD20_InterruptConfig_TypeDef 
  *         structure that contains the configuration setting for the L3GD20 Interrupt.
  */
void BSP_Gyro_ITConfig(GYRO_InterruptConfigTypeDef *pIntConfig)
{
    uint16_t interruptconfig = 0x0000;

    if(GyroscopeDrv->ConfigIT != NULL)
    {
        /* Configure latch Interrupt request and axe interrupts */                   
        interruptconfig |= ((uint8_t)(pIntConfig->Latch_Request| \
                                      pIntConfig->Interrupt_Axes) << 8);
    
        interruptconfig |= (uint8_t)(pIntConfig->Interrupt_ActiveEdge);

        GyroscopeDrv->ConfigIT(interruptconfig);
    }
}

/**
  * @brief  Enables INT1 or INT2 interrupt.
  * @param  IntPin: Interrupt pin 
  *      This parameter can be: 
  *        @arg L3GD20_INT1
  *        @arg L3GD20_INT2
  */
void BSP_Gyro_EnableIT(uint8_t IntPin)
{
    if(GyroscopeDrv->EnableIT != NULL)
    {
        GyroscopeDrv->EnableIT(IntPin);
    }
}

/**
  * @brief  Disables INT1 or INT2 interrupt.
  * @param  IntPin: Interrupt pin 
  *      This parameter can be: 
  *        @arg L3GD20_INT1
  *        @arg L3GD20_INT2
  */
void BSP_Gyro_DisableIT(uint8_t IntPin)
{
    if(GyroscopeDrv->DisableIT != NULL)
    {
        GyroscopeDrv->DisableIT(IntPin);
    }
}
  
/**
  * @brief  Get XYZ angular acceleration.
  * @param  pfData: pointer on floating array
  */
void BSP_Gyro_GetXYZ(float *pfData)
{
    if(GyroscopeDrv->GetXYZ!= NULL)
    {
        GyroscopeDrv->GetXYZ(pfData);
    }
}

/* ########################### MAGNETOMETER ########################### */
/**
  * @brief  Set Magnetometer Initialization.
  * @retval MAGNETO_OK if no problem during initialization
  */
uint8_t BSP_Magneto_Init(void)
{
    uint8_t ret = MAGNETO_ERROR;
    uint16_t ctrl = 0x0000;
    uint8_t gain = 0x00;

    MAGNETO_InitTypeDef         LSM303DLHC_InitStruct;

    if(Lsm303dlhcMagDrv.ReadID() == I_AM_LMS303DLHC_MAG)
    {
        MagnetometerDrv = &Lsm303dlhcMagDrv;

        // Fill the Magneto paramter
        LSM303DLHC_InitStruct.Power_Mode            = LSM303DLHC_CONTINUOS_CONVERSION;
        LSM303DLHC_InitStruct.MagOutput_DataRate    = LSM303DLHC_ODR_15_HZ;
        LSM303DLHC_InitStruct.MagFull_Scale         = LSM303DLHC_FS_1_3_GA;
        LSM303DLHC_InitStruct.MagTemperature        = LSM303DLHC_TEMPSENSOR_DISABLE;

        // Configure MEMS: power mode, data rate, scale and temperature
        ctrl =  (uint16_t)(LSM303DLHC_InitStruct.MagTemperature | \
                            LSM303DLHC_InitStruct.MagOutput_DataRate);
        ctrl |= (uint16_t)(LSM303DLHC_InitStruct.Power_Mode << 8);

        gain = LSM303DLHC_InitStruct.MagFull_Scale;

        // Configure MAGNETO paramter
        MagnetometerDrv->Init(ctrl, gain);

        ret = MAGNETO_OK;
    }

    return ret;
}
/**
  * @brief  Read ID of Magnetometer component.
  * @retval ID
  */
uint8_t BSP_Magneto_ReadID(void)
{
    uint8_t id = 0x00;

    if(MagnetometerDrv->ReadID != NULL)
    {
        id = MagnetometerDrv ->ReadID();
    }

    return id;
}

/**
  * @brief  Get XYZ axes magnetometer.
  * @param  pDataXYZ: Pointer to 3 angular magnetometer axes.  
  *                   pDataXYZ[0] = X axis, pDataXYZ[1] = Z axis, pDataXYZ[2] = Y axis
  */
void BSP_Magneto_GetXYZ(float *mDataXYZ)
{
    if(MagnetometerDrv->GetXYZ!= NULL)
    {
        MagnetometerDrv->GetXYZ(mDataXYZ);
    }
}
