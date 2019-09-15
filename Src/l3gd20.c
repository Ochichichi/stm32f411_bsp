/*
 * l3gd20.c
 *
 *  Created on: Sep 14, 2019
 *      Author: HiepNguyen
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
};

/**
  * @brief  Set L3GD20 Initialization.
  * @param  L3GD20_InitStruct: pointer to a L3GD20_InitTypeDef structure 
  *         that contains the configuration setting for the L3GD20.
  * @retval None
  */
void L3GD20_Init(uint16_t InitStruct)
{
    uint8_t ctrl = 0x00;

    /* Configure the low level interface */
    L3GD20_IO_Init();

    /* Write value to MEMS CTRL_REG1 register */
    ctrl = (uint8_t) InitStruct;
    L3GD20_IO_Write(&ctrl, L3GD20_CTRL_REG1_ADDR, 1);

    /* Write value to MEMS CTRL_REG4 register */  
    ctrl = (uint8_t) (InitStruct >> 8);
    L3GD20_IO_Write(&ctrl, L3GD20_CTRL_REG4_ADDR, 1);
}

/**
  * @brief L3GD20 De-initialization
  * @param  None
  * @retval None
  */
void L3GD20_DeInit(void)
{

}

/**
  * @brief Set L3GD20 in low-power mode
  * @param
  * @retval None
  */
void L3GD20_LowPower(uint16_t InitStruct)
{
    uint8_t ctrl = 0x00;

    /* Write value to MEMS CTRL_REG1 register */
    ctrl = (uint8_t) InitStruct;
    L3GD20_IO_Write(&ctrl, L3GD20_CTRL_REG1_ADDR, 1);
}

/**
  * @brief Read ID address of L3GD20
  * @param
  * @retval None
  */
uint8_t L3GD20_ReadID(void)
{
    uint8_t tmp;

    // Init low level
    L3GD20_IO_Init();

    // Read Who I am register
    L3GD20_IO_Read(&tmp, L3GD20_WHO_AM_I_ADDR, 1);

    // Return the ID
    return (uint8_t)tmp;
}

/**
  * @brief Reboot memory content of L3GD20
  * @param
  * @retval None
  */
void L3GD20_RebootCmd(void)
{
    uint8_t tmpReg;

    // Read CTRL_REG5 register
    L3GD20_IO_Read(&tmpReg, L3GD20_CTRL_REG5_ADDR, 1);

    // Enable or Disable the reboot memory
    tmpReg |= L3GD20_BOOT_NORMALMODE;

    // Write value to MEMS CTRL_REG5 register
    L3GD20_IO_Write(&tmpReg, L3GD20_CTRL_REG5_ADDR, 1);
}

/**
  * @brief Set L3GD20 Interrupt INT1 configuration
  * @param
  * @retval None
  */
void L3GD20_INT1InterruptConfig(uint16_t Int1Config)
{
    uint8_t ctrl_cfr = 0x00;
    uint8_t ctrl3 = 0x00;

    /* Read INT1_CFG register */
    L3GD20_IO_Read(&ctrl_cfr, L3GD20_INT1_CFG_ADDR, 1);

    /* Read CTRL_REG3 register */
    L3GD20_IO_Read(&ctrl3, L3GD20_CTRL_REG3_ADDR, 1);

    ctrl_cfr &= 0x80;
    ctrl_cfr |= ((uint8_t) Int1Config >> 8);

    ctrl3 &= 0xDF;
    ctrl3 |= ((uint8_t) Int1Config);   

    /* Write value to MEMS INT1_CFG register */
    L3GD20_IO_Write(&ctrl_cfr, L3GD20_INT1_CFG_ADDR, 1);

    /* Write value to MEMS CTRL_REG3 register */
    L3GD20_IO_Write(&ctrl3, L3GD20_CTRL_REG3_ADDR, 1);
}

/**
  * @brief Enable INT1 or INT2 Interrupt
  * @param
  * @retval None
  */
void L3GD20_EnableIT(uint8_t IntSel)
{
    uint8_t tmpReg;

    /* Read CTRL_REG3 register */
    L3GD20_IO_Read(&tmpReg, L3GD20_CTRL_REG3_ADDR, 1);

    if(IntSel == L3GD20_INT1)
    {
        tmpReg &= 0x7F;	
        tmpReg |= L3GD20_INT1INTERRUPT_ENABLE;
    }
    else if(IntSel == L3GD20_INT2)
    {
        tmpReg &= 0xF7;
        tmpReg |= L3GD20_INT2INTERRUPT_ENABLE;
    }

    /* Write value to MEMS CTRL_REG3 register */
    L3GD20_IO_Write(&tmpReg, L3GD20_CTRL_REG3_ADDR, 1);
}

/**
  * @brief Disable INT1 or INT2 Interrupt
  * @param
  * @retval None
  */
void L3GD20_DisableIT(uint8_t IntSel)
{
    uint8_t tmpReg;

    /* Read CTRL_REG3 register */
    L3GD20_IO_Read(&tmpReg, L3GD20_CTRL_REG3_ADDR, 1);
    
    if(IntSel == L3GD20_INT1)
    {
        tmpReg &= 0x7F;	
        tmpReg |= L3GD20_INT1INTERRUPT_DISABLE;
    }
    else if(IntSel == L3GD20_INT2)
    {
        tmpReg &= 0xF7;
        tmpReg |= L3GD20_INT2INTERRUPT_DISABLE;
    }
    
    /* Write value to MEMS CTRL_REG3 register */
    L3GD20_IO_Write(&tmpReg, L3GD20_CTRL_REG3_ADDR, 1);
}

/**
  * @brief  Set High Pass Filter Modality
  * @param  FilterStruct: contains the configuration setting for the L3GD20.        
  * @retval None
  */
void L3GD20_FilterConfig(uint8_t FilterStruct)
{
    uint8_t tmpReg;

    /* Read CTRL_REG2 register */
    L3GD20_IO_Read(&tmpReg, L3GD20_CTRL_REG2_ADDR, 1);

    tmpReg &= 0xC0;

    /* Configure MEMS: mode and cutoff frequency */
    tmpReg |= FilterStruct;

    /* Write value to MEMS CTRL_REG2 register */
    L3GD20_IO_Write(&tmpReg, L3GD20_CTRL_REG2_ADDR, 1);
}

/**
  * @brief  Enable or Disable High Pass Filter
  * @param  HighPassFilterState: new state of the High Pass Filter feature.
  *      This parameter can be: 
  *         @arg: L3GD20_HIGHPASSFILTER_DISABLE 
  *         @arg: L3GD20_HIGHPASSFILTER_ENABLE
  * @retval None
  */
void L3GD20_FilterCmd(uint8_t HighPassFilterState)
{
    uint8_t tmpReg;

    /* Read CTRL_REG5 register */
    L3GD20_IO_Read(&tmpReg, L3GD20_CTRL_REG5_ADDR, 1);

    tmpReg &= 0xEF;

    tmpReg |= HighPassFilterState;

    /* Write value to MEMS CTRL_REG5 register */
    L3GD20_IO_Write(&tmpReg, L3GD20_CTRL_REG5_ADDR, 1);
}

/**
  * @brief  Get status for L3GD20 data
  * @param  None
  * @retval Data status in a L3GD20 Data
  */
uint8_t L3GD20_GetDataStatus(void)
{
    uint8_t tmpReg;

    /* Read STATUS_REG register */
    L3GD20_IO_Read(&tmpReg, L3GD20_STATUS_REG_ADDR, 1);

    return tmpReg;
}

/**
* @brief  Calculate the L3GD20 angular data.
* @param  pfData: Data out pointer
* @retval None
*/
void L3GD20_ReadXYZAngRate(float *pfData)
{
    uint8_t tmpBuffer[6] ={0};
    int16_t RawData[3] = {0};
    uint8_t tmpReg = 0;
    float sensitivity = 0;
    int i =0;

    L3GD20_IO_Read(&tmpReg, L3GD20_CTRL_REG4_ADDR,1);

    L3GD20_IO_Read(tmpBuffer, L3GD20_OUT_X_L_ADDR,6);

    /* check in the control register 4 the data alignment (Big Endian or Little Endian)*/
    if(!(tmpReg & L3GD20_BLE_MSB))
    {
        for(i=0; i<3; i++)
        {
          RawData[i]=(int16_t)(((uint16_t)tmpBuffer[2*i+1] << 8) + tmpBuffer[2*i]);
        }
    }
    else
    {
        for(i=0; i<3; i++)
        {
          RawData[i]=(int16_t)(((uint16_t)tmpBuffer[2*i] << 8) + tmpBuffer[2*i+1]);
        }
    }

    /* Switch the sensitivity value set in the CRTL4 */
    switch(tmpReg & L3GD20_FULLSCALE_SELECTION)
    {
        case L3GD20_FULLSCALE_250:
            sensitivity=L3GD20_SENSITIVITY_250DPS;
            break;

        case L3GD20_FULLSCALE_500:
            sensitivity=L3GD20_SENSITIVITY_500DPS;
            break;

        case L3GD20_FULLSCALE_2000:
            sensitivity=L3GD20_SENSITIVITY_2000DPS;
            break;
    }
    /* Divide by sensitivity */
    for(i=0; i<3; i++)
    {
        pfData[i]=(float)(RawData[i] * sensitivity);
    }
}