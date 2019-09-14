/*
 * stm32_accelero_gyro.c
 *
 *  Created on: Sep 14, 2019
 *      Author: HoaHiep
 */

/* Includes ------------------------------------------------------------------*/
#include "stm32_accelero_gyro.h"
#include "log.h"

uint32_t I2cxTimeout = I2Cx_TIMEOUT_MAX;
uint32_t SpixTimeout = SPIx_TIMEOUT_MAX;

static I2C_HandleTypeDef I2cHandle;
static SPI_HandleTypeDef SpiHandle;

// I2Cx bus function
static void     I2Cx_Init(void);
static void     I2Cx_WriteData(uint16_t addr, uint8_t reg, uint8_t value);
static uint8_t  I2Cx_ReadData(uint16_t addr, uint8_t reg);
static void     I2Cx_Error(void);
static void     I2Cx_MspInit(I2C_HandleTypeDef *hi2c);

/* SPIx bus function */
static void    SPIx_Init(void);
static uint8_t SPIx_WriteRead(uint8_t byte);
static void    SPIx_Error (void);
static void    SPIx_MspInit(SPI_HandleTypeDef *hspi);

// Link functions for ACCELERO / COMPASS peripheral
void    LSM303DLHC_IO_Init(void);
void    LSM303DLHC_IO_ITConfig(void);
void    LSM303DLHC_IO_Write(uint16_t deviceAddr, uint8_t registerAddr, uint8_t value);
uint8_t LSM303DLHC_IO_Read(uint16_t deviceAddr, uint8_t registerAddr);

// Link function for GYRO peripheral
void    L3GD20_IO_Init(void);
void    L3GD20_IO_Write(uint8_t *pBuffer, uint8_t writeAddr, uint16_t numByteToWrite);
void    L3GD20_IO_Read(uint8_t *pBuffer, uint8_t readAddr, uint16_t numByteToRead);

/**
  * @brief  I2Cx Bus initialization.
  */
static void I2Cx_Init(void)
{
    if(HAL_I2C_GetState(&I2cHandle) == HAL_I2C_STATE_RESET)
    {
        I2cHandle.Instance              = ACCELERO_I2Cx;
        I2cHandle.Init.OwnAddress1      = 0x43;
        I2cHandle.Init.ClockSpeed       = I2Cx_MAX_COMMUNICATION_FREQ;
        I2cHandle.Init.DutyCycle        = I2C_DUTYCYCLE_2;
        I2cHandle.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
        I2cHandle.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLED;
        I2cHandle.Init.OwnAddress2      = 0x00;
        I2cHandle.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLED;
        I2cHandle.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLED;
    }

    // Init the I2C
    I2Cx_MspInit(&I2cHandle);
    if (HAL_I2C_Init(&I2cHandle) != HAL_OK)
    {
        log_error("Failed to initialize I2C\r\n");
    }
}

/**
  * @brief  I2Cx MSP Init.
  * @param  hi2c: I2C handle
  */
static void I2Cx_MspInit(I2C_HandleTypeDef *hi2c)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    // Enable the I2C peripheral
    ACCELERO_I2Cx_CLOCK_ENABLE();

    // Enable SCK and SDA GPIO clocks
    ACCELERO_I2Cx_GPIO_CLK_ENABLE();

    // I2Cx SDA & SCL pin configuration
    GPIO_InitStruct.Pin         = ACCELERO_I2Cx_SCL_PIN | ACCELERO_I2Cx_SDA_PIN;
    GPIO_InitStruct.Mode        = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull        = GPIO_NOPULL;
    GPIO_InitStruct.Speed       = GPIO_SPEED_FAST;
    GPIO_InitStruct.Alternate   = ACCELERO_I2Cx_AF;
    HAL_GPIO_Init(ACCELERO_I2Cx_GPIO_PORT, &GPIO_InitStruct);

    // Force the I2C peripheral clock reset
    ACCELERO_I2Cx_FORCE_RESET();

    // Release the I2C peripheral clock reset
    ACCELERO_I2Cx_RELEASE_RESET();

    // Enable and set I2Cx Interrupt to the lowest priority
    HAL_NVIC_SetPriority(ACCELERO_I2Cx_EV_IRQn, 0x0F, 0);
    HAL_NVIC_EnableIRQ(ACCELERO_I2Cx_EV_IRQn);

    // Enable and set I2Cx Interrupt to the lowest priority
    HAL_NVIC_SetPriority(ACCELERO_I2Cx_ER_IRQn, 0x0F, 0);
    HAL_NVIC_EnableIRQ(ACCELERO_I2Cx_ER_IRQn); 
}

/**
  * @brief  I2Cx error treatment function.
  */
static void I2Cx_Error(void)
{
    /* De-initialize the I2C comunication BUS */
    HAL_I2C_DeInit(&I2cHandle);

    /* Re- Initiaize the I2C comunication BUS */
    I2Cx_Init();
}

/**
  * @brief  Writes a value in a register of the device through BUS.
  * @param  Addr: Device address on BUS Bus.  
  * @param  Reg: The target register address to write
  * @param  Value: The target register value to be written 
  */
static void I2Cx_WriteData(uint16_t addr, uint8_t reg, uint8_t value)
{
    HAL_StatusTypeDef status = HAL_OK;

    status = HAL_I2C_Mem_Write(&I2cHandle, addr, (uint16_t)reg, I2C_MEMADD_SIZE_8BIT, &value, 1, I2cxTimeout);
    if(status != HAL_OK)
    {
        log_error("Failed to write via I2C bus\r\n");
        I2Cx_Error();
    }
}

/**
  * @brief  Reads a register of the device through BUS.
  * @param  Addr: Device address on BUS Bus.  
  * @param  Reg: The target register address to write
  * @retval Data read at register address
  */
static uint8_t I2Cx_ReadData(uint16_t addr, uint8_t reg)
{
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t value = 0;
    
    status = HAL_I2C_Mem_Read(&I2cHandle, addr, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, I2cxTimeout);
    
    /* Check the communication status */
    if(status != HAL_OK)
    {
      /* Execute user timeout callback */
      log_error("Failed to read via I2C bus\r\n");
      I2Cx_Error();
    }

    return value;
}

/* ########################### ACCELEROMETER ########################### */
void LSM303DLHC_IO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    /* Enable DRDY clock */
    ACCELERO_DRDY_GPIO_CLK_ENABLE();

    /* MEMS DRDY pin configuration */
    GPIO_InitStruct.Pin = ACCELERO_DRDY_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
    HAL_GPIO_Init(ACCELERO_DRDY_GPIO_PORT, &GPIO_InitStruct);

    I2Cx_Init();
}

void LSM303DLHC_IO_ITConfig(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    /* Enable INT1 and INT2 GPIO clock */
    ACCELERO_INT_GPIO_CLK_ENABLE();

    /* Configure GPIO PINs to detect Interrupts */
    GPIO_InitStruct.Pin = ACCELERO_INT1_PIN | ACCELERO_INT2_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    HAL_GPIO_Init(ACCELERO_INT_GPIO_PORT, &GPIO_InitStruct);

    /* Enable and set COMPASS / ACCELERO Interrupt to the lowest priority */
    HAL_NVIC_SetPriority(ACCELERO_INT1_EXTI_IRQn, 0x0F, 0x00);
    HAL_NVIC_EnableIRQ(ACCELERO_INT1_EXTI_IRQn);
}

void LSM303DLHC_IO_Write(uint16_t deviceAddr, uint8_t registerAddr, uint8_t value)
{
    I2Cx_WriteData(deviceAddr, registerAddr, value);
}

uint8_t LSM303DLHC_IO_Read(uint16_t deviceAddr, uint8_t registerAddr)
{
    return I2Cx_ReadData(deviceAddr, registerAddr);
}
/* ########################### END ########################### */

/**
  * @brief  SPIx Bus initialization.
  */
static void SPIx_Init(void)
{
    if(HAL_SPI_GetState(&SpiHandle) == HAL_SPI_STATE_RESET)
    {
        // SPI Configuration
        SpiHandle.Instance = GYRO_SPIx;

        /* SPI baudrate is set to 5.6 MHz (PCLK2/SPI_BaudRatePrescaler = 90/16 = 5.625 MHz) 
           to verify these constraints:
           ILI9341 LCD SPI interface max baudrate is 10MHz for write and 6.66MHz for read
           L3GD20 SPI interface max baudrate is 10MHz for write/read
           PCLK2 frequency is set to 90 MHz 
          */
        SpiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
        SpiHandle.Init.Direction = SPI_DIRECTION_2LINES;
        SpiHandle.Init.CLKPhase = SPI_PHASE_1EDGE;
        SpiHandle.Init.CLKPolarity = SPI_POLARITY_LOW;
        SpiHandle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
        SpiHandle.Init.CRCPolynomial = 7;
        SpiHandle.Init.DataSize = SPI_DATASIZE_8BIT;
        SpiHandle.Init.FirstBit = SPI_FIRSTBIT_MSB;
        SpiHandle.Init.NSS = SPI_NSS_SOFT;
        SpiHandle.Init.TIMode = SPI_TIMODE_DISABLED;
        SpiHandle.Init.Mode = SPI_MODE_MASTER;

        SPIx_MspInit(&SpiHandle);
        if(HAL_SPI_Init(&SpiHandle) != HAL_OK)
        {
            log_error("Failed to initialize SPI\r\n");
        }
    }
}

/**
  * @brief  SPI MSP Init.
  * @param  hspi: SPI handle
  */
static void SPIx_MspInit(SPI_HandleTypeDef *hspi)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    // Enable SPIx clock
    GYRO_SPIx_CLOCK_ENABLE();

    // Enable SPIx GPIO clock */
    GYRO_SPIx_GPIO_CLK_ENABLE();

    // Configure SPIx SCK, MOSI and MISO
    GPIO_InitStruct.Pin = (GYRO_SPIx_SCK_PIN | GYRO_SPIx_MOSI_PIN | GYRO_SPIx_MISO_PIN);
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull  = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
    GPIO_InitStruct.Alternate = GYRO_SPIx_AF;
    HAL_GPIO_Init(GYRO_SPIx_GPIO_PORT, &GPIO_InitStruct);
}

/**
  * @brief  SPIx error treatment function.
  */
static void SPIx_Error (void)
{
    /* De-initialize the SPI comunication BUS */
    HAL_SPI_DeInit(&SpiHandle);

    /* Re-Initiaize the SPI comunication BUS */
    SPIx_Init();
}

/**
  * @brief  Sends a Byte through the SPI interface and return the Byte received 
  *         from the SPI bus.
  * @param  Byte: Byte send.
  * @retval The received byte value
  */
static uint8_t SPIx_WriteRead(uint8_t byte)
{
    uint8_t receivedByte = 0;

    /* Send a Byte through the SPI peripheral */
    /* Read byte from the SPI bus */
    if(HAL_SPI_TransmitReceive(&SpiHandle, (uint8_t*) &byte, (uint8_t*) &receivedByte, 1, SpixTimeout) != HAL_OK)
    {
        log_error("Failed to Transmit/Receive via SPI bus\r\n");
        SPIx_Error();
    }

    return receivedByte;
}

/* ########################### GYROSCOPE ########################### */
// Link GYRO functions
void L3GD20_IO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    // Configure the Gyroscope Control Pins
    // Enable CS GPIO clock and Configure GPIO PIN for GYROSCOPE
    GYRO_CS_GPIO_CLK_ENABLE();
    GPIO_InitStruct.Pin = GYRO_CS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
    HAL_GPIO_Init(GYRO_CS_GPIO_PORT, &GPIO_InitStruct);

    /* Deselect : Chip Select high */
    GYRO_CS_HIGH();

    /* Enable INT1, INT2 GPIO clock and Configure GPIO PINs to detect Interrupts */
    GYRO_INT_GPIO_CLK_ENABLE();
    GPIO_InitStruct.Pin = GYRO_INT1_PIN | GYRO_INT2_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
    GPIO_InitStruct.Pull= GPIO_NOPULL;
    HAL_GPIO_Init(GYRO_INT_GPIO_PORT, &GPIO_InitStruct);

    SPIx_Init();
}

void L3GD20_IO_Write(uint8_t *pBuffer, uint8_t writeAddr, uint16_t numByteToWrite)
{
    /**
     * Configure the MS bit:
     * - When 0, the address will remain unchanged in multiple read/write command
     * - When 1, the address will be auto incremented in multiple read/write command
     */
    if(numByteToWrite > 0x01)
    {
        writeAddr |= (uint8_t)MULTIPLEBYTE_CMD;
    }

    // Select chip select low at the start of the transmission
    GYRO_CS_LOW();

    // Send the Address of the indexed register
    SPIx_WriteRead(writeAddr);

    // Send the data that will be written into the device (MSB First)
    while (numByteToWrite >= 0x01)
    {
        SPIx_WriteRead(*pBuffer);
        numByteToWrite--;
        pBuffer++;
    }

    // Set chip select High at the end of the tranmission
    GYRO_CS_HIGH();
}

void L3GD20_IO_Read(uint8_t *pBuffer, uint8_t readAddr, uint16_t numByteToRead)
{
    if(numByteToRead > 0x01)
    {
        readAddr |= (uint8_t)(READWRITE_CMD | MULTIPLEBYTE_CMD);
    }
    else {
        readAddr |= (uint8_t)READWRITE_CMD;
    }

    // Select chip select low at the start of the transmission
    GYRO_CS_LOW();

    // Send the Address of the indexed register
    SPIx_WriteRead(readAddr);

    // Receive the data that will be read from the device (MSB First)
    while (numByeToRead > 0x00)
    {
        // Send dummy byte (0x00) to generate the SPI clock to GYRO(Slave device)
        *pBuffer = SPIx_WriteRead(DUMMY_BYTE);
        numByteToRead--;
        pBuffer++;
    }

    // Set chip select High at the end of the tranmission
    GYRO_CS_HIGH();
}