#ifndef __STM32_IMU_CONFIGURATION_H
#define __STM32_IMU_CONFIGURATION_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

// ACCELEROMETER
// ACCELERO I2C1 Interface pins
#define ACCELERO_DRDY_GPIO_PORT                 GPIOE                       /* GPIOE */
#define ACCELERO_DRDY_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOE_CLK_ENABLE() 
#define ACCELERO_DRDY_GPIO_CLK_DISABLE()        __HAL_RCC_GPIOE_CLK_DISABLE() 
#define ACCELERO_DRDY_PIN                       GPIO_PIN_2                  /* PE.02 */
#define ACCELERO_DRDY_EXTI_IRQn                 TAMP_STAMP_IRQn

#define ACCELERO_INT_GPIO_PORT                  GPIOE                       /* GPIOE */
#define ACCELERO_INT_GPIO_CLK_ENABLE()          __HAL_RCC_GPIOE_CLK_ENABLE()
#define ACCELERO_INT_GPIO_CLK_DISABLE()         __HAL_RCC_GPIOE_CLK_DISABLE()
#define ACCELERO_INT1_PIN                       GPIO_PIN_4                  /* PE.04 */
#define ACCELERO_INT1_EXTI_IRQn                 EXTI4_IRQn 
#define ACCELERO_INT2_PIN                       GPIO_PIN_5                  /* PE.05 */
#define ACCELERO_INT2_EXTI_IRQn                 EXTI9_5_IRQn 

// I2C bus
#define ACCELERO_I2Cx                          I2C1
#define ACCELERO_I2Cx_CLOCK_ENABLE()           __HAL_RCC_I2C1_CLK_ENABLE()
#define ACCELERO_I2Cx_GPIO_PORT                GPIOB                       /* GPIOB */
#define ACCELERO_I2Cx_SCL_PIN                  GPIO_PIN_6                  /* PB.06 */
#define ACCELERO_I2Cx_SDA_PIN                  GPIO_PIN_9                  /* PB.09 */
#define ACCELERO_I2Cx_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOB_CLK_ENABLE() 
#define ACCELERO_I2Cx_GPIO_CLK_DISABLE()       __HAL_RCC_GPIOB_CLK_DISABLE() 
#define ACCELERO_I2Cx_AF                       GPIO_AF4_I2C1

#define ACCELERO_I2Cx_FORCE_RESET()            __HAL_RCC_I2C1_FORCE_RESET()
#define ACCELERO_I2Cx_RELEASE_RESET()          __HAL_RCC_I2C1_RELEASE_RESET()

/* I2C interrupt requests */
#define ACCELERO_I2Cx_EV_IRQn                  I2C1_EV_IRQn
#define ACCELERO_I2Cx_ER_IRQn                  I2C1_ER_IRQn

/* I2C speed and timeout max */
#define I2Cx_TIMEOUT_MAX                        0xA000 /*<! The value of the maximal timeout for I2C waiting loops */
#define I2Cx_MAX_COMMUNICATION_FREQ             ((uint32_t) 100000)

// GYROSCOPE
// SPI bus
#define GYRO_SPIx                          SPI1
#define GYRO_SPIx_CLOCK_ENABLE()           __HAL_RCC_SPI1_CLK_ENABLE()
#define GYRO_SPIx_GPIO_PORT                GPIOA                      /* GPIOA */
#define GYRO_SPIx_AF                       GPIO_AF5_SPI1
#define GYRO_SPIx_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOA_CLK_ENABLE()
#define GYRO_SPIx_GPIO_CLK_DISABLE()       __HAL_RCC_GPIOA_CLK_DISABLE()
#define GYRO_SPIx_SCK_PIN                  GPIO_PIN_5                 /* PA.05 */
#define GYRO_SPIx_MISO_PIN                 GPIO_PIN_6                 /* PA.06 */
#define GYRO_SPIx_MOSI_PIN                 GPIO_PIN_7                 /* PA.07 */

// SPI timeout max
#define SPIx_TIMEOUT_MAX                ((uint32_t)0x1000)

// Read/Write command
#define READWRITE_CMD                   ((uint8_t)0x80)
// Multiple byte read/write command
#define MULTIPLEBYTE_CMD                ((uint8_t)0x40)
// Dummy byte
#define DUMMY_BYTE                      ((uint8_t)0x00)

// Chip Select marco definition
#define GYRO_CS_LOW()   HAL_GPIO_WritePin(GYRO_CS_GPIO_PORT, GYRO_CS_PIN, GPIO_PIN_RESET)
#define GYRO_CS_HIGH()  HAL_GPIO_WritePin(GYRO_CS_GPIO_PORT, GYRO_CS_PIN, GPIO_PIN_SET)

// GYRO SPI interface pins
#define GYRO_CS_GPIO_PORT                       GPIOE                       /* GPIOE */
#define GYRO_CS_GPIO_CLK_ENABLE()               __HAL_RCC_GPIOE_CLK_ENABLE()
#define GYRO_CS_GPIO_CLK_DISABLE()              __HAL_RCC_GPIOE_CLK_DISABLE()
#define GYRO_CS_PIN                             GPIO_PIN_3                  /* PE.03 */

#define GYRO_INT_GPIO_PORT                      GPIOE                       /* GPIOE */
#define GYRO_INT_GPIO_CLK_ENABLE()              __HAL_RCC_GPIOE_CLK_ENABLE()
#define GYRO_INT_GPIO_CLK_DISABLE()             __HAL_RCC_GPIOE_CLK_DISABLE()
#define GYRO_INT1_PIN                           GPIO_PIN_0                  /* PE.00 */
#define GYRO_INT1_EXTI_IRQn                     EXTI0_IRQn 
#define GYRO_INT2_PIN                           GPIO_PIN_1                  /* PE.01 */
#define GYRO_INT2_EXTI_IRQn                     EXTI1_IRQn 

#endif // __STM32_IMU_CONFIGURATION_H
