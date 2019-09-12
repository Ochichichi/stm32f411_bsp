#ifndef __LOG_H
#define __LOG_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

/* Exported macro -------------------------------------------------------------*/
#ifdef NDEBUG
#define log_debug(fmt, ...)     do{} while(0)
#define log_err(fmt, ...)       do{} while(0)
#define log_warn(fmt, ...)       do{} while(0)
#define log_info(fmt, ...)       do{} while(0)
#else // NDEBUG
#define log_debug(fmt, ...) \
        printf("[DEBUG]\t%s\t%d\t"fmt, __FUNCTION__, __LINE__, ##__VA_ARGS__)
#define log_error(fmt, ...) \
        printf("[ERROR]\t%s\t%d\t"fmt, __FUNCTION__, __LINE__, ##__VA_ARGS__)
#define log_warn(fmt, ...) \
        printf("[WARN]\t%s\t%d\t"fmt, __FUNCTION__, __LINE__, ##__VA_ARGS__)
#define log_info(fmt, ...) \
        printf("[INFO]\t%s\t%d\t"fmt, __FUNCTION__, __LINE__, ##__VA_ARGS__)
#endif //NDEBUG

/* Definition for USARTx clock resources */
#define USARTx                           USART2
#define USARTx_CLK_ENABLE()              __HAL_RCC_USART2_CLK_ENABLE();
#define USARTx_RX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()
#define USARTx_TX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE() 

#define USARTx_FORCE_RESET()             __HAL_RCC_USART2_FORCE_RESET()
#define USARTx_RELEASE_RESET()           __HAL_RCC_USART2_RELEASE_RESET()

/* Definition for USARTx Pins */
#define USARTx_TX_PIN                    GPIO_PIN_2
#define USARTx_TX_GPIO_PORT              GPIOA  
#define USARTx_TX_AF                     GPIO_AF7_USART2
#define USARTx_RX_PIN                    GPIO_PIN_3
#define USARTx_RX_GPIO_PORT              GPIOA 
#define USARTx_RX_AF                     GPIO_AF7_USART2

/* Definition for USARTx's NVIC */
#define USARTx_IRQn                      USART2_IRQn
#define USARTx_IRQHandler                USART2_IRQHandler

/* Exported functions ---------------------------------------------*/
void log_init(void);

#endif // __LOG_H