/*
 * log.c
 *
 *  Created on: Sep 10, 2019
 *      Author: HiepNguyen
 */
/* Includes ------------------------------------------------------------------*/
#include "log.h"

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* External functions --------------------------------------------------------*/
/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
void log_init(void)
{
    /* Uart2 parameter configuration */
    huart2.Instance = USARTx;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK)
    {

    }
}

/**
  * @brief Rewrite printf function
  * @param None
  * @retval None
  */
int _write(int fd, char * str, int len)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)str, len , 100);
    return len;
}
