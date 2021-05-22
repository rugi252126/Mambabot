/*
 * uart_if.c
 *
 *  Created on: 22 May 2021
 *      Author: Alfonso, Rudy Manalo
 */

 /* Project includes. */
#include "stm32f4xx_hal.h"


/***** Macros */
/* USART3 GPIO Configuration
   PD8     ------> USART3_TX
   PD9     ------> USART3_RX
*/
#define ROS_USART_TX_PIN                              GPIO_PIN_8 /* PD8---> USART3_TX */
#define ROS_USART_RX_PIN                              GPIO_PIN_9 /* PD9---> USART3_RX */
#define ROS_USART_AF                                  GPIO_AF7_USART3
/* Interrupt handler */
#define ROS_USART_IRQ_HANDLER_ID                      USART3_IRQn

/***** Private Variables */
UART_HandleTypeDef huart3;

/***** External functions */
extern void _Error_Handler(char *, int);

/***** Local function prototypes */

/***** Local functions */


/***** Global functions */

/** USART3 initialization

    @param  none
    @return none

    It will be called once every POR.
 */
void uart_ifF_Init_USART3_UART(void)
{
    huart3.Instance = USART3;
    huart3.Init.BaudRate = 57600;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart3) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }
}

/** Function to enable USART3 interrupt

    @param  none
    @return none
 */
void uart_ifF_EnableInterrupt_USART3(void)
{
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);
}

/** Function to disable USART3 interrupt

    @param  none
    @return none
 */
void uart_ifF_DisableInterrupt_USART3(void)
{
    __HAL_UART_DISABLE_IT(&huart3, UART_IT_RXNE);
}


/**
  * @brief  UART MSP Init.
  * @param  huart pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  if(huart->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspInit 0 */

  /* USER CODE END USART3_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();

    /* Enable GPIO Channels Clock */
    /* GPIO clock is initialized in function "gpio_ifLF_InitGpioClock" */

    GPIO_InitStruct.Pin = ROS_USART_TX_PIN|ROS_USART_RX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = ROS_USART_AF;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* USART3 interrupt Init */
    HAL_NVIC_SetPriority(ROS_USART_IRQ_HANDLER_ID, 0, 0);
    HAL_NVIC_EnableIRQ(ROS_USART_IRQ_HANDLER_ID);
  /* USER CODE BEGIN USART3_MspInit 1 */

  /* USER CODE END USART3_MspInit 1 */
  }
  else
  {
      /* no action. for future use */
  }
}

/**
  * @brief  UART MSP DeInit.
  * @param  huart pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{
    if(huart->Instance==USART3)
    {
    /* USER CODE BEGIN USART3_MspDeInit 0 */

    /* USER CODE END USART3_MspDeInit 0 */
      /* Peripheral clock disable */
      __HAL_RCC_USART3_CLK_DISABLE();

      /* Disable GPIO Channels Clock will be handled in gpio_if module(if needed)
         as the port is shared with other functions.
       */

      /**USART3 GPIO Configuration
      PD8     ------> USART3_TX
      PD9     ------> USART3_RX
      */
      HAL_GPIO_DeInit(GPIOD, ROS_USART_TX_PIN|ROS_USART_RX_PIN);

      /* USART3 interrupt DeInit */
      HAL_NVIC_DisableIRQ(ROS_USART_IRQ_HANDLER_ID);
    /* USER CODE BEGIN USART3_MspDeInit 1 */

    /* USER CODE END USART3_MspDeInit 1 */
    }
    else
    {
        /* no action. for future use */
    }
}
