/*
 * timer_if.c
 *
 *  Created on: 1 May 2021
 *      Author: Alfonso, Rudy Manalo
 */

#include "stm32f4xx_hal.h"
#include "timer_if.h"

/* Private typedef -----------------------------------------------------------*/
/* Timer for Motor PWM Control */
#define TIMx_MOTOR_CTRL                          TIM1
#define TIMx_MOTOR_CTRL_CLK_ENABLE()             __HAL_RCC_TIM1_CLK_ENABLE()

/* Definition for TIMx_MOTOR_CTRL Channel Pins */
//#define TIMx_MOTOR_CTRL_CHANNEL_GPIO_PORT()       __HAL_RCC_GPIOE_CLK_ENABLE();
#define TIMx_MOTOR_CTRL_GPIO_PORT_CHANNEL1        GPIOE
#define TIMx_MOTOR_CTRL_GPIO_PORT_CHANNEL2        GPIOE
#define TIMx_MOTOR_CTRL_GPIO_PORT_CHANNEL3        GPIOE
#define TIMx_MOTOR_CTRL_GPIO_PORT_CHANNEL4        GPIOE
#define TIMx_MOTOR_CTRL_GPIO_PIN_CHANNEL1         GPIO_PIN_9
#define TIMx_MOTOR_CTRL_GPIO_PIN_CHANNEL2         GPIO_PIN_11
#define TIMx_MOTOR_CTRL_GPIO_PIN_CHANNEL3         GPIO_PIN_13
#define TIMx_MOTOR_CTRL_GPIO_PIN_CHANNEL4         GPIO_PIN_14
#define TIMx_MOTOR_CTRL_GPIO_AF_CHANNEL1          GPIO_AF1_TIM1
#define TIMx_MOTOR_CTRL_GPIO_AF_CHANNEL2          GPIO_AF1_TIM1
#define TIMx_MOTOR_CTRL_GPIO_AF_CHANNEL3          GPIO_AF1_TIM1
#define TIMx_MOTOR_CTRL_GPIO_AF_CHANNEL4          GPIO_AF1_TIM1

/* Timers for Motor Encoder */
#define TIMx_MOTOR1_ENCODER                       TIM2
#define TIMx_MOTOR1_ENCODER_CLK_ENABLE()          __HAL_RCC_TIM2_CLK_ENABLE()
#define TIMx_MOTOR2_ENCODER                       TIM3
#define TIMx_MOTOR2_ENCODER_CLK_ENABLE()          __HAL_RCC_TIM3_CLK_ENABLE()
#define TIMx_MOTOR3_ENCODER                       TIM4
#define TIMx_MOTOR3_ENCODER_CLK_ENABLE()          __HAL_RCC_TIM4_CLK_ENABLE()
#define TIMx_MOTOR4_ENCODER                       TIM8
#define TIMx_MOTOR4_ENCODER_CLK_ENABLE()          __HAL_RCC_TIM8_CLK_ENABLE()

/* Definition for Motor Encoder Channel Pins */
#define TIMx_MOTOR1_ENCODER_GPIO_PORT_CHANNEL1    GPIOA
#define TIMx_MOTOR1_ENCODER_GPIO_PORT_CHANNEL2    GPIOB
#define TIMx_MOTOR2_ENCODER_GPIO_PORT_CHANNEL1_2  GPIOB
#define TIMx_MOTOR3_ENCODER_GPIO_PORT_CHANNEL1_2  GPIOD
#define TIMx_MOTOR4_ENCODER_GPIO_PORT_CHANNEL1_2  GPIOC
#define TIMx_MOTOR1_ENCODER_GPIO_PIN_CHANNEL1     GPIO_PIN_15
#define TIMx_MOTOR1_ENCODER_GPIO_PIN_CHANNEL2     GPIO_PIN_3
#define TIMx_MOTOR2_ENCODER_GPIO_PIN_CHANNEL1     GPIO_PIN_4
#define TIMx_MOTOR2_ENCODER_GPIO_PIN_CHANNEL2     GPIO_PIN_5
#define TIMx_MOTOR3_ENCODER_GPIO_PIN_CHANNEL1     GPIO_PIN_12
#define TIMx_MOTOR3_ENCODER_GPIO_PIN_CHANNEL2     GPIO_PIN_13
#define TIMx_MOTOR4_ENCODER_GPIO_PIN_CHANNEL1     GPIO_PIN_6
#define TIMx_MOTOR4_ENCODER_GPIO_PIN_CHANNEL2     GPIO_PIN_7
#define TIMx_MOTOR1_ENCODER_GPIO_AF_CHANNEL1_2    GPIO_AF1_TIM2
#define TIMx_MOTOR2_ENCODER_GPIO_AF_CHANNEL1_2    GPIO_AF2_TIM3
#define TIMx_MOTOR3_ENCODER_GPIO_AF_CHANNEL1_2    GPIO_AF2_TIM4
#define TIMx_MOTOR4_ENCODER_GPIO_AF_CHANNEL1_2    GPIO_AF3_TIM8

#if 0
#define TIMx                           TIM3
#define TIMx_CLK_ENABLE()              __HAL_RCC_TIM3_CLK_ENABLE()


/* Definition for TIMx Channel Pins */
#define TIMx_CHANNEL_GPIO_PORT()       __HAL_RCC_GPIOB_CLK_ENABLE();
#define TIMx_GPIO_PORT_CHANNEL1        GPIOB
#define TIMx_GPIO_PORT_CHANNEL2        GPIOB
#define TIMx_GPIO_PORT_CHANNEL3        GPIOB
#define TIMx_GPIO_PORT_CHANNEL4        GPIOB
#define TIMx_GPIO_PIN_CHANNEL1         GPIO_PIN_4
#define TIMx_GPIO_PIN_CHANNEL2         GPIO_PIN_5
#define TIMx_GPIO_PIN_CHANNEL3         GPIO_PIN_0
#define TIMx_GPIO_PIN_CHANNEL4         GPIO_PIN_1
#define TIMx_GPIO_AF_CHANNEL1          GPIO_AF2_TIM3
#define TIMx_GPIO_AF_CHANNEL2          GPIO_AF2_TIM3
#define TIMx_GPIO_AF_CHANNEL3          GPIO_AF2_TIM3
#define TIMx_GPIO_AF_CHANNEL4          GPIO_AF2_TIM3
#endif

// Period value is derived based on 15Khz TIMx_MOTOR_CTRL output clock
#define  PERIOD_VALUE       (uint32_t)(1000 - 1) //(uint32_t)(666 - 1)  /* Period Value  */
#define  PULSE1_VALUE       (uint32_t)(PERIOD_VALUE/2)        /* Capture Compare 1 Value  */
#define  PULSE2_VALUE       (uint32_t)(PERIOD_VALUE*37.5/100) /* Capture Compare 2 Value  */
#define  PULSE3_VALUE       (uint32_t)(PERIOD_VALUE/4)        /* Capture Compare 3 Value  */
#define  PULSE4_VALUE       (uint32_t)(PERIOD_VALUE*12.5/100) /* Capture Compare 4 Value  */

/* Timer handler declaration */
TIM_HandleTypeDef    TimHandle;
TIM_HandleTypeDef    Encoder1_Handle;
TIM_HandleTypeDef    Encoder2_Handle;
TIM_HandleTypeDef    Encoder3_Handle;
TIM_HandleTypeDef    Encoder4_Handle;

/* Timer Output Compare Configuration Structure declaration */
TIM_OC_InitTypeDef sConfig;

/* Timer Encoder Configuration Structure declaration */
TIM_Encoder_InitTypeDef sEncoder1Config;
TIM_Encoder_InitTypeDef sEncoder2Config;
TIM_Encoder_InitTypeDef sEncoder3Config;
TIM_Encoder_InitTypeDef sEncoder4Config;

/* Counter Prescaler value */
uint32_t uhPrescalerValue = 0;
static uint32_t encoder_counter = 0u;

static void timer_ifLF_InitPwm(void);
static void timer_ifLF_InitEncoder(void);
static void timer_ifLF_ErrorHandler(void);

static void timer_ifLF_InitPwm(void)
{
	  /* Compute the prescaler value to have TIM3 counter clock equal to 15000000 Hz */
	  //uhPrescalerValue = (uint32_t)((SystemCoreClock/2) / 15000000) - 1;
	  uhPrescalerValue = (uint32_t)((SystemCoreClock) / 15000000) - 1;

	  /*##-1- Configure the TIM peripheral #######################################*/
	  /* -----------------------------------------------------------------------
	  TIM3 Configuration: generate 4 PWM signals with 4 different duty cycles.

	    In this example TIM3 input clock (TIM3CLK) is set to APB1 clock x 2,
	    since APB1 prescaler is equal to 2.
	      TIM3CLK = APB1CLK*2
	      APB1CLK = HCLK/2
	      => TIM3CLK = HCLK = SystemCoreClock

	    To get TIM3 counter clock at 15 MHz, the prescaler is computed as follows:
	       Prescaler = (TIM3CLK / TIM3 counter clock) - 1
	       Prescaler = ((SystemCoreClock) /15 MHz) - 1

	    To get TIM3 output clock at 22,52 KHz, the period (ARR)) is computed as follows:
	       ARR = (TIM3 counter clock / TIM3 output clock) - 1
	           = 665

	    TIM3 Channel1 duty cycle = (TIM3_CCR1/ TIM3_ARR + 1)* 100 = 50%
	    TIM3 Channel2 duty cycle = (TIM3_CCR2/ TIM3_ARR + 1)* 100 = 37.5%
	    TIM3 Channel3 duty cycle = (TIM3_CCR3/ TIM3_ARR + 1)* 100 = 25%
	    TIM3 Channel4 duty cycle = (TIM3_CCR4/ TIM3_ARR + 1)* 100 = 12.5%

	    Note:
	     SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f4xx.c file.
	     Each time the core clock (HCLK) changes, user had to update SystemCoreClock
	     variable value. Otherwise, any configuration based on this variable will be incorrect.
	     This variable is updated in three ways:
	      1) by calling CMSIS function SystemCoreClockUpdate()
	      2) by calling HAL API function HAL_RCC_GetSysClockFreq()
	      3) each time HAL_RCC_ClockConfig() is called to configure the system clock frequency
	  ----------------------------------------------------------------------- */

	  /* Motor Pwm Frequency = 15KHz
	     TIM3 counter clock = 15MHz
	     Period = (15MHz / 15KHz) - 1
	     Period = 1000 - 1
	     Period = 999

       Initialize TIMx_MOTOR_CTRL peripheral as follows:
	       + Prescaler = (SystemCoreClock / 15000000) - 1
	       + Period = (1000 - 1)
	       + ClockDivision = 0
	       + Counter direction = Up
	  */
	  TimHandle.Instance = TIMx_MOTOR_CTRL;

	  TimHandle.Init.Prescaler         = uhPrescalerValue;
	  TimHandle.Init.Period            = PERIOD_VALUE;
	  TimHandle.Init.ClockDivision     = 0;
	  TimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
	  TimHandle.Init.RepetitionCounter = 0;
	  //TimHandle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	  if (HAL_TIM_PWM_Init(&TimHandle) != HAL_OK)
	  {
	    /* Initialization Error */
		  timer_ifLF_ErrorHandler();
	  }

	  /*##-2- Configure the PWM channels #########################################*/
	  /* Common configuration for all channels */
	  sConfig.OCMode       = TIM_OCMODE_PWM1;
	  sConfig.OCPolarity   = TIM_OCPOLARITY_HIGH;
	  sConfig.OCFastMode   = TIM_OCFAST_DISABLE;
	  sConfig.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
	  sConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;

	  sConfig.OCIdleState  = TIM_OCIDLESTATE_RESET;

#if 0
	  /* Set the pulse value for channel 1 */
	  sConfig.Pulse = PULSE1_VALUE;
	  if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_1) != HAL_OK)
	  {
	    /* Configuration Error */
		  timer_ifLF_ErrorHandler();
	  }

	  /* Set the pulse value for channel 2 */
	  sConfig.Pulse = PULSE2_VALUE;
	  if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_2) != HAL_OK)
	  {
	    /* Configuration Error */
		  timer_ifLF_ErrorHandler();
	  }

	  /* Set the pulse value for channel 3 */
	  sConfig.Pulse = PULSE3_VALUE;
	  if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_3) != HAL_OK)
	  {
	    /* Configuration Error */
		  timer_ifLF_ErrorHandler();
	  }

	  /* Set the pulse value for channel 4 */
	  sConfig.Pulse = PULSE4_VALUE;
	  if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_4) != HAL_OK)
	  {
	    /* Configuration Error */
		  timer_ifLF_ErrorHandler();
	  }
#else
	  timer_ifF_ConfigPwmChannel(TIM_PWM_ID_1_E);
	  timer_ifF_ConfigPwmChannel(TIM_PWM_ID_2_E);
	  timer_ifF_ConfigPwmChannel(TIM_PWM_ID_3_E);
	  timer_ifF_ConfigPwmChannel(TIM_PWM_ID_4_E);
#endif

#if 0
	  /*##-3- Start PWM signals generation #######################################*/
	  /* Start channel 1 */
	  if (HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_1) != HAL_OK)
	  {
	    /* PWM Generation Error */
		  timer_ifLF_ErrorHandler();
	  }
	  /* Start channel 2 */
	  if (HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_2) != HAL_OK)
	  {
	    /* PWM Generation Error */
		  timer_ifLF_ErrorHandler();
	  }
	  /* Start channel 3 */
	  if (HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_3) != HAL_OK)
	  {
	    /* PWM generation Error */
		  timer_ifLF_ErrorHandler();
	  }
	  /* Start channel 4 */
	  if (HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_4) != HAL_OK)
	  {
	    /* PWM generation Error */
		  timer_ifLF_ErrorHandler();
	  }
#else
	  timer_ifF_StartPwm(TIM_PWM_ID_1_E);
	  timer_ifF_StartPwm(TIM_PWM_ID_2_E);
	  timer_ifF_StartPwm(TIM_PWM_ID_3_E);
	  timer_ifF_StartPwm(TIM_PWM_ID_4_E);
#endif
}


static void timer_ifLF_InitEncoder(void)
{
    /* -1- Initialize TIM1 to handle the encoder sensor */
    /* Initialize TIM1 peripheral as follow:
	       + Period = 65535
	       + Prescaler = 0
	       + ClockDivision = 0
	       + Counter direction = Up
    */
    Encoder1_Handle.Instance = TIMx_MOTOR1_ENCODER;

    Encoder1_Handle.Init.Period            = 65535;
    Encoder1_Handle.Init.Prescaler         = 0;
    Encoder1_Handle.Init.ClockDivision     = 0;
    Encoder1_Handle.Init.CounterMode       = TIM_COUNTERMODE_UP;
    Encoder1_Handle.Init.RepetitionCounter = 0;
    //Encoder1_Handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    sEncoder1Config.EncoderMode        = TIM_ENCODERMODE_TI12;

    sEncoder1Config.IC1Polarity        = TIM_ICPOLARITY_RISING;
    sEncoder1Config.IC1Selection       = TIM_ICSELECTION_DIRECTTI;
    sEncoder1Config.IC1Prescaler       = TIM_ICPSC_DIV1;
    sEncoder1Config.IC1Filter          = 0;

    sEncoder1Config.IC2Polarity        = TIM_ICPOLARITY_RISING;
    sEncoder1Config.IC2Selection       = TIM_ICSELECTION_DIRECTTI;
    sEncoder1Config.IC2Prescaler       = TIM_ICPSC_DIV1;
    sEncoder1Config.IC2Filter          = 0;

    if(HAL_TIM_Encoder_Init(&Encoder1_Handle, &sEncoder1Config) != HAL_OK)
    {
        /* Initialization Error */
    	timer_ifLF_ErrorHandler();
    }


    Encoder2_Handle.Instance = TIMx_MOTOR2_ENCODER;

    Encoder2_Handle.Init.Period            = 65535;
    Encoder2_Handle.Init.Prescaler         = 0;
    Encoder2_Handle.Init.ClockDivision     = 0;
    Encoder2_Handle.Init.CounterMode       = TIM_COUNTERMODE_UP;
    Encoder2_Handle.Init.RepetitionCounter = 0;
    //Encoder2_Handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    sEncoder2Config.EncoderMode        = TIM_ENCODERMODE_TI12;

    sEncoder2Config.IC1Polarity        = TIM_ICPOLARITY_RISING;
    sEncoder2Config.IC1Selection       = TIM_ICSELECTION_DIRECTTI;
    sEncoder2Config.IC1Prescaler       = TIM_ICPSC_DIV1;
    sEncoder2Config.IC1Filter          = 0;

    sEncoder2Config.IC2Polarity        = TIM_ICPOLARITY_RISING;
    sEncoder2Config.IC2Selection       = TIM_ICSELECTION_DIRECTTI;
    sEncoder2Config.IC2Prescaler       = TIM_ICPSC_DIV1;
    sEncoder2Config.IC2Filter          = 0;

    if(HAL_TIM_Encoder_Init(&Encoder2_Handle, &sEncoder2Config) != HAL_OK)
    {
        /* Initialization Error */
    	timer_ifLF_ErrorHandler();
    }


    Encoder3_Handle.Instance = TIMx_MOTOR3_ENCODER;

    Encoder3_Handle.Init.Period            = 65535;
    Encoder3_Handle.Init.Prescaler         = 0;
    Encoder3_Handle.Init.ClockDivision     = 0;
    Encoder3_Handle.Init.CounterMode       = TIM_COUNTERMODE_UP;
    Encoder3_Handle.Init.RepetitionCounter = 0;
    //Encoder3_Handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    sEncoder3Config.EncoderMode        = TIM_ENCODERMODE_TI12;

    sEncoder3Config.IC1Polarity        = TIM_ICPOLARITY_RISING;
    sEncoder3Config.IC1Selection       = TIM_ICSELECTION_DIRECTTI;
    sEncoder3Config.IC1Prescaler       = TIM_ICPSC_DIV1;
    sEncoder3Config.IC1Filter          = 0;

    sEncoder3Config.IC2Polarity        = TIM_ICPOLARITY_RISING;
    sEncoder3Config.IC2Selection       = TIM_ICSELECTION_DIRECTTI;
    sEncoder3Config.IC2Prescaler       = TIM_ICPSC_DIV1;
    sEncoder3Config.IC2Filter          = 0;

    if(HAL_TIM_Encoder_Init(&Encoder3_Handle, &sEncoder3Config) != HAL_OK)
    {
        /* Initialization Error */
    	timer_ifLF_ErrorHandler();
    }


    Encoder4_Handle.Instance = TIMx_MOTOR4_ENCODER;

    Encoder4_Handle.Init.Period            = 65535;
    Encoder4_Handle.Init.Prescaler         = 0;
    Encoder4_Handle.Init.ClockDivision     = 0;
    Encoder4_Handle.Init.CounterMode       = TIM_COUNTERMODE_UP;
    Encoder4_Handle.Init.RepetitionCounter = 0;
    //Encoder4_Handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    sEncoder4Config.EncoderMode        = TIM_ENCODERMODE_TI12;

    sEncoder4Config.IC1Polarity        = TIM_ICPOLARITY_RISING;
    sEncoder4Config.IC1Selection       = TIM_ICSELECTION_DIRECTTI;
    sEncoder4Config.IC1Prescaler       = TIM_ICPSC_DIV1;
    sEncoder4Config.IC1Filter          = 0;

    sEncoder4Config.IC2Polarity        = TIM_ICPOLARITY_RISING;
    sEncoder4Config.IC2Selection       = TIM_ICSELECTION_DIRECTTI;
    sEncoder4Config.IC2Prescaler       = TIM_ICPSC_DIV1;
    sEncoder4Config.IC2Filter          = 0;

    if(HAL_TIM_Encoder_Init(&Encoder4_Handle, &sEncoder4Config) != HAL_OK)
    {
        /* Initialization Error */
    	timer_ifLF_ErrorHandler();
    }
    /* Start the encoder interface */
    /*HAL_TIM_Encoder_Start_IT*/HAL_TIM_Encoder_Start(&Encoder1_Handle, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&Encoder2_Handle, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&Encoder3_Handle, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&Encoder4_Handle, TIM_CHANNEL_ALL);
}


void timer_ifF_Init(void)
{
    timer_ifLF_InitPwm();

    timer_ifLF_InitEncoder();
}

static void timer_ifLF_ErrorHandler(void)
{
	//HAL_GPIO_WritePin(LD2_GPIO_Port, GPIO_PIN_0, GPIO_PIN_SET);
}

/**
  * @brief TIM MSP Initialization
  *        This function configures the hardware resources used in this example:
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration
  * @param htim: TIM handle pointer
  * @retval None
  */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
  GPIO_InitTypeDef   GPIO_InitStruct;
  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* TIMx_MOTOR_CTRL Peripheral clock enable */
  TIMx_MOTOR_CTRL_CLK_ENABLE();

  /* Enable all GPIO Channels Clock requested */
  /* GPIO clock is initialize in function "mainLF_InitGpioClock" */
  //TIMx_MOTOR_CTRL_CHANNEL_GPIO_PORT();

  /* Configure (TIM1_Channel1), (TIM1_Channel2), (TIM1_Channel3),
     (TIM1_Channel4) in output, push-pull, alternate function mode
  */
  /* Common configuration for all channels */
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

  GPIO_InitStruct.Alternate = TIMx_MOTOR_CTRL_GPIO_AF_CHANNEL1;
  GPIO_InitStruct.Pin = TIMx_MOTOR_CTRL_GPIO_PIN_CHANNEL1;
  HAL_GPIO_Init(TIMx_MOTOR_CTRL_GPIO_PORT_CHANNEL1, &GPIO_InitStruct);

  GPIO_InitStruct.Alternate = TIMx_MOTOR_CTRL_GPIO_AF_CHANNEL2;
  GPIO_InitStruct.Pin = TIMx_MOTOR_CTRL_GPIO_PIN_CHANNEL2;
  HAL_GPIO_Init(TIMx_MOTOR_CTRL_GPIO_PORT_CHANNEL2, &GPIO_InitStruct);

  GPIO_InitStruct.Alternate = TIMx_MOTOR_CTRL_GPIO_AF_CHANNEL3;
  GPIO_InitStruct.Pin = TIMx_MOTOR_CTRL_GPIO_PIN_CHANNEL3;
  HAL_GPIO_Init(TIMx_MOTOR_CTRL_GPIO_PORT_CHANNEL3, &GPIO_InitStruct);

  GPIO_InitStruct.Alternate = TIMx_MOTOR_CTRL_GPIO_AF_CHANNEL4;
  GPIO_InitStruct.Pin = TIMx_MOTOR_CTRL_GPIO_PIN_CHANNEL4;
  HAL_GPIO_Init(TIMx_MOTOR_CTRL_GPIO_PORT_CHANNEL4, &GPIO_InitStruct);
}

/**
  * @brief TIM MSP Initialization
  *        This function configures the hardware resources used in this example:
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration
  * @param htim: TIM handle pointer
  * @retval None
  */
void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef *htim)
{
    GPIO_InitTypeDef   GPIO_InitStruct;

  /*##-1- Enable peripherals and GPIO Clocks #################################*/
    /* TIM2 Peripheral clock enable */
    TIMx_MOTOR1_ENCODER_CLK_ENABLE();
    /* TIM3 Peripheral clock enable */
    TIMx_MOTOR2_ENCODER_CLK_ENABLE();
    /* TIM4 Peripheral clock enable */
    TIMx_MOTOR3_ENCODER_CLK_ENABLE();
    /* TIM5 Peripheral clock enable */
    TIMx_MOTOR4_ENCODER_CLK_ENABLE();

  /* Enable GPIO Channels Clock */
  /* GPIO clock is initialize in function "mainLF_InitGpioClock" */

  /*##-2- Configure I/Os #####################################################*/

    /* Configuration for Motor1 encoder */
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = TIMx_MOTOR1_ENCODER_GPIO_AF_CHANNEL1_2;

    /* Motor1 encoder Channel 1 configuration */
    GPIO_InitStruct.Pin = TIMx_MOTOR1_ENCODER_GPIO_PIN_CHANNEL1;
    HAL_GPIO_Init(TIMx_MOTOR1_ENCODER_GPIO_PORT_CHANNEL1, &GPIO_InitStruct);

    /* Motor1 encoder Channel 2 configuration */
    GPIO_InitStruct.Pin = TIMx_MOTOR1_ENCODER_GPIO_PIN_CHANNEL2;
    HAL_GPIO_Init(TIMx_MOTOR1_ENCODER_GPIO_PORT_CHANNEL2, &GPIO_InitStruct);

    /* Configuration for Motor2 encoder */
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = TIMx_MOTOR2_ENCODER_GPIO_AF_CHANNEL1_2;

    /* Motor2 encoder Channel 1 configuration */
    GPIO_InitStruct.Pin = TIMx_MOTOR2_ENCODER_GPIO_PIN_CHANNEL1;
    HAL_GPIO_Init(TIMx_MOTOR2_ENCODER_GPIO_PORT_CHANNEL1_2, &GPIO_InitStruct);

    /* Motor2 encoder Channel 2 configuration */
    GPIO_InitStruct.Pin = TIMx_MOTOR2_ENCODER_GPIO_PIN_CHANNEL2;
    HAL_GPIO_Init(TIMx_MOTOR2_ENCODER_GPIO_PORT_CHANNEL1_2, &GPIO_InitStruct);

    /* Configuration for Motor3 encoder */
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = TIMx_MOTOR3_ENCODER_GPIO_AF_CHANNEL1_2;

    /* Motor3 encoder Channel 1 configuration */
    GPIO_InitStruct.Pin = TIMx_MOTOR3_ENCODER_GPIO_PIN_CHANNEL1;
    HAL_GPIO_Init(TIMx_MOTOR3_ENCODER_GPIO_PORT_CHANNEL1_2, &GPIO_InitStruct);

    /* Motor3 encoder Channel 2 configuration */
    GPIO_InitStruct.Pin = TIMx_MOTOR3_ENCODER_GPIO_PIN_CHANNEL2;
    HAL_GPIO_Init(TIMx_MOTOR3_ENCODER_GPIO_PORT_CHANNEL1_2, &GPIO_InitStruct);

    /* Configuration for Motor4 encoder */
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = TIMx_MOTOR4_ENCODER_GPIO_AF_CHANNEL1_2;

    /* Motor4 encoder Channel 1 configuration */
    GPIO_InitStruct.Pin = TIMx_MOTOR4_ENCODER_GPIO_PIN_CHANNEL1;
    HAL_GPIO_Init(TIMx_MOTOR4_ENCODER_GPIO_PORT_CHANNEL1_2, &GPIO_InitStruct);

    /* Motor4 encoder Channel 2 configuration */
    GPIO_InitStruct.Pin = TIMx_MOTOR4_ENCODER_GPIO_PIN_CHANNEL2;
    HAL_GPIO_Init(TIMx_MOTOR4_ENCODER_GPIO_PORT_CHANNEL1_2, &GPIO_InitStruct);
}

void timer_ifF_ConfigPwmChannel(enum tim_pwm_id_et tim_id_e)
{
	switch(tim_id_e)
	{
	    case TIM_PWM_ID_1_E:
	    {
	        /* Set the duty cycle for channel 1 */
	    	sConfig.Pulse = 0u;
	    	if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_1) != HAL_OK)
	    	{
	    	    /* Configuration Error */
	    		timer_ifLF_ErrorHandler();
	    	}
		    break;
	    }

	    case TIM_PWM_ID_2_E:
	    {
	        /* Set the duty cycle for channel 2 */
	    	sConfig.Pulse = 0u;
	    	if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_2) != HAL_OK)
	    	{
	    	    /* Configuration Error */
	    		timer_ifLF_ErrorHandler();
	    	}
	    	break;
	    }

	    case TIM_PWM_ID_3_E:
	    {
	        /* Set the duty cycle for channel 3 */
	    	sConfig.Pulse = 0u;
	    	if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_3) != HAL_OK)
	    	{
	    	    /* Configuration Error */
	    		timer_ifLF_ErrorHandler();
	    	}
	    	break;
	    }

	    case TIM_PWM_ID_4_E:
	    {
	        /* Set the duty cycle for channel 4 */
	    	sConfig.Pulse = 0u;
	    	if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_4) != HAL_OK)
	    	{
	    	    /* Configuration Error */
	    		timer_ifLF_ErrorHandler();
	    	}
	    	break;
	    }

	    default:
	    	/* no action */
	    	break;
	}
}

void timer_ifF_StartPwm(enum tim_pwm_id_et tim_id_e)
{
	switch(tim_id_e)
	{
	    case TIM_PWM_ID_1_E:
	    {
	    	/* Start PWM signals generation for channel 1 */
	    	if (HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_1) != HAL_OK)
	    	{
	    	    /* Configuration Error */
	    		timer_ifLF_ErrorHandler();
	    	}
		    break;
	    }

	    case TIM_PWM_ID_2_E:
	    {
	    	/* Start PWM signals generation for channel 2 */
	    	if (HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_2) != HAL_OK)
	    	{
	    	    /* Configuration Error */
	    		timer_ifLF_ErrorHandler();
	    	}
	    	break;
	    }

	    case TIM_PWM_ID_3_E:
	    {
	    	/* Start PWM signals generation for channel 3 */
	    	if (HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_3) != HAL_OK)
	    	{
	    	    /* Configuration Error */
	    		timer_ifLF_ErrorHandler();
	    	}
	    	break;
	    }

	    case TIM_PWM_ID_4_E:
	    {
	    	/* Start PWM signals generation for channel 4 */
	    	if (HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_4) != HAL_OK)
	    	{
	    	    /* Configuration Error */
	    		timer_ifLF_ErrorHandler();
	    	}
	    	break;
	    }

	    default:
	    	/* no action */
	    	break;
	}
}

void timer_ifF_StopPwm(enum tim_pwm_id_et tim_id_e)
{
	switch(tim_id_e)
	{
	    case TIM_PWM_ID_1_E:
	    {
	    	/* Stop PWM signals generation for channel 1 */
	    	if (HAL_TIM_PWM_Stop(&TimHandle, TIM_CHANNEL_1) != HAL_OK)
	    	{
	    	    /* Configuration Error */
	    		timer_ifLF_ErrorHandler();
	    	}
		    break;
	    }

	    case TIM_PWM_ID_2_E:
	    {
	    	/* Stop PWM signals generation for channel 2 */
	    	if (HAL_TIM_PWM_Stop(&TimHandle, TIM_CHANNEL_2) != HAL_OK)
	    	{
	    	    /* Configuration Error */
	    		timer_ifLF_ErrorHandler();
	    	}
	    	break;
	    }

	    case TIM_PWM_ID_3_E:
	    {
	    	/* Stop PWM signals generation for channel 3 */
	    	if (HAL_TIM_PWM_Stop(&TimHandle, TIM_CHANNEL_3) != HAL_OK)
	    	{
	    	    /* Configuration Error */
	    		timer_ifLF_ErrorHandler();
	    	}
	    	break;
	    }

	    case TIM_PWM_ID_4_E:
	    {
	    	/* Stop PWM signals generation for channel 4 */
	    	if (HAL_TIM_PWM_Stop(&TimHandle, TIM_CHANNEL_4) != HAL_OK)
	    	{
	    	    /* Configuration Error */
	    		timer_ifLF_ErrorHandler();
	    	}
	    	break;
	    }

	    default:
	    	/* no action */
	    	break;
	}
}

void timer_ifF_UpdatePwm(uint32_t duty_cycle_percentage, enum tim_pwm_id_et tim_id_e)
{
	uint32_t duty_cycle;

	// Get the equivalent duty cycle value based on the received pwm percentage
	duty_cycle = (((PERIOD_VALUE*10) / 100) * duty_cycle_percentage) / 10;

	switch(tim_id_e)
	{
	    case TIM_PWM_ID_1_E:
	    {
	    	TIMx_MOTOR_CTRL->CCR1 = duty_cycle;
			break;
	    }

	    case TIM_PWM_ID_2_E:
	    {
	    	TIMx_MOTOR_CTRL->CCR2 = duty_cycle;
		    break;
	    }

	    case TIM_PWM_ID_3_E:
	    {
	    	TIMx_MOTOR_CTRL->CCR3 = duty_cycle;
		    break;
	    }

	    case TIM_PWM_ID_4_E:
	    {
	    	TIMx_MOTOR_CTRL->CCR4 = duty_cycle;
		    break;
	    }

	    default:
	    	/* no action */
	    	break;
	}
}

uint32_t timer_ifF_getEncoderCount(void)
{
	//uint32_t uwDirection = 0u;

	//uwDirection = __HAL_TIM_IS_TIM_COUNTING_DOWN(&Encoder1_Handle);

	//return (uwDirection);

	//encoder_counter = TIMx_MOTOR1_ENCODER->CNT;//
	return (encoder_counter);
}

uint32_t timer_ifF_getMotorDirection(void)
{
	uint32_t uwDirection = 0u;

	// Counting-up = 0
	// Counting down = 1
	uwDirection = __HAL_TIM_IS_TIM_COUNTING_DOWN(&Encoder4_Handle);

	return (uwDirection);
}

void timer_ifF_ReadEncoderCounts(void)
{
    encoder_counter = TIMx_MOTOR4_ENCODER->CNT;//__HAL_TIM_GET_COUNTER(&Encoder1_Handle);
}
