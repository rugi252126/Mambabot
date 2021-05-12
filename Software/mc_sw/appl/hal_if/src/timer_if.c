/*
 * timer_if.c
 *
 *  Created on: 1 May 2021
 *      Author: Alfonso, Rudy Manalo
 */

#include "stm32f4xx_hal.h"
#include "project.h"
#include "timer_if.h"
#include "encoder.h"
//#include "motor.h"
#include "math.h"

/* Private typedef -----------------------------------------------------------*/
#define TIM_PWM_ID_1_K                            MOTOR_ID_1_K /* for Motor1 Pwm Control     */
#define TIM_PWM_ID_2_K                            MOTOR_ID_2_K /* for Motor2 Pwm Control     */
#define TIM_PWM_ID_3_K                            MOTOR_ID_3_K /* for Motor3 Pwm Control     */
#define TIM_PWM_ID_4_K                            MOTOR_ID_4_K /* for Motor4 Pwm Control     */
#define TIM_PWM_ID_5_K                            (uint8_t)(4) /* for Head-light Pwm Control */

/* Timer for Motor PWM Control */
#define TIMx_MOTOR_CTRL                           TIM1
#define TIMx_MOTOR_CTRL_CLK_ENABLE()              __HAL_RCC_TIM1_CLK_ENABLE()

/* Definition for TIMx_MOTOR_CTRL Channel Pins */
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

/* Free running timer */
#define TIMx_FREE_RUNNING_TIMER                   TIM7
#define TIMx_FREE_RUNNING_TIMER_CLK_ENABLE()      __HAL_RCC_TIM7_CLK_ENABLE()

/* Period value is derived based on 15Khz TIMx_MOTOR_CTRL output clock */
#define PERIOD_VALUE                             (uint32_t)(1000 - 1)    /* Period Value            */
#define PWM_RESOLUTION                           (uint32_t)(10)          /* Resolution              */
#define PWM_PERCENT_TO_DUTY_CYCLE                (uint32_t)(100)         /* Raw to Percent(%) value */

/* Period value is derived based on 1Khz TIMx_FREE_RUNNING_TIMER output clock */
#define FREE_RUNNING_PERIOD_VALUE                (uint32_t)(1000 - 1)    /* Period Value            */

/* Timer handler declaration */
TIM_HandleTypeDef    TimMotorHandle;
TIM_HandleTypeDef    TimFreeRunningHandle;
TIM_HandleTypeDef    Encoder1_Handle;
TIM_HandleTypeDef    Encoder2_Handle;
TIM_HandleTypeDef    Encoder3_Handle;
TIM_HandleTypeDef    Encoder4_Handle;

/* Timer Output Compare Configuration Structure declaration */
TIM_OC_InitTypeDef sMotorConfig;

/* Timer Encoder Configuration Structure declaration */
TIM_Encoder_InitTypeDef sEncoder1Config;
TIM_Encoder_InitTypeDef sEncoder2Config;
TIM_Encoder_InitTypeDef sEncoder3Config;
TIM_Encoder_InitTypeDef sEncoder4Config;

/***** Local function prototypes */
static void timer_ifLF_InitFreeRunningTimer(void);
static void timer_ifLF_InitPwm(void);
static void timer_ifLF_InitEncoder(void);
static void timer_ifLF_ErrorHandler(void);

/***** Local functions */

/** Initialization of free running timer

    @param  none
    @return none
 */
static void timer_ifLF_InitFreeRunningTimer(void)
{
    /* Counter Prescaler value */
    uint32_t tmp_prescalerValue = 0;

    /* Compute the prescaler value to have TIMx_FREE_RUNNING_TIMER counter clock equal to 1000000 Hz */
    /* SystemCoreClock = 180MHz */
    tmp_prescalerValue = (uint32_t)((SystemCoreClock/2) / 1000000) - 1;

    /* Free Running Timer Frequency = 1KHz
       TIMx_FREE_RUNNING_TIMER counter clock = 1MHz
       Period = (1MHz / 1KHz) - 1
       Period = 1000 - 1
       Period = 999
    */
    TimFreeRunningHandle.Instance = TIMx_FREE_RUNNING_TIMER;

    TimFreeRunningHandle.Init.Prescaler         = tmp_prescalerValue;
    TimFreeRunningHandle.Init.Period            = FREE_RUNNING_PERIOD_VALUE;
    TimFreeRunningHandle.Init.ClockDivision     = 0;
    TimFreeRunningHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
    TimFreeRunningHandle.Init.RepetitionCounter = 0;

    if (HAL_TIM_Base_Init(&TimFreeRunningHandle) != HAL_OK)
    {
        /* Initialization Error */
        timer_ifLF_ErrorHandler();
    }

    /* Start timer with interrupt capability */
    if (HAL_TIM_Base_Start_IT(&TimFreeRunningHandle) != HAL_OK)
    {
        /* Initialization Error */
        timer_ifLF_ErrorHandler();
    }
}

/** Initialization of timer used for Pwm generation

    @param  none
    @return none
 */
static void timer_ifLF_InitPwm(void)
{
    /* Counter Prescaler value */
    uint32_t tmp_prescalerValue = 0;

	/* Compute the prescaler value to have TIMx_MOTOR_CTRL counter clock equal to 15000000 Hz */
    /* SystemCoreClock = 180MHz */
    tmp_prescalerValue = (uint32_t)((SystemCoreClock) / 15000000) - 1;

	/*##-1- Configure the TIM peripheral #######################################*/
	/* -----------------------------------------------------------------------
	TIM1 Configuration: generate 4 PWM signals

	Motor Pwm Frequency = 15KHz
	   TIMx_MOTOR_CTRL counter clock = 15MHz
	   Period = (15MHz / 15KHz) - 1
	   Period = 1000 - 1
	   Period = 999

       Initialize TIMx_MOTOR_CTRL peripheral as follows:
	       + Prescaler = (SystemCoreClock / 15000000) - 1
	       + Period = (1000 - 1)
	       + ClockDivision = 0
	       + Counter direction = Up
	*/
    TimMotorHandle.Instance = TIMx_MOTOR_CTRL;

    TimMotorHandle.Init.Prescaler         = tmp_prescalerValue;
    TimMotorHandle.Init.Period            = PERIOD_VALUE;
    TimMotorHandle.Init.ClockDivision     = 0;
    TimMotorHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
    TimMotorHandle.Init.RepetitionCounter = 0;
    //TimMotorHandle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&TimMotorHandle) != HAL_OK)
    {
        /* Initialization Error */
        timer_ifLF_ErrorHandler();
    }

    /*##-2- Configure the PWM channels #########################################*/
    /* Common configuration for all channels */
    sMotorConfig.OCMode       = TIM_OCMODE_PWM1;
    sMotorConfig.OCPolarity   = TIM_OCPOLARITY_HIGH;
    sMotorConfig.OCFastMode   = TIM_OCFAST_DISABLE;
    sMotorConfig.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
    sMotorConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;

    sMotorConfig.OCIdleState  = TIM_OCIDLESTATE_RESET;

    timer_ifF_ConfigPwmChannel(TIM_PWM_ID_1_K);
    timer_ifF_ConfigPwmChannel(TIM_PWM_ID_2_K);
    timer_ifF_ConfigPwmChannel(TIM_PWM_ID_3_K);
    timer_ifF_ConfigPwmChannel(TIM_PWM_ID_4_K);

    /*##-3- Start PWM signals generation #######################################*/
    timer_ifF_StartPwm(TIM_PWM_ID_1_K);
    timer_ifF_StartPwm(TIM_PWM_ID_2_K);
    timer_ifF_StartPwm(TIM_PWM_ID_3_K);
    timer_ifF_StartPwm(TIM_PWM_ID_4_K);
}

/** Initialization of timers used for encoder reading

    @param  none
    @return none
 */
static void timer_ifLF_InitEncoder(void)
{
    /* -1- Initialize TIMx_MOTOR1_ENCODER to handle the encoder sensor */
    /* Initialize TIMx_MOTOR1_ENCODER peripheral as follow:
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

    /* -1- Initialize TIMx_MOTOR2_ENCODER to handle the encoder sensor */
    /* Initialize TIMx_MOTOR2_ENCODER peripheral as follow:
	       + Period = 65535
	       + Prescaler = 0
	       + ClockDivision = 0
	       + Counter direction = Up
    */
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

    /* -1- Initialize TIMx_MOTOR3_ENCODER to handle the encoder sensor */
    /* Initialize TIMx_MOTOR3_ENCODER peripheral as follow:
	       + Period = 65535
	       + Prescaler = 0
	       + ClockDivision = 0
	       + Counter direction = Up
    */
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

    /* -1- Initialize TIMx_MOTOR4_ENCODER to handle the encoder sensor */
    /* Initialize TIMx_MOTOR4_ENCODER peripheral as follow:
	       + Period = 65535
	       + Prescaler = 0
	       + ClockDivision = 0
	       + Counter direction = Up
    */
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

    /* Start the encoder interface timers */
    HAL_TIM_Encoder_Start(&Encoder1_Handle, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&Encoder2_Handle, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&Encoder3_Handle, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&Encoder4_Handle, TIM_CHANNEL_ALL);
}

/** Error handler function

    @param  none
    @return none
 */
static void timer_ifLF_ErrorHandler(void)
{
	// TODO: Save the cause of error
    while(1);
}

/***** Global functions */

/** Module initialization

    @param  none
    @return none

    It will be called once every POR.
 */
void timer_ifF_Init(void)
{
    timer_ifLF_InitPwm();

    timer_ifLF_InitEncoder();

    timer_ifLF_InitFreeRunningTimer();
}

/** Function interface to configure pwm channels

    @param  idx      {[0..n] pwm channel Id}
    @return none
 */
void timer_ifF_ConfigPwmChannel(uint8_t idx)
{
    switch(idx)
    {
        case TIM_PWM_ID_1_K:
        {
            /* Set the duty cycle for channel 1 */
            sMotorConfig.Pulse = 0u;
            if (HAL_TIM_PWM_ConfigChannel(&TimMotorHandle, &sMotorConfig, TIM_CHANNEL_1) != HAL_OK)
            {
                /* Configuration Error */
                timer_ifLF_ErrorHandler();
            }
            break;
        }

        case TIM_PWM_ID_2_K:
        {
            /* Set the duty cycle for channel 2 */
            sMotorConfig.Pulse = 0u;
            if (HAL_TIM_PWM_ConfigChannel(&TimMotorHandle, &sMotorConfig, TIM_CHANNEL_2) != HAL_OK)
            {
                /* Configuration Error */
                timer_ifLF_ErrorHandler();
            }
            break;
        }

        case TIM_PWM_ID_3_K:
        {
            /* Set the duty cycle for channel 3 */
            sMotorConfig.Pulse = 0u;
            if (HAL_TIM_PWM_ConfigChannel(&TimMotorHandle, &sMotorConfig, TIM_CHANNEL_3) != HAL_OK)
            {
                /* Configuration Error */
                timer_ifLF_ErrorHandler();
            }
            break;
        }

        case TIM_PWM_ID_4_K:
        {
            /* Set the duty cycle for channel 4 */
            sMotorConfig.Pulse = 0u;
            if (HAL_TIM_PWM_ConfigChannel(&TimMotorHandle, &sMotorConfig, TIM_CHANNEL_4) != HAL_OK)
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

/** Function interface to start pwm generation

    @param  idx      {[0..n] pwm channel Id}
    @return none
 */
void timer_ifF_StartPwm(uint8_t idx)
{
    switch(idx)
    {
        case TIM_PWM_ID_1_K:
        {
            /* Start PWM signals generation for channel 1 */
            if (HAL_TIM_PWM_Start(&TimMotorHandle, TIM_CHANNEL_1) != HAL_OK)
            {
                /* Configuration Error */
                timer_ifLF_ErrorHandler();
            }
            break;
        }

        case TIM_PWM_ID_2_K:
        {
            /* Start PWM signals generation for channel 2 */
            if (HAL_TIM_PWM_Start(&TimMotorHandle, TIM_CHANNEL_2) != HAL_OK)
            {
                /* Configuration Error */
                timer_ifLF_ErrorHandler();
            }
            break;
        }

        case TIM_PWM_ID_3_K:
        {
            /* Start PWM signals generation for channel 3 */
            if (HAL_TIM_PWM_Start(&TimMotorHandle, TIM_CHANNEL_3) != HAL_OK)
            {
                /* Configuration Error */
                timer_ifLF_ErrorHandler();
            }
            break;
        }

        case TIM_PWM_ID_4_K:
        {
            /* Start PWM signals generation for channel 4 */
            if (HAL_TIM_PWM_Start(&TimMotorHandle, TIM_CHANNEL_4) != HAL_OK)
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

/** Function interface to stop pwm generation

    @param  idx      {[0..n] pwm channel Id}
    @return none
 */
void timer_ifF_StopPwm(uint8_t idx)
{
    switch(idx)
    {
        case TIM_PWM_ID_1_K:
        {
            /* Stop PWM signals generation for channel 1 */
            if (HAL_TIM_PWM_Stop(&TimMotorHandle, TIM_CHANNEL_1) != HAL_OK)
            {
                /* Configuration Error */
                timer_ifLF_ErrorHandler();
            }
            break;
        }

        case TIM_PWM_ID_2_K:
        {
            /* Stop PWM signals generation for channel 2 */
            if (HAL_TIM_PWM_Stop(&TimMotorHandle, TIM_CHANNEL_2) != HAL_OK)
            {
                /* Configuration Error */
                timer_ifLF_ErrorHandler();
            }
            break;
        }

        case TIM_PWM_ID_3_K:
        {
            /* Stop PWM signals generation for channel 3 */
            if (HAL_TIM_PWM_Stop(&TimMotorHandle, TIM_CHANNEL_3) != HAL_OK)
            {
                /* Configuration Error */
                timer_ifLF_ErrorHandler();
            }
            break;
        }

        case TIM_PWM_ID_4_K:
        {
            /* Stop PWM signals generation for channel 4 */
            if (HAL_TIM_PWM_Stop(&TimMotorHandle, TIM_CHANNEL_4) != HAL_OK)
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

/** Function interface to update the pwm duty cycle

    @param  idx      {[0..n] pwm channel Id}
    @return none
 */
void timer_ifF_UpdatePwm(uint8_t idx, uint8_t duty_cycle_percentage)
{
    uint32_t duty_cycle;

    // Get the equivalent duty cycle value based on the received pwm percentage
    duty_cycle = (((PERIOD_VALUE*PWM_RESOLUTION) / PWM_PERCENT_TO_DUTY_CYCLE) * duty_cycle_percentage) / PWM_RESOLUTION;

    switch(idx)
    {
        case TIM_PWM_ID_1_K:
        {
            TIMx_MOTOR_CTRL->CCR1 = duty_cycle;
            break;
        }

        case TIM_PWM_ID_2_K:
        {
            TIMx_MOTOR_CTRL->CCR2 = duty_cycle;
            break;
        }

        case TIM_PWM_ID_3_K:
        {
            TIMx_MOTOR_CTRL->CCR3 = duty_cycle;
            break;
        }

        case TIM_PWM_ID_4_K:
        {
            TIMx_MOTOR_CTRL->CCR4 = duty_cycle;
            break;
        }

        default:
            /* no action */
            break;
    }
}

/** Function interface to return back the encoder counts captured by timer

    @param  idx              {[0..n] pwm channel Id}
    @return encoder counts
 */
uint32_t timer_ifF_getEncoderCount(uint8_t idx)
{
	uint32_t ret_enc_cnt = 0u;

    switch(idx)
    {
        case MOTOR_ID_1_K:
        {
            ret_enc_cnt = TIMx_MOTOR1_ENCODER->CNT;
            break;
        }
        case MOTOR_ID_2_K:
        {
            ret_enc_cnt = TIMx_MOTOR2_ENCODER->CNT;
            break;
        }
        case MOTOR_ID_3_K:
        {
            ret_enc_cnt = TIMx_MOTOR3_ENCODER->CNT;
            break;
        }
        case MOTOR_ID_4_K:
        {
            ret_enc_cnt = TIMx_MOTOR4_ENCODER->CNT;
            break;
        }
        default:
            /* no action */
            break;
    }

    return (ret_enc_cnt);
}

/** Function interface to return back the encoder counts direction captured by timer

    @param  idx              {[0..n] pwm channel Id}
    @return Count_up=0; Count_down=1
 */
uint32_t timer_ifF_getEncoderCountDirection(uint8_t idx)
{
    uint32_t ret_dir = 0u;

    switch(idx)
    {
        case MOTOR_ID_1_K:
        {
            ret_dir = __HAL_TIM_IS_TIM_COUNTING_DOWN(&Encoder1_Handle);
            break;
        }
        case MOTOR_ID_2_K:
        {
            ret_dir = __HAL_TIM_IS_TIM_COUNTING_DOWN(&Encoder2_Handle);
            break;
        }
        case MOTOR_ID_3_K:
        {
            ret_dir = __HAL_TIM_IS_TIM_COUNTING_DOWN(&Encoder3_Handle);
            break;
        }
        case MOTOR_ID_4_K:
        {
            ret_dir = __HAL_TIM_IS_TIM_COUNTING_DOWN(&Encoder4_Handle);
            break;
        }
        default:
            /* no action */
            break;
    }

    return (ret_dir);
}

/** Free-running timer ISR handler

    @param  none
    @return none
 */
void TIM7_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&TimFreeRunningHandle);
}

/** Free-running timer callback function.
    The function is called automatically everytime the timer value has elapsed.

    @param  none
    @return none

    This function is used to process further the encoder counts.
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    static uint8_t TEST_ctr = 0u;

    /* read and process the encoder counts */
    encoderF_ReadCounts_Callback();

    TEST_ctr++;
    if(TEST_ctr >= 50)
    {
        TEST_ctr = 0u;
        //motorF_StateMachine();
    }


    //HAL_GPIO_TogglePin(LD2_GPIO_Port, GPIO_PIN_0); // for TESTING ONLY
    //HAL_GPIO_TogglePin(LD2_GPIO_Port, GPIO_PIN_1); // for TESTING ONLY
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
    /* GPIO clock is initialized in function "mainLF_InitGpioClock" */

    /* Configure (TIM1_Channel1), (TIM1_Channel2), (TIM1_Channel3),
     (TIM1_Channel4) in output, push-pull, alternate function mode
    */

    /* Common configuration for all channels */
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

    /* Pwm Channel 1 */
    GPIO_InitStruct.Alternate = TIMx_MOTOR_CTRL_GPIO_AF_CHANNEL1;
    GPIO_InitStruct.Pin = TIMx_MOTOR_CTRL_GPIO_PIN_CHANNEL1;
    HAL_GPIO_Init(TIMx_MOTOR_CTRL_GPIO_PORT_CHANNEL1, &GPIO_InitStruct);

    /* Pwm Channel 2 */
    GPIO_InitStruct.Alternate = TIMx_MOTOR_CTRL_GPIO_AF_CHANNEL2;
    GPIO_InitStruct.Pin = TIMx_MOTOR_CTRL_GPIO_PIN_CHANNEL2;
    HAL_GPIO_Init(TIMx_MOTOR_CTRL_GPIO_PORT_CHANNEL2, &GPIO_InitStruct);

    /* Pwm Channel 3 */
    GPIO_InitStruct.Alternate = TIMx_MOTOR_CTRL_GPIO_AF_CHANNEL3;
    GPIO_InitStruct.Pin = TIMx_MOTOR_CTRL_GPIO_PIN_CHANNEL3;
    HAL_GPIO_Init(TIMx_MOTOR_CTRL_GPIO_PORT_CHANNEL3, &GPIO_InitStruct);

    /* Pwm Channel 4 */
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
    /* TIMx_MOTOR1_ENCODER Peripheral clock enable */
    TIMx_MOTOR1_ENCODER_CLK_ENABLE();
    /* TIMx_MOTOR2_ENCODER Peripheral clock enable */
    TIMx_MOTOR2_ENCODER_CLK_ENABLE();
    /* TIMx_MOTOR3_ENCODER Peripheral clock enable */
    TIMx_MOTOR3_ENCODER_CLK_ENABLE();
    /* TIMx_MOTOR4_ENCODER Peripheral clock enable */
    TIMx_MOTOR4_ENCODER_CLK_ENABLE();

    /* Enable GPIO Channels Clock */
    /* GPIO clock is initialized in function "mainLF_InitGpioClock" */

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

/**
  * @brief  Initializes the TIM Base MSP.
  * @param  htim pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval None
  */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
    /* 1. Enable timer TIMx_FREE_RUNNING_TIMER peripheral clock */
    TIMx_FREE_RUNNING_TIMER_CLK_ENABLE();

    /* 2. Configure IO */
    /* This is not needed as this is just a free running timer */

    /* 3. Interrupt init */
    HAL_NVIC_SetPriority(TIM7_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM7_IRQn);
}
