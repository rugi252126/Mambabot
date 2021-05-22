/*
 * startup.c
 *
 *  Created on: 22 May 2021
 *      Author: Alfonso, Rudy Manalo
 */

/* Project includes. */
/* hal */
#include "stm32f4xx_hal.h"
/* application and interfaces */
#include "project.h"
#include "ros_if.h"
#include "uart_if.h"
#include "gpio_if.h"
#include "timer_if.h"
#include "encoder.h"
#include "motor.h"
#include "scheduler.h"


/***** Private Variables */
enum init_state_et
{
    HW_INIT_E,
    HW_SW_INIT_E,
    SW_INIT_E
};


/***** Local function prototypes */
static void startupLF_InitSequence(enum init_state_et  state_e);
static void SystemClock_Config(void);

/***** Local functions */

/** The application entry point.
    This will be called after RESET and serve as the application entry point.

    @param  none
    @return none
 */
static void startupLF_InitSequence(enum init_state_et  state_e)
{
    switch(state_e)
    {
        case HW_INIT_E:
        {
#if 0 /* if freeRTOS is integrated in a project, no need to call HAL_Init() because freeRTOS will handle the systick timer configuration */
            /* MCU Configuration----------------------------------------------------------*/
                /* STM32F4xx HAL library initialization:
                     - Configure the Flash prefetch
                     - Systick timer is configured by default as source of time base, but user
                       can eventually implement his proper time base source (a general purpose
                       timer for example or other time source), keeping in mind that Time base
                       duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and
                       handled in milliseconds basis.
                     - Set NVIC Group Priority to 4
                     - Low Level Initialization
                   */
            /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
            HAL_Init();
#endif

            /* Configure the system clock to 180 MHz */
            SystemClock_Config();

            /* Initialize all configured peripherals */
            /* GPIOs */
            gpio_ifF_Init();
            /* USART3 */
            uart_ifF_Init_USART3_UART();
            /* Timers */
            timer_ifF_Init();
            break;
        }
        case HW_SW_INIT_E:
        {
            /* ROS initialization */
            ros_ifF_Init();
            break;
        }
        case SW_INIT_E:
        {
            /* encoder application initialization */
            encoderF_Init();
            /* motor application initialization */
            motorF_Init();
            break;
        }
        default:
            /* no action */
            break;
    }
}

/* 180Mhz system clock */
/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 180000000
  *            HCLK(Hz)                       = 180000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 8
  *            PLL_N                          = 360
  *            PLL_P                          = 2
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_OscInitTypeDef RCC_OscInitStruct;

    /* Enable Power Control clock */
    __HAL_RCC_PWR_CLK_ENABLE();

    /* The voltage scaling allows optimizing the power consumption when the device is
       clocked below the maximum system frequency, to update the voltage scaling value
       regarding system frequency refer to product datasheet.  */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /* Enable HSE Oscillator and activate PLL with HSE as source */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 360;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    /* Activate the Over-Drive mode */
    HAL_PWREx_EnableOverDrive();

    /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
       clocks dividers */
    RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}

/***** Global functions */

/** The application entry point.
    This will be called after RESET and serve as the application entry point.

    @param  none
    @return none
 */
int startupF_Init(void)
{
    /* hardware/microcontroller related initialization */
    startupLF_InitSequence(HW_INIT_E);

    /* initialization of hardware and software related data */
    startupLF_InitSequence(HW_SW_INIT_E);

    /* initialization of application data */
    startupLF_InitSequence(SW_INIT_E);

    /* create task and start the application */
    schedulerF_Task();

    /* Infinite loop. It won't come here unless there is a problem */
    while(1){};
}
