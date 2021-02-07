/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/

// for semihosting and UART debugging includes
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "stm32f4xx.h"
//! freeRTOS includes
#include "FreeRTOS.h" // freeRTOS main header file
#include "task.h" // in able to create a task
			
// create task handler variables
TaskHandle_t xTaskHandle1=NULL;
TaskHandle_t xTaskHandle2=NULL;

// task functions prototype
void led_task_handler(void *params);
#ifdef BUTTON_TASK_USED
void button_task_handler(void *params);
#else
void button_ISR_handler(void);
#endif
static void prvSetupUart(void);
static void prvSetupGpio(void);
static void pvSetupHardware(void);
void printmsg(char *msg);

#ifdef USE_SEMIHOSTING
// use for semihosting
extern void initialise_monitor_handles();
#endif
#define TRUE 1
#define FALSE 0
#define AVAILABLE (TRUE)
#define NOT_AVAILABLE (FALSE)
#define NOT_PRESSED (FALSE)
#define PRESSED (TRUE)

char usr_msg[250]={0};
uint8_t UART_ACCESS_KEY = AVAILABLE;

uint8_t button_status_flag = NOT_PRESSED;


int main(void)
{
#ifdef USE_SEMIHOSTING
	// must be called before any printf
	initialise_monitor_handles();

	printf("This is a hello world example code \n");
#endif

	// SEGGER INtegration - Enable the cycle counting for timestamp
	DWT->CTRL |= (1 << 0);

	// 1. Reset the RCC clock configuration to the default reset state.
	// HSI ON, PLL OFF, HSE OFF, system clock = 16MHz, cpu_clock = 16MHz
	// note: 16MHz because HSI = 16MHz. The info can be found in the datasheet
	RCC_DeInit();

	// 2. update the SystemCoreClock variable
	SystemCoreClockUpdate();

	// Hardware setup
	pvSetupHardware();

	// copy the message into the variable "usr_msg"
	//sprintf(usr_msg, "Starting LED and Button software\r\n");
	// print the data
	//printmsg(usr_msg);

	// SEGGER Integration - start recording
	SEGGER_SYSVIEW_Conf();
	SEGGER_SYSVIEW_Start();

	// 3. Create 2 tasks: task-1 and task-2
	xTaskCreate( led_task_handler,
	             "LED-TASK",
				 configMINIMAL_STACK_SIZE, /* 130 words = 520 Bytes (1word = 4Byte) */
	             NULL, /* not sending any parameter */
	             1, /* 0 priority is the lowest priority or the idle task: MAX priority is defined "configMAX_PRIORITIES" */
	             NULL ); /* this is like ID number. can be used to delete or suspend the task */

#ifdef BUTTON_TASK_USED
	xTaskCreate( button_task_handler,
	             "BUTTON-TASK",
				 configMINIMAL_STACK_SIZE, /* 130 words = 520 Bytes (1word = 4Byte) */
	             NULL, /* not sending any parameter */
	             1, /* 0 priority is the lowest priority or the idle task: MAX priority is defined "configMAX_PRIORITIES" */
	             NULL ); /* this is like ID number. can be used to delete or suspend the task */
#endif

	// 4. Start the scheduler. This will start executing the tasks that have been created in step #3.
	vTaskStartScheduler(); // this function will never return because it will give control to the task

	// Since "vTaskStartScheduler" will never return this for loop will never be executed unless there is a problem.
	for(;;);
}

// LED task handler/implementation
void led_task_handler(void *params)
{
	// task handler should never return
    while(1)
    {
        if(PRESSED == button_status_flag)
        {
        	// turn ON LED
        	// write bit is okay since we are writing to specific bit
        	GPIO_WriteBit(GPIOB, GPIO_Pin_7, Bit_SET);
        }
        else
        {
        	// switch-OFF LED
        	GPIO_WriteBit(GPIOB, GPIO_Pin_7, Bit_RESET);
        }
    }
}

#ifdef BUTTON_TASK_USED
// Button task handler/implementation
void button_task_handler(void *params)
{
	// task handler should never return
	while(1)
	{
        // get button status
		// read the button pin
		// read bit is okay since we are writing to specific bit
		if(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13))
		{
			// button is pressed
			// in my NUCLEO board, if button is pressed it will return HIGH
			button_status_flag = PRESSED;
		}
		else
		{
			// button is not pressed
			// in my NUCLEO board, if button is not pressed it will return LOW
			button_status_flag = NOT_PRESSED;
		}

	}
}
#else
// Button ISR handler
void button_ISR_handler(void)
{
	// toggle the status whenever the button is pressed
	button_status_flag ^= 1; // XOR
}
#endif

// UART3 configuration and initialization
static void prvSetupUart(void)
{
	GPIO_InitTypeDef gpio_uart_pins;
	USART_InitTypeDef uart3_init;

    // 1. Enable the UART3
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);


	// 2. Configure the alternate function of MCU pins(PD8 and PD9) to behave as UART3_TX and UART3_RX
	// PD8 = UART3_TX, PD9 = UART3_RX
	// first, initialize everything to zero as the variables contain garbage values
    memset(&gpio_uart_pins, 0, sizeof(gpio_uart_pins));
	gpio_uart_pins.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9; // TX and RX
	gpio_uart_pins.GPIO_Mode = GPIO_Mode_AF; // Alternate function
	gpio_uart_pins.GPIO_PuPd = GPIO_PuPd_UP; // pull-up/pull-down; recommend to set to Pull-up because
	                                         // when line is idle it is set to 1 or pull to logical 1
	// parameters: base address of the Pin (in this case D), GPIO pin configuration
	GPIO_Init(GPIOD, &gpio_uart_pins);

	// 3. AF mode settings for the pins
	// set pin PD8 as UART3_TX. GPIO_AF_USART3: Connect USART3 pins to AF7. Check datasheet for more info
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_USART3);
	// set pin PD9 as UART3_RX. GPIO_AF_USART3: Connect USART3 pins to AF7. Check datasheet for more info
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_USART3);

	// 4. UART parameter initializations
	// first, initialize everything to zero as the variables contain garbage values
    memset(&uart3_init, 0, sizeof(uart3_init));
	uart3_init.USART_BaudRate = 115200;
	uart3_init.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	uart3_init.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	uart3_init.USART_Parity = USART_Parity_No;
	uart3_init.USART_StopBits = USART_StopBits_1;
	uart3_init.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART3, &uart3_init);

	// 5. Enable the UART3 peripheral
	USART_Cmd(USART3, ENABLE);
}

// for LED, I will be using the USER LED2 connected to PB7
// for Button, it is connected to PC13
static void prvSetupGpio(void)
{
	GPIO_InitTypeDef led_init, button_init;

	// 1. Enable the GPIOB and GPIOC Peripheral clock
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); // for GPIOB
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); // for GPIOC
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);  // for GPIOC external interrupt

    // 2. LED initialization
	led_init.GPIO_Mode = GPIO_Mode_OUT;
	led_init.GPIO_OType = GPIO_OType_PP; // push-pull
	led_init.GPIO_Pin = GPIO_Pin_7;
	led_init.GPIO_Speed = GPIO_Low_Speed;
	led_init.GPIO_PuPd = GPIO_PuPd_NOPULL; // no pull-up or pull-down
	GPIO_Init(GPIOB, &led_init);

    // 2. Button initialization
	button_init.GPIO_Mode = GPIO_Mode_IN;
	button_init.GPIO_OType = GPIO_OType_PP; // push-pull
	button_init.GPIO_Pin = GPIO_Pin_13;
	button_init.GPIO_Speed = GPIO_Low_Speed;
	button_init.GPIO_PuPd = GPIO_PuPd_NOPULL; // no pull-up or pull-down
	GPIO_Init(GPIOC, &button_init);

	// interrupt configuration for the button(PC13)
	// 1. system configuration for EXTI line (SYSCFG settings)
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource13);

	// 2. EXTI line configuration 13, falling edge, interrupt mode
	EXTI_InitTypeDef exti_init;
	exti_init.EXTI_Line = EXTI_Line13;
	exti_init.EXTI_Mode = EXTI_Mode_Interrupt;
	exti_init.EXTI_Trigger = EXTI_Trigger_Falling;
	exti_init.EXTI_LineCmd = ENABLE;
	EXTI_Init(&exti_init);

	// 3. NVIC settings (IRQ settings for the selected EXTI line*(13)
    // set hardware priority: 0 is the highest priority; 15 is the lowest
	NVIC_SetPriority(EXTI15_10_IRQn, 5);
	// enable the IRQ number
	NVIC_EnableIRQ(EXTI15_10_IRQn);
}

static void pvSetupHardware(void)
{
	// Setup button and LED
	prvSetupGpio();

    // setup UART2
	prvSetupUart();
}

void printmsg(char *msg)
{

	for (uint32_t i=0; i < strlen(msg); i++)
	{
		// make sure the Transmit data register is empty before transmitting new data
		while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) != SET);
		// transmit data
		USART_SendData(USART3, msg[i]);
	}
}

void EXTI15_10_IRQHandler(void)
{
	// SEGGER Trace
	traceISR_ENTER();

    // 1. clear the interrupt pending bit of the EXTI line (pin 13)
	EXTI_ClearITPendingBit(EXTI_Line13);

	// call the button ISR handler
	button_ISR_handler();

	traceISR_EXIT();
}
