/*
 * gpio_if.c
 *
 *  Created on: 12 May 2021
 *      Author: Alfonso, Rudy Manalo
 */

 /* Project includes. */
#include "project.h"
#include "motor.h"
#include "gpio_if.h"

/***** Macros */
#if defined(DEBUG_PORT_USED)
#define DEBUG_PORT_DO_PIN_REG_K                  GPIOE
#define DEBUG_PORT_DO_PIN_1_K                    GPIO_PIN_10
#define DEBUG_PORT_DO_PIN_2_K                    GPIO_PIN_12
#endif
/* GPIO motor control pins */
#define MOTOR_CTRL_NUM_GPIO_PIN_K                (uint8_t)(2) /* two digital output pins per motor to control the direction */
#define MOTOR1_DO_PIN_REG_K                      GPIOB
#define MOTOR1_DO_PIN_1_K                        GPIO_PIN_11
#define MOTOR1_DO_PIN_2_K                        GPIO_PIN_10
#define MOTOR2_DO_PIN_REG_K                      GPIOB
#define MOTOR2_DO_PIN_1_K                        GPIO_PIN_1
#define MOTOR2_DO_PIN_2_K                        GPIO_PIN_0
#if defined(VAR_4WD_USED)
#define MOTOR3_DO_PIN_REG_K                      GPIOD
#define MOTOR3_DO_PIN_1_K                        GPIO_PIN_15
#define MOTOR3_DO_PIN_2_K                        GPIO_PIN_14
#define MOTOR4_DO_PIN_REG_K                      GPIOC
#define MOTOR4_DO_PIN_1_K                        GPIO_PIN_8
#define MOTOR4_DO_PIN_2_K                        GPIO_PIN_9
#endif // VAR_4WD_USED


/* Motor Id to GPIO Register mapping */
static GPIO_TypeDef *motorId2GpioReg[MOTOR_NUM_ID_K] =
{
    MOTOR1_DO_PIN_REG_K
   ,MOTOR2_DO_PIN_REG_K
#if defined(VAR_4WD_USED)
   ,MOTOR3_DO_PIN_REG_K
   ,MOTOR4_DO_PIN_REG_K
#endif // VAR_4WD_USED
};

/* Motor Id to GPIO Pin mapping */
static const uint16_t motorId2GpioPin[MOTOR_NUM_ID_K][MOTOR_CTRL_NUM_GPIO_PIN_K] =
{
    {MOTOR1_DO_PIN_1_K, MOTOR1_DO_PIN_2_K}
   ,{MOTOR2_DO_PIN_1_K, MOTOR2_DO_PIN_2_K}
#if defined(VAR_4WD_USED)
   ,{MOTOR3_DO_PIN_1_K, MOTOR3_DO_PIN_2_K}
   ,{MOTOR4_DO_PIN_1_K, MOTOR4_DO_PIN_2_K}
#endif // VAR_4WD_USED
};


/***** Local function prototypes */
#if defined(DEBUG_PORT_USED)
static void gpio_ifLF_InitDebugPort(void);
#endif
static void gpio_ifLF_InitGpioClock(void);
static void gpio_ifLF_InitGpioMotorDirectionControl(void);

/***** Local functions */

#if defined(DEBUG_PORT_USED)
/** GPIO Debug Port Initialization
    The debug ports are used for debugging, cyclic time measurement, testing, etc..

    @param  none
    @return none

    It will be called once every POR.
 */
static void gpio_ifLF_InitDebugPort(void)
{
    GPIO_InitTypeDef GPIO_debugInitStruct;

    /*Configure GPIO for debugging */
    GPIO_debugInitStruct.Pin = DEBUG_PORT_DO_PIN_1_K;
    GPIO_debugInitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_debugInitStruct.Pull = GPIO_NOPULL;
    GPIO_debugInitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DEBUG_PORT_DO_PIN_REG_K, &GPIO_debugInitStruct);

    /*Configure GPIO for debugging */
    GPIO_debugInitStruct.Pin = DEBUG_PORT_DO_PIN_2_K;
    GPIO_debugInitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_debugInitStruct.Pull = GPIO_NOPULL;
    GPIO_debugInitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DEBUG_PORT_DO_PIN_REG_K, &GPIO_debugInitStruct);
}
#endif // DEBUG_PORT_USED

/** GPIO Clock Initialization

    @param  none
    @return none

    It will be called once every POR.
 */
static void gpio_ifLF_InitGpioClock(void)
{
    /** GPIO Ports Clock Enable */
    /* IO for motor1 channel1 */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /* IO for motor1 channel 2 and motor2 encoders */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /* IO for motor4 encoders */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    /* IO for motor3 encoder and USART3 for ROSserial communication */
    __HAL_RCC_GPIOD_CLK_ENABLE();
    /* IO for motor1, motor2, motor3, and motor4 pwm generation */
    __HAL_RCC_GPIOE_CLK_ENABLE();
}

/** GPIO Initialization for motor direction control

    @param  none
    @return none

    It will be called once every POR.
 */
static void gpio_ifLF_InitGpioMotorDirectionControl(void)
{
    GPIO_InitTypeDef GPIO_motorInitStruct;

    /*Configure GPIO for Motor1 pin1 */
    GPIO_motorInitStruct.Pin = MOTOR1_DO_PIN_1_K;
    GPIO_motorInitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_motorInitStruct.Pull = GPIO_NOPULL;
    GPIO_motorInitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(MOTOR1_DO_PIN_REG_K, &GPIO_motorInitStruct);

    /*Configure GPIO for Motor1 pin2 */
    GPIO_motorInitStruct.Pin = MOTOR1_DO_PIN_2_K;
    GPIO_motorInitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_motorInitStruct.Pull = GPIO_NOPULL;
    GPIO_motorInitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(MOTOR1_DO_PIN_REG_K, &GPIO_motorInitStruct);

    /*Configure GPIO for Motor2 pin1 */
    GPIO_motorInitStruct.Pin = MOTOR2_DO_PIN_1_K;
    GPIO_motorInitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_motorInitStruct.Pull = GPIO_NOPULL;
    GPIO_motorInitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(MOTOR2_DO_PIN_REG_K, &GPIO_motorInitStruct);

    /*Configure GPIO for Motor2 pin2 */
    GPIO_motorInitStruct.Pin = MOTOR2_DO_PIN_2_K;
    GPIO_motorInitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_motorInitStruct.Pull = GPIO_NOPULL;
    GPIO_motorInitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(MOTOR2_DO_PIN_REG_K, &GPIO_motorInitStruct);

#if defined(VAR_4WD_USED)
    /*Configure GPIO for Motor3 pin1 */
    GPIO_motorInitStruct.Pin = MOTOR3_DO_PIN_1_K;
    GPIO_motorInitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_motorInitStruct.Pull = GPIO_NOPULL;
    GPIO_motorInitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(MOTOR3_DO_PIN_REG_K, &GPIO_motorInitStruct);

    /*Configure GPIO for Motor3 pin2 */
    GPIO_motorInitStruct.Pin = MOTOR3_DO_PIN_2_K;
    GPIO_motorInitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_motorInitStruct.Pull = GPIO_NOPULL;
    GPIO_motorInitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(MOTOR3_DO_PIN_REG_K, &GPIO_motorInitStruct);

    /*Configure GPIO for Motor4 pin1 */
    GPIO_motorInitStruct.Pin = MOTOR4_DO_PIN_1_K;
    GPIO_motorInitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_motorInitStruct.Pull = GPIO_NOPULL;
    GPIO_motorInitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(MOTOR4_DO_PIN_REG_K, &GPIO_motorInitStruct);

    /*Configure GPIO for Motor4 pin2 */
    GPIO_motorInitStruct.Pin = MOTOR4_DO_PIN_2_K;
    GPIO_motorInitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_motorInitStruct.Pull = GPIO_NOPULL;
    GPIO_motorInitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(MOTOR4_DO_PIN_REG_K, &GPIO_motorInitStruct);
#endif // VAR_4WD_USED
}

/***** Global functions */

/** Module initialization

    @param  none
    @return none

    It will be called once every POR.
 */
void gpio_ifF_Init(void)
{
    gpio_ifLF_InitGpioClock();

    gpio_ifLF_InitGpioMotorDirectionControl();

#if defined(DEBUG_PORT_USED)
    gpio_ifLF_InitDebugPort();
#endif
}

/** Function to set the motor direction

    @param  mot_id   {[0..MOTOR_NUM_ID_K] motor ID}
            dir      {[0..2] motor direction      }
    @return none
 */
void gpio_ifF_setMotorDirection(uint8_t mot_id, uint8_t dir)
{
    if(MOTOR_FORWARD_DIRECTION_K == dir)
    {
        /* motor is in forward direction */
        HAL_GPIO_WritePin(motorId2GpioReg[mot_id], motorId2GpioPin[mot_id][0], GPIO_PIN_SET);
        HAL_GPIO_WritePin(motorId2GpioReg[mot_id], motorId2GpioPin[mot_id][1], GPIO_PIN_RESET);
    }
    else if(MOTOR_BACKWARD_DIRECTION_K == dir)
    {
        /* motor is in backward direction */
        HAL_GPIO_WritePin(motorId2GpioReg[mot_id], motorId2GpioPin[mot_id][0], GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motorId2GpioReg[mot_id], motorId2GpioPin[mot_id][1], GPIO_PIN_SET);
    }
    else
    {
        /* no direction. switch-off the control */
        HAL_GPIO_WritePin(motorId2GpioReg[mot_id], motorId2GpioPin[mot_id][0], GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motorId2GpioReg[mot_id], motorId2GpioPin[mot_id][1], GPIO_PIN_RESET);
    }
}

#if defined(DEBUG_PORT_USED)
/** Function to switch-on/off the debug port

    @param  debug_id    {[0..n] debug port Id}
            status      {[0..1] OFF/ON       }
    @return none
 */
void gpio_ifF_setDebugPort(uint8_t debug_id, uint8_t status)
{
    if(0u == debug_id)
    {
        HAL_GPIO_WritePin(DEBUG_PORT_DO_PIN_REG_K, DEBUG_PORT_DO_PIN_1_K, status);
    }
    else if(1u == debug_id)
    {
        HAL_GPIO_WritePin(DEBUG_PORT_DO_PIN_REG_K, DEBUG_PORT_DO_PIN_2_K, status);
    }
    else
    {
        /* for future expansion of debug port */
    }
}

/** Function to toggle the debug port

    @param  debug_id    {[0..n] debug port Id}
    @return none
 */
void gpio_ifF_toggleDebugPort(uint8_t debug_id)
{
    if(0u == debug_id)
    {
        HAL_GPIO_TogglePin(DEBUG_PORT_DO_PIN_REG_K, DEBUG_PORT_DO_PIN_1_K);
    }
    else if(1u == debug_id)
    {
        HAL_GPIO_TogglePin(DEBUG_PORT_DO_PIN_REG_K, DEBUG_PORT_DO_PIN_2_K);
    }
    else
    {
        /* for future expansion of debug port */
    }
}
#endif // DEBUG_PORT_USED
