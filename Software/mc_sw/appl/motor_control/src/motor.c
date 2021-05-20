/*
 * motor.c
 *
 *  Created on: 11 May 2021
 *      Author: Alfonso, Rudy Manalo
 */

/* Standard includes. */
#include <string.h> /* for memset */
#include <stdlib.h> /* for absolute value */
 /* Project includes. */
#include "project.h"
#include "gpio_if.h"
#include "timer_if.h"
#include "motor.h"

/***** Macros */
#define MOTOR_TASK_MS_K                  (uint8_t)(50)
#define MOTOR_NEW_DIR_DELAY_TIME_MS_K    (uint8_t)(500 / MOTOR_TASK_MS_K)

/***** Variables */
enum motor_state_et
{
    MOTOR_STOP_E,
    MOTOR_RUNNING,
    MOTOR_STOP_OVER_E
};

struct motor_st
{
    enum motor_state_et    motor_state_e;
    uint8_t                current_direction;
    uint8_t                prev_direction;
    uint8_t                pwm_duty_cycle;
    uint16_t               new_dir_delay_time;
    uint32_t               pwm_period;
};

static struct motor_st     motor_s[MOTOR_NUM_ID_K];


/***** Local function prototypes */
static void motorLF_Stop(uint8_t mot_id);
static void motorLF_Start(uint8_t mot_id);

/***** Local functions */

/** Function to stop the motor

    @param  mot_id   {[0..MOTOR_NUM_ID_K] motor ID}
    @return none
 */
static void motorLF_Stop(uint8_t mot_id)
{
    /* reset the control digital output pins */
    gpio_ifF_setMotorDirection(mot_id, 0u);

    /* reset pwm */
    timer_ifF_UpdatePwm(mot_id, 0u);
}

/** Function to start the motor

    @param  mot_id   {[0..MOTOR_NUM_ID_K] motor ID}
    @return none
 */
static void motorLF_Start(uint8_t mot_id)
{
    /* direction plausibility check */
    if(motor_s[mot_id].current_direction != 0u)
    {
        /* set the pwm duty cycle */
        timer_ifF_UpdatePwm(mot_id, motor_s[mot_id].pwm_duty_cycle);

        if(MOTOR_FORWARD_DIRECTION_K == motor_s[mot_id].current_direction)
        {
            /* motor is in forward direction */
            gpio_ifF_setMotorDirection(mot_id, MOTOR_FORWARD_DIRECTION_K);
        }
        else
        {
            /* motor is in backward direction */
            gpio_ifF_setMotorDirection(mot_id, MOTOR_BACKWARD_DIRECTION_K);
        }
    }
}

/***** Global functions */

/** Module initialization

    @param  none
    @return none

    It will be called once every POR.
 */
void motorF_Init(void)
{
    (void)memset(&motor_s, 0x00, sizeof(motor_s));
}

/** Module state machine

    @param  none
    @return none

    It will be called cyclically every MOTOR_TASK_MS_K.
 */
void motorF_StateMachine(void)
{
    uint8_t idx = 0u;

    for(idx = 0u; idx < (uint8_t)MOTOR_NUM_ID_K; idx++)
    {
        switch(motor_s[idx].motor_state_e)
        {
            case MOTOR_STOP_E:
            {
                if(motor_s[idx].pwm_duty_cycle != 0u)
                {
                    /* start the motor */
                    motorLF_Start(idx);

                    /* assign current direction since motor is starting-up */
                    motor_s[idx].prev_direction = motor_s[idx].current_direction;

                    /* move to MOTOR_RUNNING state */
                    motor_s[idx].motor_state_e = MOTOR_RUNNING;
                }
                else{/* no action */}
                break;
            }
            case MOTOR_RUNNING:
            {
                if(0u == motor_s[idx].pwm_duty_cycle)
                {
                    /* stop the motor */
                    motorLF_Stop(idx);

                    /* move to MOTOR_STOP_E state */
                    motor_s[idx].motor_state_e = MOTOR_STOP_E;
                }
                else if(motor_s[idx].current_direction != motor_s[idx].prev_direction)
                {
                    /* stop the motor */
                    motorLF_Stop(idx);

                    /* save the current direction to be used later */
                    motor_s[idx].prev_direction = motor_s[idx].current_direction;

                    /* reset the new direction delay time */
                    motor_s[idx].new_dir_delay_time = 0u;

                    /* move to MOTOR_STOP_OVER_E state */
                    motor_s[idx].motor_state_e = MOTOR_STOP_OVER_E;
                }
                else
                {
                    /* update pwm or direction if needed */
                    motorLF_Start(idx);
                }
                break;
            }
            case MOTOR_STOP_OVER_E:
            {
                motor_s[idx].new_dir_delay_time++;
                if(motor_s[idx].new_dir_delay_time >= MOTOR_NEW_DIR_DELAY_TIME_MS_K)
                {
                    if(motor_s[idx].pwm_duty_cycle != 0u)
                    {
                        /* activate back the motor */

                        /* start the motor */
                        motorLF_Start(idx);

                        /* move to MOTOR_RUNNING state */
                        motor_s[idx].motor_state_e = MOTOR_RUNNING;
                    }
                    else
                    {
                        /* request is no longer present */

                        /* move to MOTOR_STOP_E state */
                        motor_s[idx].motor_state_e = MOTOR_STOP_E;
                    }
                }
                break;
            }
            default:
                /* no action */
                break;
        }
    }
}

/** Interface function to set the required motor pwm and direction

    @param  mot_id   {[0..MOTOR_NUM_ID_K] motor ID}
            pwm      { motor pwm }
    @return none
 */
void motorF_SetPwmAndDirection(uint8_t mot_id, int16_t pwm)
{
    int16_t tmp_abs = 0;

    tmp_abs = abs(pwm);

    motor_s[mot_id].pwm_duty_cycle = (uint8_t)tmp_abs;

    if(pwm > 0)
    {
        motor_s[mot_id].current_direction = MOTOR_FORWARD_DIRECTION_K;
    }
    else if(pwm < 0)
    {
        motor_s[mot_id].current_direction = MOTOR_BACKWARD_DIRECTION_K;
    }
    else{/* no action */}
}

// TODO: Add motor over current error checking
