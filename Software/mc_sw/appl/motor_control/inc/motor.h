/*
 * motor.h
 *
 *  Created on: 11 May 2021
 *      Author: Alfonso, Rudy Manalo
 */

#ifndef MOTOR_CONTROL_INC_MOTOR_H_
#define MOTOR_CONTROL_INC_MOTOR_H_

#define MOTOR_FORWARD_DIRECTION_K        (uint8_t)(1)
#define MOTOR_BACKWARD_DIRECTION_K       (uint8_t)(2)

/** Module initialization

    @param  none
    @return none

    It will be called once every POR.
 */
void motorF_Init(void);

/** Module state machine

    @param  none
    @return none

    It will be called cyclically every MOTOR_TASK_MS_K.
 */
void motorF_StateMachine(void);

/** Interface function to set the required motor pwm and direction

    @param  mot_id   {[0..MOTOR_NUM_ID_K] motor ID}
            pwm      { motor pwm }
    @return none
 */
void motorF_SetPwmAndDirection(uint8_t mot_id, int16_t pwm);


#endif /* MOTOR_CONTROL_INC_MOTOR_H_ */
