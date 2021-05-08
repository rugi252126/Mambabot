/*
 * timer_if.h
 *
 *  Created on: 1 May 2021
 *      Author: Alfonso, Rudy Manalo
 */

#ifndef TIMER_IF_INC_TIMER_IF_H_
#define TIMER_IF_INC_TIMER_IF_H_

enum tim_pwm_id_et
{
	TIM_PWM_ID_1_E,    /* for Motor1 Pwm Control     */
	TIM_PWM_ID_2_E,    /* for Motor2 Pwm Control     */
	TIM_PWM_ID_3_E,    /* for Motor3 Pwm Control     */
	TIM_PWM_ID_4_E,    /* for Motor4 Pwm Control     */
	TIM_PWM_ID_5_E,    /* for Head-light Pwm Control */
	TIM_PWM_ID_6_E,
	TIM_PWM_ID_NUM_E
};

void timer_ifF_Init(void);
void timer_ifF_ConfigPwmChannel(enum tim_pwm_id_et tim_id_e);
void timer_ifF_StartPwm(enum tim_pwm_id_et tim_id_e);
void timer_ifF_StopPwm(enum tim_pwm_id_et tim_id_e);
void timer_ifF_UpdatePwm(uint32_t duty_cycle_percentage, enum tim_pwm_id_et tim_id_e);
uint32_t timer_ifF_getEncoderCount(void);
void timer_ifF_ReadEncoderCounts(void);
uint32_t timer_ifF_getMotorDirection(void);

#endif /* TIMER_IF_INC_TIMER_IF_H_ */
