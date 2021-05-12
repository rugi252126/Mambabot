/*
 * timer_if.h
 *
 *  Created on: 1 May 2021
 *      Author: Alfonso, Rudy Manalo
 */

#ifndef TIMER_IF_INC_TIMER_IF_H_
#define TIMER_IF_INC_TIMER_IF_H_

/** Module initialization

    @param  none
    @return none

    It will be called once every POR.
 */
void timer_ifF_Init(void);

/** Function interface to configure pwm channels

    @param  idx      {[0..n] pwm channel Id}
    @return none
 */
void timer_ifF_ConfigPwmChannel(uint8_t idx);

/** Function interface to start pwm generation

    @param  idx      {[0..n] pwm channel Id}
    @return none
 */
void timer_ifF_StartPwm(uint8_t idx);

/** Function interface to stop pwm generation

    @param  idx      {[0..n] pwm channel Id}
    @return none
 */
void timer_ifF_StopPwm(uint8_t idx);

/** Function interface to update the pwm duty cycle

    @param  idx      {[0..n] pwm channel Id}
    @return none
 */
void timer_ifF_UpdatePwm(uint8_t idx, uint8_t duty_cycle_percentage);

/** Function interface to return back the encoder counts captured by timer

    @param  idx              {[0..n] pwm channel Id}
    @return encoder counts
 */
uint32_t timer_ifF_getEncoderCount(uint8_t idx);

/** Function interface to return back the encoder counts direction captured by timer

    @param  idx              {[0..n] pwm channel Id}
    @return Count_up=0; Count_down=1
 */
uint32_t timer_ifF_getEncoderCountDirection(uint8_t idx);


#endif /* TIMER_IF_INC_TIMER_IF_H_ */
