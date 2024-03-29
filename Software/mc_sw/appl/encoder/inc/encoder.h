/*
 * encoder.h
 *
 *  Created on: 9 May 2021
 *      Author: Alfonso, Rudy Manalo
 */

#ifndef ENCODER_INC_ENCODER_H_
#define ENCODER_INC_ENCODER_H_

/** Module initialization

    @param  none
    @return none

    It will be called once every POR.
 */
void encoderF_Init(void);

#ifdef __cplusplus
extern "C"{
#endif
/** Function to return the current motor RPM
    RPM is calculated based on encoder information and with respect
    to elapsed time.

    @param mot_id   {[0..MOTOR_NUM_ID_K] corresponding motor encoder ID}
    @return motor RPM
 */
int32_t encoderF_getRPM(uint8_t mot_id);

/** Function to return the registered encoder count over the period of time.
    The information is useful for debugging and monitoring of encoder behavior
    as in-line with motor movements.

    @param  param mot_id   {[0..MOTOR_NUM_ID_K] corresponding motor encoder ID}
    @return encoder counts
 */
int32_t encoderF_getEncoderCountsOvertime(uint8_t mot_id);
#ifdef __cplusplus
}
#endif

/** Function reads the encoder counts and direction from the timer interface module
    The function will be called in every timer interrupt.
    The timer is configured as a free running timer and interrupt is constantly
    called in a given time.

    @param  none
    @return none

    Make sure the needed time to execute this function won't exceed the
    configured interrupt interval time.
    Ideal time should be half of the configured interrupt time for better
    performance and best results. (actual measured execution time: 7uS)
    example: having interrupt every 1ms, processing time should not exceed 0.5ms
 */
void encoderF_ReadCounts_Callback(void);

#endif /* ENCODER_INC_ENCODER_H_ */
