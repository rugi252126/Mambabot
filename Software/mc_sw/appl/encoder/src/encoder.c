/*
 * encoder.c
 *
 *  Created on: 9 May 2021
 *      Author: Alfonso, Rudy Manalo
 */

/* Standard includes. */
#include <string.h>
/* Project includes. */
#include "stm32f4xx_hal.h"
#include "project.h"
#include "timer_if.h"
#include "encoder.h"

/***** Macros */
#define COUNT_TO_MILLISECONDS_K      (uint32_t)(1000)
#define MILLISECONDS_TO_MINUTES_K    (int32_t)(60000)
#define MAX_ENCODER_COUNT_K          (uint32_t)(65535)
#define MIN_ENCODER_COUNT_K          (uint32_t)(0)


/***** Variables */
struct time_st
{
    uint32_t xMs_time_ctr;      /* counter is incremented every 1ms */
    uint32_t current_time;
    uint32_t previous_time;
};

/* the counts will increment/decrement by 4 as per the timer encoder configuration */
struct counts_st
{
    uint32_t current_cnt;
    uint32_t previous_cnt;
    uint32_t delta_cnt;
    int32_t  overtime_cnt;
};

struct encoder_st
{
    uint32_t         rpm[MOTOR_NUM_ID_K];
    struct time_st   time_s;
    struct counts_st counts_s[MOTOR_NUM_ID_K];
};

static struct encoder_st  encoder_s;


/***** Local function prototypes */
static void encoderLF_processEncoderCounts(uint8_t mot_id, uint32_t dir);

/***** Local functions */

/** Function to process the read encoder counts from the timer before application
    can use it. Counts overflow will be taken care too.

    @param  mot_id   {[0..MOTOR_NUM_ID_K] corresponding motor encoder ID}
            dir      {[0..1] encoder count direction. 0=up-count; 1=down-count }
    @return none
 */
static void encoderLF_processEncoderCounts(uint8_t mot_id, uint32_t dir)
{
    uint32_t tmp_cnt = 0u;

    if(1u == dir)
    { /* Counting down */
        /* check for overflow */
        if(encoder_s.counts_s[mot_id].previous_cnt < encoder_s.counts_s[mot_id].current_cnt)
        {
            /* overflow occurred. do the correction */
            tmp_cnt = (MAX_ENCODER_COUNT_K - encoder_s.counts_s[mot_id].current_cnt) + encoder_s.counts_s[mot_id].previous_cnt;
            encoder_s.counts_s[mot_id].delta_cnt = tmp_cnt;
        }
        else
        {
            encoder_s.counts_s[mot_id].delta_cnt = encoder_s.counts_s[mot_id].previous_cnt - encoder_s.counts_s[mot_id].current_cnt;
        }

        /* save a copy of current encoder count */
        encoder_s.counts_s[mot_id].previous_cnt = encoder_s.counts_s[mot_id].current_cnt;

        /* register the counts over the period of time */
        encoder_s.counts_s[mot_id].overtime_cnt -= encoder_s.counts_s[mot_id].delta_cnt;
    }
    else
    { /* Counting up */
        /* check for overflow */
        if(encoder_s.counts_s[mot_id].previous_cnt > encoder_s.counts_s[mot_id].current_cnt)
        {
            /* overflow occurred. do the correction */
            tmp_cnt = (MAX_ENCODER_COUNT_K - encoder_s.counts_s[mot_id].previous_cnt) + encoder_s.counts_s[mot_id].current_cnt;
            encoder_s.counts_s[mot_id].delta_cnt = tmp_cnt;
        }
        else
        {
            encoder_s.counts_s[mot_id].delta_cnt = encoder_s.counts_s[mot_id].current_cnt - encoder_s.counts_s[mot_id].previous_cnt;
        }

        /* save a copy of current encoder count */
        encoder_s.counts_s[mot_id].previous_cnt = encoder_s.counts_s[mot_id].current_cnt;

        /* register the counts over the period of time */
        encoder_s.counts_s[mot_id].overtime_cnt += encoder_s.counts_s[mot_id].delta_cnt;
    }
}


/***** Global functions */

/** Module initialization

    @param  none
    @return none

    It will be called once every POR.
 */
void encoderF_Init(void)
{
    (void)memset(&encoder_s, 0x00, sizeof(encoder_s));
}

/** Function to return the current motor RPM
    RPM is calculated based on encoder information and with respect
    to elapsed time.

    @param mot_id   {[0..MOTOR_NUM_ID_K] corresponding motor encoder ID}
    @return motor RPM
 */
int32_t encoderF_getRPM(uint8_t mot_id)
{
    /* get the elapsed time */
    uint32_t dT = encoder_s.time_s.current_time - encoder_s.time_s.previous_time;

    /* convert milliseconds to minutes */
    double dtm = (double)dT / MILLISECONDS_TO_MINUTES_K;

    /* save a copy of current time */
    encoder_s.time_s.previous_time = encoder_s.time_s.current_time;

    // calculate wheel's speed (in RPM) and return the value
    return (((double)encoder_s.counts_s[mot_id].delta_cnt / (int32_t)MOTOR_ENCODER_COUNTS_PER_REVOLUTION_K) / dtm);
}

/** Function to return the registered encoder count over the period of time.
    The information is useful for debugging and monitoring of encoder behavior
    as in-line with motor movements.

    @param  param mot_id   {[0..MOTOR_NUM_ID_K] corresponding motor encoder ID}
    @return encoder counts
 */
int32_t encoderF_getEncoderCountsOvertime(uint8_t mot_id)
{
    return (encoder_s.counts_s[mot_id].overtime_cnt);
}

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
void encoderF_ReadCounts_Callback(void)
{
    uint8_t idx = 0u;
    uint32_t cnt_dir = 0u;

    /* Overflow will occur approximately around 50 days on continuous operation */
    encoder_s.time_s.xMs_time_ctr++;
    /* convert count to millisecond (e.g. 1 = 1000ms */
    encoder_s.time_s.current_time = encoder_s.time_s.xMs_time_ctr * COUNT_TO_MILLISECONDS_K;

    /* Read the current encoder counts and direction */
    for(idx=0u; idx < (uint8_t)MOTOR_NUM_ID_K; idx++)
    {
        encoder_s.counts_s[idx].current_cnt = timer_ifF_getEncoderCount(idx);
        cnt_dir = timer_ifF_getEncoderCountDirection(idx);

        /* encoder plausibility check */
        if(    (MAX_ENCODER_COUNT_K == encoder_s.counts_s[idx].current_cnt )
            && (MIN_ENCODER_COUNT_K == encoder_s.counts_s[idx].previous_cnt)
        )
        {
            /* no further action. no pulses generated from encoder
               having MIN_ENCODER_COUNT_K and MAX_ENCODER_COUNT_K value won't happen unless encoder is not generating any pulses
            */
        }
        else
        {
            encoderLF_processEncoderCounts(idx, cnt_dir);
        }

    }
}
