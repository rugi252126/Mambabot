/*
 * project.h
 *
 *  Created on: 9 May 2021
 *      Author: Alfonso, Rudy Manalo
 */

#ifndef PROJECT_H_
#define PROJECT_H_

#include "stm32f4xx_hal.h"
#include "stm32f429xx.h"

/* comment-out if debug port is not used */
#define DEBUG_PORT_USED

#define VAR_4WD                                (uint8_t)(0xAA)

#define PROJECT_VARIANT_USED                    VAR_4WD
#define MOTOR_ENCODER_COUNTS_PER_REVOLUTION_K   930


/* motor Id */
#define MOTOR_ID_1_K                            (uint8_t)(0)
#define MOTOR_ID_2_K                            (uint8_t)(1)
#define MOTOR_ID_3_K                            (uint8_t)(2)
#define MOTOR_ID_4_K                            (uint8_t)(3)
#define MOTOR_NUM_ID_K                          (uint8_t)(4)




#endif /* PROJECT_H_ */
