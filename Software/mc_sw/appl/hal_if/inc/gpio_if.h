/*
 * gpio_if.h
 *
 *  Created on: 12 May 2021
 *      Author: Alfonso, Rudy Manalo
 */

#ifndef HAL_IF_INC_GPIO_IF_H_
#define HAL_IF_INC_GPIO_IF_H_

/** Module initialization

    @param  none
    @return none

    It will be called once every POR.
 */
void gpio_ifF_Init(void);

/** Function to set the motor direction

    @param  mot_id   {[0..MOTOR_NUM_ID_K] motor ID}
            dir      {[0..2] motor direction      }
    @return none
 */
void gpio_ifF_setMotorDirection(uint8_t mot_id, uint8_t dir);

#if defined(DEBUG_PORT_USED)
/** Function to switch-on/off the debug port

    @param  debug_id    {[0..n] debug port Id}
            status      {[0..1] OFF/ON       }
    @return none

    It will be called once every POR.
 */
void gpio_ifF_setDebugPort(uint8_t debug_id, uint8_t status);

/** Function to toggle the debug port

    @param  debug_id    {[0..n] debug port Id}
    @return none

    It will be called once every POR.
 */
void gpio_ifF_toggleDebugPort(uint8_t debug_id);
#endif

#endif /* HAL_IF_INC_GPIO_IF_H_ */
