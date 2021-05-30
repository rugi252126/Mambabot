/*
 * ros_if.h
 *
 *  Created on: 8 May 2021
 *      Author: Alfonso, Rudy Manalo
 */

#ifndef INC_ROS_IF_H_
#define INC_ROS_IF_H_

/** Module initialization

    @param  none
    @return none

    It will be called once every POR.
 */
void ros_ifF_Init();

/** ROS interface cyclic function

    @param  none
    @return none

    It will be called cyclically every ROS_IF_TASK_MS_K.
 */
void ros_ifF_Cyclic();

/** Callback function every time USART3_IRQHandler interrupt is called.
    The data from USART3 data register will be copied to the ringbuffer.

    @param  none
    @return none
 */
void ros_ifF_USART_Rx_Callback();

#if defined(DEBUG_OVER_ROS_USED)
/** Test function(interface). Used to send data over rosserial for debugging purposes.

    @param  none
    @return none
 */
void ros_ifF_TestAndDebug();
#endif // DEBUG_OVER_ROS_USED

#ifdef __cplusplus
extern "C"{
#endif


#ifdef __cplusplus
}
#endif


#endif /* INC_ROS_IF_H_ */
