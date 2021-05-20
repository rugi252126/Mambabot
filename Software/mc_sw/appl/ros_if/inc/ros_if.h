/*
 * ros_if.h
 *
 *  Created on: 8 May 2021
 *      Author: Alfonso, Rudy Manalo
 */

#ifndef INC_ROS_IF_H_
#define INC_ROS_IF_H_

void ros_ifF_Init();
void ros_ifF_Loop();
void ros_ifF_TestAndDebug(void);
void ros_ifF_USART_Rx_Callback();

#ifdef __cplusplus
extern "C"{
#endif


#ifdef __cplusplus
}
#endif


#endif /* INC_ROS_IF_H_ */
