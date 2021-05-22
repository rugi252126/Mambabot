/*
 * ros_if.cpp
 *
 *  Created on: 8 May 2021
 *      Author: Alfonso, Rudy Manalo
 */

/* Project includes. */
#include "stm32f4xx_hal.h"
#include "ringbuffer.h"
#include "project.h"
#include "encoder.h"
#include "uart_if.h"
/* ROS includes */
#include "ros.h"
#include "std_msgs/String.h"

#define ROS_CHATTER_USED

const static uint16_t rbuflen = 1024;
uint8_t RxBuffer[rbuflen];
struct ringbuffer rb;

ros::NodeHandle nh;
std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

#if defined(DEBUG_OVER_ROS_USED)
static void ros_ifLF_TestAndDebug(void);
#endif // DEBUG_OVER_ROS_USED

void ros_topic_senderCallback(const std_msgs::String& msg);
ros::Subscriber<std_msgs::String> listen("ros_topic_sender", ros_topic_senderCallback);

char hello[] = "STM32 to HOST PC Test";

void ros_topic_senderCallback(const std_msgs::String& msg)
{
	//HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
}

#if defined(DEBUG_OVER_ROS_USED)
static void ros_ifLF_TestAndDebug(void)
{

    char buffer[50];

#if 0

    sprintf (buffer, "Motor1 Encoder  : %ld", encoderF_getEncoderCountsOvertime(0));
    nh.loginfo(buffer);

    sprintf (buffer, "Motor2 Encoder  : %ld", encoderF_getEncoderCountsOvertime(1));
    nh.loginfo(buffer);

    sprintf (buffer, "Motor3 Encoder  : %ld", encoderF_getEncoderCountsOvertime(2));
    nh.loginfo(buffer);

    sprintf (buffer, "Motor4 Encoder  : %ld", encoderF_getEncoderCountsOvertime(3));
    nh.loginfo(buffer);
#else
    sprintf (buffer, "Motor1 Encoder  : %ld", encoderF_getRPM(0));
    nh.loginfo(buffer);

    sprintf (buffer, "Motor2 Encoder  : %ld", encoderF_getRPM(1));
    nh.loginfo(buffer);

    sprintf (buffer, "Motor3 Encoder  : %ld", encoderF_getRPM(2));
    nh.loginfo(buffer);

    sprintf (buffer, "Motor4 Encoder  : %ld", encoderF_getRPM(3));
    nh.loginfo(buffer);
#endif
}
#endif // DEBUG_OVER_ROS_USED

extern "C" void ros_ifF_USART_Rx_Callback()
{
	ringbuffer_putchar(&rb, huart3.Instance->DR);
}

extern "C" void ros_ifF_Init()
{
	ringbuffer_init(&rb, RxBuffer, rbuflen);

	// enable interrupt
	uart_ifF_EnableInterrupt_USART3();

	// Initialize ROS
	nh.initNode();

	nh.advertise(chatter);

	nh.subscribe(listen);
}

extern "C" void ros_ifF_Cyclic()
{
#ifdef ROS_CHATTER_USED
	str_msg.data = hello;
	chatter.publish(&str_msg);
#endif

	nh.spinOnce();
}

#if defined(DEBUG_OVER_ROS_USED)
extern "C" void ros_ifF_TestAndDebug(void)
{
    ros_ifLF_TestAndDebug();
}
#endif // DEBUG_OVER_ROS_USED


