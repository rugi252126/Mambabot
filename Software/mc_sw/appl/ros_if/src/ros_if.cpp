/*
 * ros_if.cpp
 *
 *  Created on: 8 May 2021
 *      Author: Alfonso, Rudy Manalo
 */


#include "main.h"
#include "stm32f4xx_hal.h"
#include "ringbuffer.h"
#include "encoder.h"

#include "ros.h"
#include "std_msgs/String.h"

#define ROS_CHATTER_USED

extern UART_HandleTypeDef huart3;

const static uint16_t rbuflen = 1024;
uint8_t RxBuffer[rbuflen];
struct ringbuffer rb;

ros::NodeHandle nh;
std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

static void ros_ifLF_TestAndDebug(void);

void ros_topic_senderCallback(const std_msgs::String& msg);
ros::Subscriber<std_msgs::String> listen("ros_topic_sender", ros_topic_senderCallback);

char hello[] = "STM32 to HOST PC Test";

void ros_topic_senderCallback(const std_msgs::String& msg)
{
	//HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
}

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

//void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
//	//HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
//  //nh.getHardware()->flush();
//}
//
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
//	//HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
//  //nh.getHardware()->reset_rbuf();
//}

extern "C" void ros_ifF_USART_Rx_Callback()
{
	//HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

	ringbuffer_putchar(&rb, huart3.Instance->DR);
}

extern "C" void ros_ifF_Init()
{
	ringbuffer_init(&rb, RxBuffer, rbuflen);
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);

	// Initialize ROS
	nh.initNode();

	nh.advertise(chatter);

	nh.subscribe(listen);
}

extern "C" void ros_ifF_Loop()
{
	//HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

#ifdef ROS_CHATTER_USED
	str_msg.data = hello;
	chatter.publish(&str_msg);
#endif

	nh.spinOnce();

#ifdef ROS_CHATTER_USED
	//HAL_GPIO_TogglePin(LD2_GPIO_Port, GPIO_PIN_0);
	//HAL_Delay(50);

#endif


}

extern "C" void ros_ifF_TestAndDebug(void)
{
    ros_ifLF_TestAndDebug();
}


