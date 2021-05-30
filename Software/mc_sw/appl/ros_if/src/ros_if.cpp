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
#include "motor.h"
#include "uart_if.h"
/* ROS includes */
#include "ros.h"
//#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h" // for subscribing to "cmd_vel" topic

/***** Macros */
#define ROS_IF_TASK_MS_K                 (uint8_t)(50)
#define ROS_IF_CMD_VEL_TIMEOUT_MS_K      (uint8_t)(500 / ROS_IF_TASK_MS_K)

/***** Variables */
const static uint16_t rbuflen = 1024;
uint8_t RxBuffer[rbuflen];
struct ringbuffer rb;

struct cmd_vel_st
{
    double rq_linear_vel_x;
    double rq_linear_vel_y;
    double rq_angular_vel_z;
};

struct ros_if_st
{
    uint8_t   cmd_vel_timeout;
    int16_t   linear_x;
};

static struct ros_if_st   ros_if_s;
static struct cmd_vel_st  cmd_vel_s;

// ROS callback function prototypes
void ros_ifLF_cmd_vel_Callback(const geometry_msgs::Twist &cmd_vel_msg);

// ROS publisher/subscriber declaration
ros::NodeHandle nh;

ros::Subscriber<geometry_msgs::Twist>cmd_vel_sub("cmd_vel", ros_ifLF_cmd_vel_Callback);

// local function prototypes
#if defined(DEBUG_OVER_ROS_USED)
static void ros_ifLF_TestAndDebug();
#endif // DEBUG_OVER_ROS_USED
static void ros_ifLF_checkCmdVel_Timeout();
static void ros_ifLF_resetCmdVel();
static void ros_ifLF_motorContorl();

/** Function to control the motor as per request.

    @param  none
    @return none
 */
static void ros_ifLF_motorContorl()
{
    // start - FOR TESTING ONLY
    double tmp_vel;
    tmp_vel = cmd_vel_s.rq_linear_vel_x * 100;
    ros_if_s.linear_x = (int16_t)tmp_vel;

    motorF_SetPwmAndDirection(0,ros_if_s.linear_x);
    motorF_SetPwmAndDirection(1,ros_if_s.linear_x);
    motorF_SetPwmAndDirection(2,ros_if_s.linear_x);
    motorF_SetPwmAndDirection(3,ros_if_s.linear_x);
    // end - FOR TESTING ONLY
}

/** Function to reset the command velocity variables if timeout is detected.

    @param  none
    @return none
 */
static void ros_ifLF_resetCmdVel()
{
    cmd_vel_s.rq_linear_vel_x = 0;
    cmd_vel_s.rq_linear_vel_y = 0;
    cmd_vel_s.rq_angular_vel_z = 0;
}

/** Function to monitor the command velocity activity.
    Timeout will be set if no command is received within a given time.

    @param  none
    @return none
 */
static void ros_ifLF_checkCmdVel_Timeout()
{
    if(ros_if_s.cmd_vel_timeout >= ROS_IF_CMD_VEL_TIMEOUT_MS_K - 1u)
    {
        ros_ifLF_resetCmdVel();
    }
    else
    {
        ros_if_s.cmd_vel_timeout++;
    }

}

#if defined(DEBUG_OVER_ROS_USED)
/** Test function. Used to send data over rosserial for debugging purposes.

    @param  none
    @return none
 */
static void ros_ifLF_TestAndDebug()
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
    sprintf (buffer, "Motor1 RPM  : %ld", encoderF_getRPM(0));
    nh.loginfo(buffer);

    sprintf (buffer, "Motor2 RPM  : %ld", encoderF_getRPM(1));
    nh.loginfo(buffer);

    sprintf (buffer, "Motor3 RPM  : %ld", encoderF_getRPM(2));
    nh.loginfo(buffer);

    sprintf (buffer, "Motor4 RPM  : %ld", encoderF_getRPM(3));
    nh.loginfo(buffer);
#endif

    sprintf (buffer, "Linear Vel X  : %f", cmd_vel_s.rq_linear_vel_x);
    nh.loginfo(buffer);
    sprintf (buffer, "Linear Vel Y : %d", ros_if_s.linear_x);
    nh.loginfo(buffer);
    sprintf (buffer, "Angular Vel Z : %f", cmd_vel_s.rq_angular_vel_z);
    nh.loginfo(buffer);
}
#endif // DEBUG_OVER_ROS_USED

/** Callback function every time linear and angular speed is received from "cmd_vel" topic.

    @param  cmd_vel_msg   {see geometry_msgs::Twist}
    @return none
 */
void ros_ifLF_cmd_vel_Callback(const geometry_msgs::Twist &cmd_vel_msg)
{
    // this callback function receives "cmd_vel_msg" object where linear and angular speed are stored
    cmd_vel_s.rq_linear_vel_x = cmd_vel_msg.linear.x;
    cmd_vel_s.rq_linear_vel_y = cmd_vel_msg.linear.y;
    cmd_vel_s.rq_angular_vel_z = cmd_vel_msg.angular.z;

    // reset the timeout variable
    ros_if_s.cmd_vel_timeout = 0u;
}

/** Callback function every time USART3_IRQHandler interrupt is called.
    The data from USART3 data register will be copied to the ringbuffer.

    @param  none
    @return none
 */
extern "C" void ros_ifF_USART_Rx_Callback()
{
    ringbuffer_putchar(&rb, huart3.Instance->DR);
}

/** Module initialization

    @param  none
    @return none

    It will be called once every POR.
 */
extern "C" void ros_ifF_Init()
{
	ringbuffer_init(&rb, RxBuffer, rbuflen);

	// enable interrupt
	uart_ifF_EnableInterrupt_USART3();

	// Initialize ROS
	nh.initNode();

    // initialize publisher
	//nh.advertise(vel_pub);

    // initialize subscriber
	nh.subscribe(cmd_vel_sub);

//	// wait until connection is established
//    while (!nh.connected())
//    {
//        nh.spinOnce();
//    }
//    nh.loginfo("ROBOT BASE IS CONNECTED");
}

/** ROS interface cyclic function

    @param  none
    @return none

    It will be called cyclically every ROS_IF_TASK_MS_K.
 */
extern "C" void ros_ifF_Cyclic()
{
    ros_ifLF_checkCmdVel_Timeout();

    ros_ifLF_motorContorl();

    // call to process all pending callbacks
	nh.spinOnce();
}

#if defined(DEBUG_OVER_ROS_USED)
/** Test function(interface). Used to send data over rosserial for debugging purposes.

    @param  none
    @return none
 */
extern "C" void ros_ifF_TestAndDebug()
{
    ros_ifLF_TestAndDebug();
}
#endif // DEBUG_OVER_ROS_USED


