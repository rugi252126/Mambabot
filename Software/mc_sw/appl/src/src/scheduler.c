/*
 * scheduler.c
 *
 *  Created on: 22 May 2021
 *      Author: Alfonso, Rudy Manalo
 */

/* Project includes. */
/* freeRTOS includes */
#include "FreeRTOS.h" /* freeRTOS main header file */
#include "task.h"     /* in able to create a task  */
/* application includes */
#include "project.h"
#include "ros_if.h"
#include "motor.h"
#if defined(DEBUG_PORT_USED)
#include "gpio_if.h" // FOR TESTING ONLY to check the tasks using oscilloscope
#endif

/***** Macros */
/* in decimal form (e.g. 50ms = 50) */
#define SCHEDULER_50MS_TASK_DELAY_K                 50
#define SCHEDULER_100MS_TASK_DELAY_K                100
#define SCHEDULER_1S_TASK_DELAY_K                   1000

/***** Private Variables */
/* create task handler variables */
TaskHandle_t xTask50msHandle=NULL;
TaskHandle_t xTask100msHandle=NULL;
TaskHandle_t xTask1sHandle=NULL;


/***** Local function prototypes */
/* task functions prototype */
static void schedulerLF_vTask50ms_handler(void *params);
static void schedulerLF_vTask100ms_handler(void *params);
static void schedulerLF_vTask1s_handler(void *params);

/***** Local functions */

static void TASK_TEST(void)
{
    static uint32_t ctr = 0u;

    ctr++;
    if(5u == ctr)
    {
        motorF_SetPwmAndDirection(0, 100);
        motorF_SetPwmAndDirection(1, 100);
        motorF_SetPwmAndDirection(2, 100);
        motorF_SetPwmAndDirection(3, 100);
    }
    else if(15u == ctr)
    {
        motorF_SetPwmAndDirection(0, 0);
        motorF_SetPwmAndDirection(1, 0);
        motorF_SetPwmAndDirection(2, 0);
        motorF_SetPwmAndDirection(3, 0);
    }
    else if(20u == ctr)
    {
        motorF_SetPwmAndDirection(0, -100);
        motorF_SetPwmAndDirection(1, -100);
        motorF_SetPwmAndDirection(2, -100);
        motorF_SetPwmAndDirection(3, -100);
    }
    else if(30u == ctr)
    {
        motorF_SetPwmAndDirection(0, 0);
        motorF_SetPwmAndDirection(1, 0);
        motorF_SetPwmAndDirection(2, 0);
        motorF_SetPwmAndDirection(3, 0);
    }
    else
    {

    }
}

/** 50ms Task handler/implementation

    @param  params { pointer parameter }
    @return none
 */
static void schedulerLF_vTask50ms_handler(void *params)
{
    while(1)
    {
#if defined(DEBUG_PORT_USED)
        /* FOR TESTING ONLY */
        //gpio_ifF_toggleDebugPort(0);
#endif

        /* motor main cyclic function */
        motorF_StateMachine();

        /* ROS main cyclic function */
        ros_ifF_Cyclic();

        /* convert ms to tick and send to vTaskDelay */
        vTaskDelay(pdMS_TO_TICKS(SCHEDULER_50MS_TASK_DELAY_K));
    }
}

/** 100ms Task handler/implementation

    @param  params { pointer parameter }
    @return none
 */
static void schedulerLF_vTask100ms_handler(void *params)
{
    while(1)
    {
#if defined(DEBUG_PORT_USED)
        /* FOR TESTING ONLY */
        gpio_ifF_toggleDebugPort(1);
#endif

        /* read ROS request */

        /* convert ms to tick and send to vTaskDelay */
        vTaskDelay(pdMS_TO_TICKS(SCHEDULER_100MS_TASK_DELAY_K));
    }
}

/** 1second Task handler/implementation

    @param  params { pointer parameter }
    @return none
 */
static void schedulerLF_vTask1s_handler(void *params)
{
    while(1)
    {
#if defined(DEBUG_PORT_USED)
        /* FOR TESTING ONLY */
        gpio_ifF_toggleDebugPort(0);
#endif

#if defined(DEBUG_OVER_ROS_USED)
        ros_ifF_TestAndDebug();
#endif // DEBUG_OVER_ROS_USED

        /* set motor request and direction */
        TASK_TEST(); //motorF_SetPwmAndDirection(0,0);

        /* convert ms to tick and send to vTaskDelay */
        vTaskDelay(pdMS_TO_TICKS(SCHEDULER_1S_TASK_DELAY_K));
    }
}

/***** Global functions */

/** Function to create task schedulers

    @param  none
    @return none
 */
void schedulerF_Task(void)
{
    BaseType_t tmp_stat;

    /** create tasks */
    /* 50ms task */
    tmp_stat = xTaskCreate( schedulerLF_vTask50ms_handler, /* task handler for implementation */
                 "TASK-50ms",                              /* task name */
                 500,                                      /* 500 words = 2KB (1word = 4Byte) */
                 NULL,                                     /* not sending any parameter */
                 2,                                        /* 0 priority is the lowest priority or the idle task: MAX priority is defined "configMAX_PRIORITIES" */
                 &xTask50msHandle );                       /* this is like ID number. can be used to delete or suspend the task */

    if(tmp_stat != pdPASS)
    {
        /* Latch on any failure/error. */
        configASSERT( 0 );
    }

    /* 100ms task */
    tmp_stat = xTaskCreate( schedulerLF_vTask100ms_handler, "TASK-100ms", 500, NULL, 2, &xTask100msHandle );
    if(tmp_stat != pdPASS)
    {
        /* Latch on any failure/error. */
        configASSERT( 0 );
    }
    /* 1s task */
    tmp_stat = xTaskCreate( schedulerLF_vTask1s_handler, "TASK-1s", 500, NULL, 2, &xTask1sHandle );
    if(tmp_stat != pdPASS)
    {
        /* Latch on any failure/error. */
        configASSERT( 0 );
    }

    /* start the scheduler. This will start executing the tasks that have been created above.
       this function will never return because it will give control to the task
     */
    vTaskStartScheduler();

    /* since "vTaskStartScheduler" will never return, this for loop will never be executed unless there is a problem. */
    for(;;);
}

/** FreeRTOS callback function for any stack overflow error

    @param  xTask      {Task handler}
            pcTaskName {Task name}
    @return none
 */
void vApplicationStackOverflowHook( TaskHandle_t xTask, char * pcTaskName )
{
    /* Latch on any failure/error. */
    configASSERT( 0 );
}

/** FreeRTOS callback function gets called for every systick timer interrupt.
    Current configuration the function is called every 1ms(default = 1000Hz).

    @param  none
    @return none
 */
void vApplicationTickHook( void )
{
    /* freeRTOS integration
       Since Systick timer handles by freeRTOS, call function "HAL_IncTick"
       here to update the "uwTick" variable. The value of "uwTick" is used
       by other module like ROS to manage some activities that is why it is
       important to keep its value updated.
     */
    HAL_IncTick();
}

/** FreeRTOS callback function gets called when no other task is ready to execute.
    On a completely unloaded system this is getting called at over 2.5MHz!

    @param  none
    @return none
 */
void vApplicationIdleHook( void )
{
    static uint64_t u64IdleTicksCnt = 0u;

    u64IdleTicksCnt++;
}

/** FreeRTOS callback function gets called if there is memory allocation

    @param  none
    @return none
 */
void vApplicationMallocFailedHook( void )
{
    /* Latch on any failure/error. */
    configASSERT( 0 );
}

