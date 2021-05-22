/*
 * uart_if.h
 *
 *  Created on: 22 May 2021
 *      Author: Alfonso, Rudy Manalo
 */

#ifndef HAL_IF_INC_UART_IF_H_
#define HAL_IF_INC_UART_IF_H_

/** USART3 initialization

    @param  none
    @return none

    It will be called once every POR.
 */
void uart_ifF_Init_USART3_UART(void);

#ifdef __cplusplus
extern "C"{
#endif
/** Function to enable USART3 interrupt

    @param  none
    @return none
 */
void uart_ifF_EnableInterrupt_USART3(void);

/** Function to disable USART3 interrupt

    @param  none
    @return none
 */
void uart_ifF_DisableInterrupt_USART3(void);
#ifdef __cplusplus
}
#endif

#endif /* HAL_IF_INC_UART_IF_H_ */
