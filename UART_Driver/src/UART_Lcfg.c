/*
 * UART_Lcfg.c
 *
 *  Created on: Jan 16, 2021
 *      Author: mas
 */
#include "UART_Lcfg.h"


 USART_Config_t UART_ConfigArray[ NUMBER_OF_CONFIGURED_UART]=
		 /*	USART_ID   USART_Mode         USART_Baud         USART_NoOfStopBits    USART_WordLength   USART_ParityControl     USART_HWFlowControl */
 {
		 {USART1_,   USART_MODE_TXRX,   USART_STD_BAUD_9600,   USART_STOPBITS_1,  USART_WORDLEN_8BITS,  USART_PARITY_DISABLE,  USART_HW_FLOW_CTRL_NONE},

 };
