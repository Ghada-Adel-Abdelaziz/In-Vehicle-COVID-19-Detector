/******************************************************************************
 * Module:      UART
 * File Name:   UART_Lcfg.h
 * Description: Link time Configuration Source file for
 *              STM32F407 Microcontroller
 * Author:      Toqa&Ghada
 * Date:        16/1/2021
 ******************************************************************************/
#include "UART_Lcfg.h"
#include "GPIO_Lcfg.h"
extern char RX_Buffer[20];


void LED1_ON(void)   // UART TX complete
{
	static char i = 0;
	i++;

	if(i == 1)
	{
		GPIO_WriteOutputPin(ORANGE_LED,1);
	}
	else if(i == 2)
	{
		i = 0;
		GPIO_WriteOutputPin(ORANGE_LED,0);
	}

}

void LED2_ON(void)   // UART RX complete
{
	static char i = 0;
	i++;

	if(i == 1)
	{
		GPIO_WriteOutputPin(GREEN_LED,1);
	}
	else if(i == 2)
	{
		i = 0;
		GPIO_WriteOutputPin(GREEN_LED,0);
	}

	Uart_SendDataAsync(USART2_,RX_Buffer,sizeof(RX_Buffer));
}


 USART_Config_t UART_ConfigArray[ NUMBER_OF_CONFIGURED_UART]=
		 /*	USART_ID   USART_Mode         USART_Baud         USART_NoOfStopBits    USART_WordLength   USART_ParityControl     USART_HWFlowControl */
 {

		 {USART2_,   USART_MODE_TXRX,   USART_STD_BAUD_9600,   USART_STOPBITS_1,  USART_WORDLEN_8BITS,  USART_PARITY_DISABLE,  USART_HW_FLOW_CTRL_NONE, LED1_ON, LED2_ON},

 };
