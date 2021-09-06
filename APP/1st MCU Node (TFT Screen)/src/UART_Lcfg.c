/*
 * UART_Lcfg.c
 *
 *  Created on: Jan 16, 2021
 *      Author: Ghada & Toqa
 */
#include "UART_Lcfg.h"
#include "GPIO_Lcfg.h"
#include "GPIO_cfg.h"
#include "Mode_manager.h"
#include "TFT.h"


extern void Communication_Manager_TX_Notification(void);

extern void Communication_Manager_RX_Notification(void);
//extern void LED1_ON(void);
//extern void LED2_ON(void);
extern char RX_Buffer[20];

void CAN_RX_Callback(void);

void LED1_ON(void)   // UART TX complete
{
}

void LED2_ON(void)   // UART RX complete
{

}

void LED1_ON2(void)   // UART TX complete
{

}

void UART_RX_Callback(void)   // UART RX complete
{
	switch (RX_Buffer[0])
	{
	case Camera_ID:
		CAM_SetMaskState (RX_Buffer[1]);
		break;

	case Cough_ID:
		Jetson_Set_COVID_State (RX_Buffer[1]);

		break;
	case API_ID:
		Jetson_Set_API_State (RX_Buffer);

		break;

	default:
		break;
	}

}
USART_Config_t UART_ConfigArray[ NUMBER_OF_CONFIGURED_UART]=

		/*	USART_ID   USART_Mode         USART_Baud         USART_NoOfStopBits    USART_WordLength   USART_ParityControl     USART_HWFlowControl */
{
		//{USART2_,   USART_MODE_ONLY_TX,   USART_STD_BAUD_9600,   USART_STOPBITS_1,  USART_WORDLEN_8BITS,  USART_PARITY_DISABLE,  USART_HW_FLOW_CTRL_NONE},
		// {USART2_,   USART_MODE_TXRX,   USART_STD_BAUD_9600,   USART_STOPBITS_1,  USART_WORDLEN_8BITS,  USART_PARITY_DISABLE,  USART_HW_FLOW_CTRL_NONE, LED1_ON2, LED2_ON2},
		{USART2_,   USART_MODE_TXRX,   USART_STD_BAUD_9600,   USART_STOPBITS_1,  USART_WORDLEN_8BITS,  USART_PARITY_DISABLE,  USART_HW_FLOW_CTRL_NONE, LED1_ON2, Communication_Manager_RX_Notification},
		{USART3_,   USART_MODE_TXRX,   USART_STD_BAUD_115200,   USART_STOPBITS_1,  USART_WORDLEN_8BITS,  USART_PARITY_DISABLE,  USART_HW_FLOW_CTRL_NONE, TFT_Writing_Complete_CallBack, TFT_Reading_Complete_CallBack},
		{USART4_,   USART_MODE_TXRX,   USART_STD_BAUD_115200,   USART_STOPBITS_1,  USART_WORDLEN_8BITS,  USART_PARITY_DISABLE,  USART_HW_FLOW_CTRL_NONE, LED1_ON2, CAN_RX_Callback},

};
