/******************************************************************************
 * Module:      UART
 * File Name:   UART.h
 * Description: Header file for UART Module on STM32F407 Microcontroller
 * Author:      Toqa & Ghada
 * Date:        16/1/2021
 ******************************************************************************/

#ifndef INC_STM32F407XX_UARTDRIVER_H_
#define INC_STM32F407XX_UARTDRIVER_H_

#include "stm32f4xxx.h"

typedef enum
{
	RESETTING,
	SETTING
}FLAG_STATUS;

typedef enum
{
	FALSE,
	TRUE

}Tx_or_Rx_Feedback;


#define UART_ERR_RX_NO_NEW_DATA                  (uint8_t)3

#define UART_E_OK                                (uint8_t)0
#define UART_E_NOT_OK                            (uint8_t)1



/********************* interrupt sources *************************/



#define UART_INT_PE                              ((uint8_t)0x08)
#define UART_INT_TXE                             ((uint8_t)0x07)
#define UART_INT_TC                              ((uint8_t)0x06)
#define UART_INT_RXNE                            ((uint8_t)0x05)
#define UART_INT_IDLE                            ((uint8_t)0x04)



/******************************  END  ****************************/


/******************************************************************************************
 *								APIs supported by this driver
 *		 For more information about the APIs check the function definitions
 ******************************************************************************************/


/* Init  */
void USART_Init(void);
void USART_SetBaudRate(uint8_t USART_ID, uint32_t BaudRate);
/*
 * Data Send and Receive
 */

/******************** NEW***************************/

uint8_t Uart_SendDataAsync(uint8_t Id , uint8_t* Data , uint16_t DataSize);
uint8_t Uart_ReceiveDataASync(uint8_t Id , uint8_t* Data);
void Uart_IntControl(uint8_t USART_ID , uint8_t IntSource , uint8_t State);
void UART_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi);


/*******************  END  *************************/

#endif /* INC_STM32F407XX_UARTDRIVER_H_ */
