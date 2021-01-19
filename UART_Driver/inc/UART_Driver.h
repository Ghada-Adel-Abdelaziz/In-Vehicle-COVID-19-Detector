/*
 * stm32f407xx_uartdriver.h
 *
 *  Created on: 15-Jun-2020
 *      Author: Selva Kumar
 */

#ifndef INC_STM32F407XX_UARTDRIVER_H_
#define INC_STM32F407XX_UARTDRIVER_H_

#include "stm32f4xxx.h"

typedef enum
{
	_RESET_,
	_SET_
}FLAG_STATUS;

/******************************************************************************************
 *								APIs supported by this driver
 *		 For more information about the APIs check the function definitions
 ******************************************************************************************/
  
/*
 * Init and De-init
 */
void USART_Init(void);


/*
 * Data Send and Receive
 */
void USART_SendData(uint8_t USART_ID , uint8_t *pTxBuffer, uint32_t Len);
void  USART_ReceiveData(uint8_t USART_ID , uint8_t *pTxBuffer, uint32_t Len);
uint8_t USART_SendDataIT(uint8_t USART_ID , uint8_t *pTxBuffer, uint32_t Len);
uint8_t USART_ReceiveDataIT(uint8_t USART_ID , uint8_t *pTxBuffer, uint32_t Len);

/*
 * IRQ Configuration and ISR handling
 */
 /*
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void USART_IRQHandling(USART_Handle_t *pHandle);
*/

/*
 * Other Peripheral Control APIs
 */

void USART_ClearFlag(uint8_t USART_ID, uint16_t StatusFlagName);
void USART_SetBaudRate(uint8_t USART_ID, uint32_t BaudRate);

/*
 * Application callback
 */
 /*
void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t AppEv);
*/



#endif /* INC_STM32F407XX_UARTDRIVER_H_ */
