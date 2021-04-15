/*
 * ADC.h
 *
 *  Created on: Jan 17, 2021
 *      Author: Toqa & Ghada
 */

#ifndef ADC_H_
#define ADC_H_
#include "stm32f4xxx.h"


/* Interrupt sources */
//HA review: should be constant without user control
#define ADC_IT_EOC         ((uint8_t)0x05)

typedef enum
{
	NOT_OK,
	BUSY,
	OK
}ADC_CONVERSION_STATUS;

void ADC_Init(void);
//void ADC_RegularChannelConfig(uint8_t ADC_ID, uint8_t ch_num, uint8_t rank, uint8_t sample_time); //HA Review: commented code not allowed
void ADC_RegularChannelConfig(void);
void ADC_SoftwareStartConv(uint8_t ADC_ID);
void ADC_EOCOnEachRegularChannelCmd(uint8_t ADC_ID, uint8_t State);

void ADC_IntControl(uint8_t ADC_ID , uint8_t IntSource , uint8_t State);
//HA Review: To be part of the ADC_IntControl actions
void UART_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi);

uint16_t ADC_getValue(uint8_t Ch_Num);

#endif /* ADC_H_ */
