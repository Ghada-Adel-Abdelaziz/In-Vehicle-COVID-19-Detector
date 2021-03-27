/*
 * timer.h
 *
 *  Created on: Feb 27, 2021
 *      Author: esraa
 */

#ifndef TIMER_H_
#define TIMER_H_

#include "stm32f4xxx.h"

/* Interrupt sources */
#define TIM_UPDATE_EVENT_IE       ((uint8_t)0x00)


//typedef enum
//{
//	FLAG_RESET,
//	FLAG_SET
//}TIM_FLAG_STATUS;

typedef enum
{
	STOP,
	START
}TIMER_STATE;

void Timer_Init();
void Timer_SetCounter(uint8_t TIM_ID, uint32_t Counter);
void Timer_SetAutoreload(uint8_t TIM_ID, uint32_t Autoreload);
void Timer_Cmd(uint8_t TIM_ID, TIMER_STATE cmd);

void TIM_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi);
void TIM_IntControl(uint8_t TIM_ID , uint8_t IntSource , uint8_t State);


void TIM_OC_Init(void);
void CC1R_SetValue(uint8_t TIM_ID, uint32_t Counter);



#endif /* TIMER_H_ */
