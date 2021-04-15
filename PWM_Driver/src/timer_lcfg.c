/*
 * timer_lcfg.c
 *
 *  Created on: Feb 27, 2021
 *      Author: Toqa & Ghada
 */


#include "timer_Cfg.h"
#include "timer.h"
#include "timer_Lcfg.h"

void LED1_ON(void)   // TIMER2_ counting complete
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

void LED2_ON(void)   // TIMER3_ counting complete
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

}

TIM_Config_t TIM_ConfigArray[NUMBER_OF_CONFIGURED_TIMER] =
{
		{TIMER2_, 83/*41999*/, TIM_CounterMode_Up, 2000, TIM_CKD_DIV1, LED1_ON},
		{TIMER3_, 83/*41999*/, TIM_CounterMode_Up, 1000, TIM_CKD_DIV1, LED2_ON},
};


TIM_OC_Config_t OC_ConfigArray[NUMBER_OF_CONFIGURED_CHANNEL] =
{
		{CH1_ID, TIMER2_, TIM_OCMode_PWM1, 1000, TIM_OCPolarity_Low, TIM_OutputState_Enable}
};

/* Timer2 connected in APB1 bus which its clock is 42MHz
 * back to page 152,166,208 in data sheet and regarding to
 * RCC configurations the timer clock is equal
 * to the bus clock to which it is connected multiply by 2*/

/* So TIM2 CLK is 84Mhz
 * thus PSC = 41,999 to get a tick time = 0.5ms
 * 84000000 / (PSC[44999]+1) = 2000 Hz = 2khz
 * 2khz is the final frequency of the timer
 * so put in the auto reload reg 2000 to get 1 sec period
 * */


// 83 to get Ttick 1 micro sec.
