/*
 * timer_lcfg.c
 *
 *  Created on: Feb 27, 2021
 *      Author: Ghada & Toaa
 */


#include "GPIO.h"
#include "GPIO_Lcfg.h"

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

