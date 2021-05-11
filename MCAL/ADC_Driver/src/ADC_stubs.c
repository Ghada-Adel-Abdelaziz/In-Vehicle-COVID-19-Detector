/*
 * ADC_stubs.c
 *
 *  Created on: Feb 27, 2021
 *      Author: Ghada & Toaa
 */


#include "GPIO.h"
#include "GPIO_Lcfg.h"

void LED_ON(void)   // ADC Conversion complete
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
