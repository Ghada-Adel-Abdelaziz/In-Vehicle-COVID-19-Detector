/*
 * Buzzer_stubs.c
 *
 *  Created on: Feb 27, 2021
 *      Author: Ghada & Toaa
 */


#include "GPIO.h"
#include "GPIO_Lcfg.h"

void Buzzer_Cycle_Complete_Callback(void)
{
	//GPIO_WriteOutputPin(BLUE_LED,1);

	static uint8_t counter = 0;

	counter ++;

	if(counter == 1)
	{
		Buzzer_Req(  2 ,  1 ,  1 ,   MEDIUM_POWER);
	}
	else if(counter == 2)
	{
		Buzzer_Req(  2,  1 ,  1 ,   MAXIMUM_POWER);
	}

}

