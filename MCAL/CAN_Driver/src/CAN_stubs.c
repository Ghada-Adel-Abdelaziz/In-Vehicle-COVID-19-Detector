/*
 * CAN_stubs.c
 *
 *  Created on: Feb 27, 2021
 *      Author: Ghada & Toaa
 */

#include "GPIO.h"
#include "GPIO_Lcfg.h"
#include "CAN_stubs.h"
#include "CAN.h"


void LED1_ON(void)   // CAN TX complete
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

void LED2_ON(void)   // CAN RX complete
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

	CAN_wrMsg      (&(CAN_TxMsg));
}

