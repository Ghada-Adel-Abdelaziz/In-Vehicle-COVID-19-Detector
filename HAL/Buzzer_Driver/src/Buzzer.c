/*
 * HR_sensor.c
 *
 *  Created on: May 9, 2021
 *      Author: mas
 */

#include "Buzzer.h"
#include "Buzzer_lcfg.h"
#include "Buzzer_cfg.h"
#include "GPIO.h"
#include "timer.h"
#include "timer_lcfg.h"

void Buzzer_Init(void)
{
	uint8_t counter=0;
	uint8_t state=0;
	uint8_t PinActualNumber=0;
	uint16_t i;

	for( counter = 0 ; counter < NUMBER_OF_CONFIGURED_BUZZER ; counter++ )
	{
		state = Buzzer_ConfigArray[counter].buzzer_state;

		if( Buzzer_ConfigArray[counter].buzzer_mode == DIMMING_MODE)
		{

			if( state == BUZZER_ON )
			{
				for(i=0; i<2000; i++)
				{

					CC1R_SetValue(TIMER2_, Buzzer_ConfigArray[counter].buzzer_id , i);

					delay_ms(2);

				}

				//for(j=0; j<1000000; j++);

				for(i=2000; i>0; i--)
				{
					CC1R_SetValue(TIMER2_, Buzzer_ConfigArray[counter].buzzer_id , i);

					delay_ms(2);
				}
			}
			else
			{
				CC1R_SetValue(TIMER2_, Buzzer_ConfigArray[counter].buzzer_id , 0);
			}
		}
		else
		{
			if( state == BUZZER_ON )
			{
				GPIO_WriteOutputPin(Buzzer_ConfigArray[counter].buzzer_id , 1);
			}
			else
			{
				GPIO_WriteOutputPin(Buzzer_ConfigArray[counter].buzzer_id , 0);
			}
		}
	}

}





void Buzzer_cmd(uint8_t buzzer_id , uint8_t mode, uint8_t buzzer_state)
{
	uint8_t i;
	if( mode == DIMMING_MODE)
	{

		if( buzzer_state == BUZZER_ON )
		{
			/*for(i=0; i<2000; i++)
			{

				CC1R_SetValue(TIMER2_, buzzer_id , i);

				delay_ms(2);

			}
*/
			//for(j=0; j<1000000; j++);

			for(i=2000; i>0; i--)
			{
				CC1R_SetValue(TIMER2_, buzzer_id , i);

				delay_ms(2);
			}
		}
		else
		{
			CC1R_SetValue(TIMER2_, buzzer_id , 0);
		}
	}
	else
	{
		if( buzzer_state == BUZZER_ON )
		{
			GPIO_WriteOutputPin(buzzer_id , 1);
		}
		else
		{
			GPIO_WriteOutputPin(buzzer_id , 0);
		}
	}
}
