/*
 * HR_sensor.c
 *
 *  Created on: May 9, 2021
 *      Author: Toaa & Ghada
 */

#include "Buzzer.h"
#include "Buzzer_lcfg.h"
#include "Buzzer_cfg.h"

#include "timer.h"

#include "Common_Macros.h"

#define TRUE	1
#define FALSE   0
#define BUZZER_BUSY		1
#define BUZZER_NOT_BUSY	0

static uint8_t Nmber_Of_Peeps = 0;
static uint8_t Peep_Time = 0;
static uint8_t OFF_Time = 0;
static BUZZER_POWER_RANGE Power = 0;
static uint8_t Buzzer_New_Req = 0;

static uint32_t ONtime_Counter = 0;
static uint32_t OFFtime_Counter = 0;
static uint16_t Peep_Counter = 0;

static uint8_t Buzzer_Req_State = BUZZER_NOT_BUSY;

static void(*Fptr)(void);


typedef enum
{
	IDLE,
	BUZZER_ENABLE,
	BUZZER_DISABLE,
	BUZZER_ON_TIME,
	BUZZER_OFF_TIME,
}BUZZER_STATE;

static BUZZER_STATE Buzzer_State = IDLE;

void Buzzer_Init(void)
{
//	Nmber_Of_Peeps = 0;
//	Peep_Time = 0;
//	OFF_Time = 0;
//	Power = 0;
//	Buzzer_New_Req = FALSE;
//
//	PWM_Set_Duty(BUZZER1_PREPH_ID , 0);

	Timer_Init();
	TIM_OC_Init();
	TIM_IRQConfig(28, ENABLE);    // 28 is the number of timer2 in the NVIC
	TIM_IRQConfig(29, ENABLE);	  // 29 is the number of timer2 in the NVIC
	TIM_IntControl(TIMER2_, TIM_UPDATE_EVENT_IE, ENABLE);  // enable interrupt when timer2 overflow
	TIM_IntControl(TIMER3_, TIM_UPDATE_EVENT_IE, ENABLE);  // enable interrupt when timer3 overflow
	Timer_Cmd(TIMER2_, START);    // to start the timer2
	Timer_Cmd(TIMER3_, START);    // to start the timer3

	Fptr = Buzzer_ConfigArray[0].Buzzer_Complete_Cycle_Fptr ;
}

///////////////////////////////////////////////////////////////////////////


REQUEST_STATE Buzzer_Req( uint8_t peep_time , uint8_t off_time , uint8_t num_of_peeps ,  BUZZER_POWER_RANGE buzzer_power)
{

	REQUEST_STATE returnValue = BUZZER_E_OK;

	if( Buzzer_Req_State == BUZZER_NOT_BUSY )
	{
		Nmber_Of_Peeps = num_of_peeps;
		Peep_Time = peep_time;

		OFF_Time = off_time;

		Power = buzzer_power;

		Buzzer_New_Req = TRUE;
		Buzzer_Req_State = BUZZER_BUSY;
	}
	else
	{
		returnValue = BUZZER_E_NOT_OK;
	}

	return returnValue;
}


void Buzzer_Manage(void)
{
	uint32_t period = 0;
	switch( Buzzer_State )
	{
	case IDLE:

		//CC1R_SetValue( BUZZER1_PREPH_ID , 0);   // set duty cycle to 0

		PWM_Set_Duty(BUZZER1_PREPH_ID , 0);      // 0% duty cycle
		if( Buzzer_New_Req == TRUE)
		{
			Buzzer_New_Req = FALSE;
			Buzzer_State = BUZZER_ENABLE;
		}
		break;

	case BUZZER_ENABLE:

		switch( Power )
		{
		case LOW_POWER:

			//			TIM_Get_Period(BUZZER1_PREPH_ID, period);
			//			CC1R_SetValue( BUZZER1_PREPH_ID ,  period/4);

			PWM_Set_Duty(BUZZER1_PREPH_ID , 25);   // 25% duty cycle

			Buzzer_State = BUZZER_ON_TIME;
			break;

		case MEDIUM_POWER:

			//			TIM_Get_Period(BUZZER1_PREPH_ID, period);
			//			CC1R_SetValue( BUZZER1_PREPH_ID ,  period/2);

			PWM_Set_Duty(BUZZER1_PREPH_ID , 50); 	// 50% duty cycle

			Buzzer_State = BUZZER_ON_TIME;
			break;

		case MAXIMUM_POWER:

			//			TIM_Get_Period(BUZZER1_PREPH_ID, period);
			//			CC1R_SetValue( BUZZER1_PREPH_ID ,  period);

			PWM_Set_Duty(BUZZER1_PREPH_ID , 100);    // 100% duty cycle

			Buzzer_State = BUZZER_ON_TIME;
			break;
		}
		break;

		case BUZZER_ON_TIME:

			ONtime_Counter ++; 			// consider this task is called every 10 ms
			if( ( ( ONtime_Counter * 10 ) / 1000 ) >= Peep_Time)   // consider on time is an integer seconds
			{
				ONtime_Counter = 0;
				Buzzer_State = BUZZER_DISABLE;
			}
			break;

		case BUZZER_DISABLE:

			//CC1R_SetValue( BUZZER1_PREPH_ID , 0);   // set duty cycle to 0

			PWM_Set_Duty(BUZZER1_PREPH_ID , 0);
			Buzzer_State = BUZZER_OFF_TIME;
			break;

		case BUZZER_OFF_TIME:
			OFFtime_Counter ++; 			// consider this task is called every 10 ms
			if( ( ( OFFtime_Counter * 10 ) / 1000 ) >= OFF_Time)   // consider on time is an integer seconds
			{
				OFFtime_Counter = 0;

				Peep_Counter ++;
				if( Peep_Counter ==  Nmber_Of_Peeps)
				{
					Peep_Counter = 0;
					Buzzer_State = IDLE;

					Buzzer_Req_State = BUZZER_NOT_BUSY;

					Fptr();
				}
				else
				{
					Buzzer_State = BUZZER_ENABLE;
				}

			}
			break;
	}
}

