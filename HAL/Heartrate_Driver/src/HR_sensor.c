/*
 * HR_sensor.c
 *
 *  Created on: May 9, 2021
 *      Author: mas
 */

#include "ADC.h"
#include "HR_sensor.h"
#include "HR_sensor_lcfg.h"


#define HR_SENSOR_ON			1
#define HR_SENSOR_OFF			0
#define CONVERSION_DONE 		1
#define CONVERSION_NOT_DONE		0

static uint16_t HR_sensor_reading = 0;



typedef enum
{
	IDLE,
	HR_ON,
	HR_OFF,
}HR_STATE;

static HR_STATE HR_State = IDLE;
static uint8_t HR_Sensor_Status = HR_SENSOR_OFF;
static uint8_t HR_Sensor_Conversion_Complete_FB = CONVERSION_NOT_DONE;

//void Conversion_Complete_CallBack(void);


void Conversion_Complete_CallBack(void)
{
	HR_sensor_reading = ADC_getValue(HR_Sensor_ConfigArray->Sensor_chID);

	HR_Sensor_Conversion_Complete_FB =  CONVERSION_DONE;
}


void HR_Sensor_Init(void)
{
	ADC_Init();
	ADC_RegularChannelConfig();

	HR_Sensor_Cmd(HR_Sensor_ConfigArray->Sensor_PrephID , HR_DISABLE);
}


void HR_Sensor_Read(uint16_t *data)
{
	*data = ( 3 * HR_sensor_reading) / 1023;
}



/*void HR_Sensor_Enable(uint8_t sensor_PrephID)
{
	ADC_SoftwareStartConv(sensor_PrephID);        // start conversion will start the ch on the rank (consider we
have only one ch)
}*/

void HR_Sensor_Cmd( uint8_t sensor_PrephID , SENSOR_STATUS sensor_cmd)
{
	switch(sensor_cmd)
	{
	case HR_ENABLE:

		HR_Sensor_Status = HR_SENSOR_ON;

		break;

	case HR_DISABLE:

		HR_Sensor_Status = HR_SENSOR_OFF;

		break;
	}
}



void HR_Sensor_Manage(void)
{
	switch( HR_State )
	{
	case IDLE:

		if( HR_Sensor_Status == HR_SENSOR_ON )
		{
			ADC_SoftwareStartConv(HR_Sensor_ConfigArray->Sensor_PrephID);     // to start conversion again
			HR_State = HR_ENABLE;
		}

		break;

	case HR_ON:

		if( HR_Sensor_Conversion_Complete_FB ==  CONVERSION_DONE)
		{
			HR_Sensor_Conversion_Complete_FB = CONVERSION_NOT_DONE;

			HR_State = IDLE;
		}
		break;

	case HR_OFF:

		break;
	}
}



