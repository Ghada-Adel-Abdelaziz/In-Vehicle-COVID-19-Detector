/*
 * HR_sensor.c
 *
 *  Created on: May 9, 2021
 *      Author: Ghada & Toaa
 */

#include "ADC.h"
#include "HR_sensor.h"
#include "HR_sensor_lcfg.h"

static uint16_t HR_sensor_reading = 0;
static SENSOR_STATUS HR_Status = IDLE;


//void Conversion_Complete_CallBack(void);


void Conversion_Complete_CallBack(void)
{
	HR_sensor_reading = ADC_getValue(HR_Sensor_ConfigArray->Sensor_chID);

	if( HR_Status == ENABLED )
	{
		ADC_SoftwareStartConv(HR_Sensor_ConfigArray->Sensor_PrephID);     // to start conversion again
	}
}


void HR_Sensor_Init(void)
{

	HR_Sensor_Cmd(HR_Sensor_ConfigArray->Sensor_PrephID , IDLE);
}


void HR_Sensor_Read(uint16_t *data)
{
	*data = HR_sensor_reading;
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
	case IDLE:
		HR_Status = IDLE;
		break;

	case ENABLED:
		//HR_Sensor_Enable(sensor_PrephID);
		ADC_SoftwareStartConv(sensor_PrephID);
		HR_Status = ENABLED;
		break;
	}
}
