/*
 * HR_sensor.c
 *
 *  Created on: May 9, 2021
 *      Author: mas
 */

#include "ADC.h"

void HR_Sensor_Init(void)
{

}


void HR_Sensor_Read(uint8_t sensor_id , uint16_t *data)
{
	*data = ADC_getValue(sensor_id);
}
