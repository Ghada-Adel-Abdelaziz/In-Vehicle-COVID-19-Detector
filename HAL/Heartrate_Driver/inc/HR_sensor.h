/*
 * HR_sensor.h
 *
 *  Created on: May 9, 2021
 *      Author: mas
 */

#ifndef HR_SENSOR_H_
#define HR_SENSOR_H_

#include "stm32f4xxx.h"


typedef enum
{
	HR_DISABLE,
	HR_ENABLE
}SENSOR_STATUS;


void HR_Sensor_Init(void);
void HR_Sensor_Read( uint16_t *data);
//void HR_Sensor_Enable(uint8_t sensor_PrephID);
void HR_Sensor_Cmd(uint8_t sensor_PrephID , SENSOR_STATUS sensor_cmd);
void HR_Sensor_Manage(void);



#endif /* HR_SENSOR_H_ */

