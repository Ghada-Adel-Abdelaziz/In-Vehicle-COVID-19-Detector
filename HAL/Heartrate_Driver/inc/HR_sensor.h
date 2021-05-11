/*
 * HR_sensor.h
 *
 *  Created on: May 9, 2021
 *      Author: mas
 */

#ifndef HR_SENSOR_H_
#define HR_SENSOR_H_

#include "stm32f4xxx.h"

void HR_Sensor_Init(void);
void HR_Sensor_Read(uint8_t sensor_id , uint16_t *data);



#endif /* HR_SENSOR_H_ */
