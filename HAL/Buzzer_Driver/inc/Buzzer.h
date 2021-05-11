/*
 * HR_sensor.h
 *
 *  Created on: May 9, 2021
 *      Author: mas
 */

#ifndef HR_SENSOR_H_
#define HR_SENSOR_H_

#include "stm32f4xxx.h"

//typedef enum
//{
//	NORMAL_MODE,
//	DIMMING_MODE
//};

void Buzzer_Init(void);
void Buzzer_cmd(uint8_t buzzer_id , uint8_t mode, uint8_t buzzer_state);
//void Buzzer_OFF(void);




#endif /* HR_SENSOR_H_ */
