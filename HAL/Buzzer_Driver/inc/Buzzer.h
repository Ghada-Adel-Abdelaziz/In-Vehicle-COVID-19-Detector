/*
 * HR_sensor.h
 *
 *  Created on: May 9, 2021
 *      Author: Toaa & Ghada
 */

#ifndef HR_SENSOR_H_
#define HR_SENSOR_H_

#include "stm32f4xxx.h"

typedef enum
{
	LOW_POWER,
	MEDIUM_POWER,
	MAXIMUM_POWER
}BUZZER_POWER_RANGE;

typedef enum
{
    BUZZER_E_OK,
	BUZZER_E_NOT_OK
}REQUEST_STATE;

void Buzzer_Init(void);

REQUEST_STATE Buzzer_Req( uint8_t peep_time , uint8_t off_time , uint8_t num_of_peeps ,  BUZZER_POWER_RANGE buzzer_power);
void Buzzer_Manage(void);


#endif /* HR_SENSOR_H_ */
