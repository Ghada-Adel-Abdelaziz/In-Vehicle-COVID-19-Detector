/*
 * HR_sensor_lcdg.h
 *
 *  Created on: May 9, 2021
 *      Author: mas
 */

#include "ADC_Lcfg.h"
#include "HR_sensor_cfg.h"


typedef struct
{
     uint8_t sensor_id;
} HR_Sensor_Config_t;



#define SENSOR1_ID   ADC_Channel_1
#define SENSOR2_ID   ADC_Channel_2
#define SENSOR3_ID   ADC_Channel_3




extern HR_Sensor_Config_t HR_Sensor_ConfigArray[NUMBER_OF_CONFIGURED_HR_SENSORS];
