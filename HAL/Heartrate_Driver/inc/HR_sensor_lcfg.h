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
     uint8_t Sensor_PrephID;
     uint8_t Sensor_chID;
} HR_Sensor_Config_t;



#define HR_SENSOR_CH_ID   ADC_Channel_1


#define HR_SENSOR_PREPH_ID    ADC_1




extern HR_Sensor_Config_t HR_Sensor_ConfigArray[NUMBER_OF_CONFIGURED_HR_SENSORS];
