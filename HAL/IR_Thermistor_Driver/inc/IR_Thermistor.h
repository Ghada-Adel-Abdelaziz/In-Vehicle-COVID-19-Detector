/*
 * IR_Thermistor.h
 *
 *  Created on: May 25, 2021
 *      Author: Toaa & Ghada
 */

#ifndef IR_THERMISTOR_H_
#define IR_THERMISTOR_H_

#include "I2C_lcfg.h"
#include "I2C_cfg.h"
#include "I2C.h"

#include "stm32f4xxx.h"


typedef enum
{
	NULL,
	OBJECT_TEMPERATURE,
	AMBIENT_TEMPERATURE,
	DEVICE_ID1,
	DEVICE_ID2
}SENSOR_DATA_TYPE_TO_READ;

typedef enum
{
	CONFIG_DATA,
	SENSOR_ADD,
}SENSOR_DATA_TYPE_TO_WRITE;

typedef enum
{
    SENSOR_E_OK,
	SENSOR_E_NOT_OK
}SENSOR_ERROR_STATUS;




void mlx90614_Sensor_Init(void);

SENSOR_ERROR_STATUS mlx90614_Request_Reading(uint8_t sensor_preph_id , float * sensor_data , SENSOR_DATA_TYPE_TO_READ sensor_data_type);
SENSOR_ERROR_STATUS mlx90614_Request_Writing(uint8_t sensor_preph_id , uint8_t *data , SENSOR_DATA_TYPE_TO_WRITE sensor_data_type);

#endif /* IR_THERMISTOR_H_ */
