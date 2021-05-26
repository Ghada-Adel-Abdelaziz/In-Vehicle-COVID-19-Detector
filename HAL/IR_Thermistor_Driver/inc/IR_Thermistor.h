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


/* device prepheral ID */
#define MLX90614_PREPH_ID	I2C_1

#define MLX90614_SIZE_OF_DATA	2

#define MLX90614_ADDR        		    (uint8_t)( 0x5B << 1 )
#define MLX90614_OBJECT_TEMP_ADDR       0x07
#define MLX90614_AMBIENT_TEMP_ADDR      0x06


/* OPCODE DEFINES */
#define MLX90614_OP_RAM_ACCESS			0x02
#define MLX90614_OP_EEPROM_ACCESS		0x01
#define MLX90614_OP_SLEEP				0xC6


#define MLX90614_AMBIENT_GET_TEMP_CMD 		( ( MLX90614_OP_RAM_ACCESS << 4) | MLX90614_AMBIENT_TEMP_ADDR) /* ambient temperature */
#define MLX90614_OBJECT_GET_TEMP_CMD 		( ( MLX90614_OP_RAM_ACCESS << 4) | MLX90614_OBJECT_TEMP_ADDR) /* object 1 temperature */


/* EEPROM Register Address */

#define MLX90614_SLAVE_ADDR_CONFIG_REG_ADDR	    0x00
#define MLX90614_CONFIG_REG_ADDR				0x02
#define MLX90614_ID1_REG_ADDR					0x0E
#define MLX90614_ID2_REG_ADDR					0x0F



#define MLX90614_CONFIG_REG_ACCESS_CMD 					( ( MLX90614_OP_EEPROM_ACCESS << 4) | MLX90614_CONFIG_REG_ADDR )
#define MLX90614_SLAVE_ADDR_CONFIG_REG_ACCESS_CMD		( ( MLX90614_OP_EEPROM_ACCESS << 4) | MLX90614_SLAVE_ADDR_CONFIG_REG_ADDR )

#define MLX90614_GET_ID1_CMD							( ( MLX90614_OP_EEPROM_ACCESS << 4) | MLX90614_ID1_REG_ADDR )
#define MLX90614_GET_ID2_CMD							( ( MLX90614_OP_EEPROM_ACCESS << 4) | MLX90614_ID2_REG_ADDR )


void mlx90614_Sensor_Init(void);

SENSOR_ERROR_STATUS mlx90614_Request_Reading(uint8_t sensor_preph_id , float * sensor_data , SENSOR_DATA_TYPE_TO_READ sensor_data_type);
SENSOR_ERROR_STATUS mlx90614_Request_Writing(uint8_t sensor_preph_id , uint8_t *data , SENSOR_DATA_TYPE_TO_WRITE sensor_data_type);

#endif /* IR_THERMISTOR_H_ */
