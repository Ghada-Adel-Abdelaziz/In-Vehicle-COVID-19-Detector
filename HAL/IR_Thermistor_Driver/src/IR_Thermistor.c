/*
 * IR_Thermistor.c
 *
 *  Created on: May 25, 2021
 *      Author: Toaa & Ghada
 */

#include "IR_Thermistor.h"

#define TRANSFER_COMPLETE		1
#define TRQANSFER_NOT_COMPLETE	0

typedef enum
{
	MLX90614_IDLE,
	MLX90614_READING,
	MLX90614_WRITING
}SENSOR_STATE;


static SENSOR_DATA_TYPE_TO_READ Data_Tobe_Read = NULL;

static uint8_t Data[2];
static float * Sensor_Reading = 0;

static uint8_t Transfer_Complete_Flag = TRQANSFER_NOT_COMPLETE;


static SENSOR_STATE Current_Sensor_State = MLX90614_IDLE;


void mlx90614_Sensor_Init(void)
{
	Current_Sensor_State = MLX90614_IDLE;
}

void Sensor_Data_Transfer_Complete_CallBack(void)
{
	if( Current_Sensor_State ==  MLX90614_READING )
	{
		Current_Sensor_State = MLX90614_IDLE;

		switch( Data_Tobe_Read )
		{
		case OBJECT_TEMPERATURE:

			Data_Tobe_Read = NULL;

			*Sensor_Reading = ( Data[1] << 8 ) | Data[0];

			*Sensor_Reading = *Sensor_Reading * 0.02 - 273.15;    // to convert from kelvin to celsius

			break;

		case AMBIENT_TEMPERATURE:

			Data_Tobe_Read = NULL;

			break;

		case DEVICE_ID1:

		case DEVICE_ID2:

			*Sensor_Reading = ( Data[1] << 8 ) | Data[0];
			Data_Tobe_Read = NULL;

			break;
		}

		/* Do somthing */
	}
	else if( Current_Sensor_State ==  MLX90614_WRITING )
	{
		Current_Sensor_State = MLX90614_IDLE;

		/* Do Something */
	}
}

SENSOR_ERROR_STATUS mlx90614_Request_Reading(uint8_t sensor_preph_id , float * sensor_data , SENSOR_DATA_TYPE_TO_READ sensor_data_type)
{
	I2C_ERROR_STATUS sensor_returnValue = SENSOR_E_OK;
	I2C_ERROR_STATUS i2c_returnValue = I2C_E_OK;



	switch( sensor_data_type )
	{
	case OBJECT_TEMPERATURE:

		i2c_returnValue = I2C_ReceiveDataASync(sensor_preph_id , Data , MLX90614_SIZE_OF_DATA , MLX90614_ADDR , MLX90614_OBJECT_GET_TEMP_CMD);

		if( i2c_returnValue == I2C_E_OK )
		{
			Sensor_Reading = sensor_data;
			sensor_returnValue = SENSOR_E_OK;

			Current_Sensor_State = MLX90614_READING;

			Data_Tobe_Read = OBJECT_TEMPERATURE;
		}
		else
		{
			sensor_returnValue = SENSOR_E_NOT_OK;
		}
		break;



	case AMBIENT_TEMPERATURE:

		i2c_returnValue = I2C_ReceiveDataASync(sensor_preph_id , Data , MLX90614_SIZE_OF_DATA , MLX90614_ADDR , MLX90614_AMBIENT_GET_TEMP_CMD);

		if( i2c_returnValue == I2C_E_OK )
		{
			Sensor_Reading = sensor_data;
			sensor_returnValue = SENSOR_E_OK;

			Current_Sensor_State = MLX90614_READING;

			Data_Tobe_Read = AMBIENT_TEMPERATURE;
		}
		else
		{
			sensor_returnValue = SENSOR_E_NOT_OK;
		}
		break;

	case DEVICE_ID1:

		i2c_returnValue = I2C_ReceiveDataASync(sensor_preph_id , Data , MLX90614_SIZE_OF_DATA , MLX90614_ADDR , MLX90614_GET_ID1_CMD);

		Sensor_Reading = sensor_data;
		sensor_returnValue = SENSOR_E_OK;

		Current_Sensor_State = MLX90614_READING;

		Data_Tobe_Read = DEVICE_ID1;

		break;

	case DEVICE_ID2:

		i2c_returnValue = I2C_ReceiveDataASync(sensor_preph_id , Data , MLX90614_SIZE_OF_DATA , MLX90614_ADDR , MLX90614_GET_ID2_CMD);

		Sensor_Reading = sensor_data;
		sensor_returnValue = SENSOR_E_OK;

		Current_Sensor_State = MLX90614_READING;

		Data_Tobe_Read = DEVICE_ID2;

		break;
	}


	return sensor_returnValue;

}


SENSOR_ERROR_STATUS mlx90614_Request_Writing(uint8_t sensor_preph_id , uint8_t *data , SENSOR_DATA_TYPE_TO_WRITE sensor_data_type)
{
	I2C_ERROR_STATUS sensor_returnValue = SENSOR_E_OK;
	I2C_ERROR_STATUS i2c_returnValue = I2C_E_OK;

	switch( sensor_data_type )
	{
	case CONFIG_DATA:

		i2c_returnValue = I2C_SendDataAsync( sensor_preph_id ,  data , MLX90614_SIZE_OF_DATA , MLX90614_ADDR , MLX90614_CONFIG_REG_ACCESS_CMD);

		if( i2c_returnValue ==  I2C_E_OK)
		{
			sensor_returnValue = SENSOR_E_OK;

			Current_Sensor_State = MLX90614_WRITING;
		}
		else
		{
			sensor_returnValue = SENSOR_E_NOT_OK;
		}
		break;

	case SENSOR_ADD:

		i2c_returnValue = I2C_SendDataAsync( sensor_preph_id ,  data , MLX90614_SIZE_OF_DATA , MLX90614_ADDR , MLX90614_SLAVE_ADDR_CONFIG_REG_ACCESS_CMD);

		if( i2c_returnValue ==  I2C_E_OK)
		{
			sensor_returnValue = SENSOR_E_OK;

			Current_Sensor_State = MLX90614_WRITING;
		}
		else
		{
			sensor_returnValue = SENSOR_E_NOT_OK;
		}

		break;
	}

	return sensor_returnValue;
}
