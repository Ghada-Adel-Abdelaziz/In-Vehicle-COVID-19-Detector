/******************************************************************************
 * Module: 		I2C
 * File Name: 	I2C_lcfg.c
 * Description: ADC Source file for
 * 				STM32F407 Microcontroller
 * Author: 		Toqa & Ghada
 * Date:		26/3/2021
 ******************************************************************************/

#include "I2C_lcfg.h"
#include "I2C_cfg.h"
#include "I2C_stubs.h"

extern void Sensor_Data_Transfer_Complete_CallBack(void);

 I2C_Config_t I2C_ConfigArray[NUMBER_OF_CONFIGURED_I2C]=
{
		{I2C_1, I2C_Mode_I2C, I2C_DutyCycle_2, I2C_Ack_Enable, I2C_Address_7bit, Sensor_Data_Transfer_Complete_CallBack},
};
