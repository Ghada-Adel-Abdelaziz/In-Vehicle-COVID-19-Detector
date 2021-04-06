/*
 * I2C_lcfg.c
 *
 *  Created on: Apr 2, 2021
 *      Author: esraa
 */

#include "I2C_lcfg.h"
#include "I2C_cfg.h"


extern I2C_Config_t I2C_ConfigArray[NUMBER_OF_CONFIGURED_I2C]=
{
		{I2C_1, I2C_Mode_I2C, I2C_DutyCycle_2, I2C_Ack_Enable, I2C_Address_7bit},
};
