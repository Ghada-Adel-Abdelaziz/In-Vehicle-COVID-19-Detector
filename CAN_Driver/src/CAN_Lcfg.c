/*
 *  CAN_Lcfg.c
 *
 *  Created on: April 23, 2021
 *      Author: Ghada & Toaa
 */


CAN_msg CAN_ConfigArray[ NUMBER_OF_CONFIGURED_I2C ]=
{
		{id, data[8], len, format, type, u8ActiveFlag, call back func },
};

filter_type CAN_FilterArray [ CANHANDLER_u8MAXFILTERNUMBERS ]=
{
		{u8Id, u8Frame, u8Type, call back func},
};