/******************************************************************************
 * Module: 		GPIO
 * File Name: 	GPIO_Lcfg.c
 * Description: Link time Configuration Source file for
 * 				STM32F407 Microcontroller
 * Author: 		Toqa&Ghada
 * Date:		9/1/2021
 ******************************************************************************/

#include "GPIO_lcfg.h"
#include "GPIO.h"
/* GPIO Pre-Compile Configuration Header file */
#include "GPIO_cfg.h"



/************************************************************************
 *      creating instance and initializing configuration structure      *
 ************************************************************************/

GPIO_pinconfig_t GPIO_PinConfigArray[NUMBER_OF_CONFIGURED_PINS] =
/*	PinNumber   PinMode        PinSpeed       PinPuPdControl    PinOPType   AltFunMode */
{
	{LED0,   GPIO_MODE_OUT,  GPIO_SPEED_MEDIUM,  GPIO_NO_PUPD,  GPIO_TYPE_PP,   0},
	{LED1,   GPIO_MODE_OUT,  GPIO_SPEED_MEDIUM,  GPIO_NO_PUPD,  GPIO_TYPE_PP,   0},
	//{LED2,   GPIO_MODE_OUT,  GPIO_SPEED_MEDIUM,  GPIO_NO_PUPD,  GPIO_TYPE_PP,   0},
//	{LED3,   GPIO_MODE_OUT,  GPIO_SPEED_MEDIUM,  GPIO_NO_PUPD,  GPIO_TYPE_PP,   0},
	{trial_Tx,   GPIO_MODE_ALTFN,  GPIO_SPEED_MEDIUM,  GPIO_NO_PUPD,  GPIO_TYPE_PP, 7},
	{trial_Rx,   GPIO_MODE_ALTFN,  GPIO_SPEED_MEDIUM,  GPIO_NO_PUPD,  GPIO_TYPE_PP, 7},

};
